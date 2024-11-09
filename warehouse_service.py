#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseArray, PoseStamped
from swift_msgs.msg import SwiftMsgs
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

# Import the Waypoint service type (make sure you have the service defined correctly)
from waypoint_navigation.srv import GetWaypoints  # Ensure this is correctly defined

class WaypointService(Node):

    def __init__(self):
        super().__init__('waypoint_service')

        # PID gains (adjust these values based on your system tuning)
        self.Kp = [0.6, 0.6, 0.8]  # Roll, Pitch, Throttle
        self.Ki = [0.01, 0.01, 0.02]
        self.Kd = [0.3, 0.3, 0.5]

        # Store errors and drone state
        self.prev_error = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0]
        self.drone_position = [0.0, 0.0, 0.0, 0.0]  # x, y, z, yaw

        # Setpoint for the waypoint (default is the origin)
        self.setpoint = [0.0, 0.0, 0.0]

        # Callback groups for thread-safe handling
        self.callback_group = ReentrantCallbackGroup()

        # Publisher for drone commands
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)

        # Subscribers for sensor data
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 10)
        self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)

        # Timer to run the PID control loop
        self.timer = self.create_timer(0.1, self.pid_control, callback_group=self.callback_group)

        # Create the Waypoint service
        self.srv = self.create_service(Waypoint, 'waypoint_service', self.waypoint_callback)

    def whycon_callback(self, msg):
        """Update drone position from WhyCon marker."""
        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z

    def odometry_callback(self, msg):
        """Update drone orientation from odometry data."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.drone_position[3] = math.degrees(yaw)

    def waypoint_callback(self, request, response):
        """Handle the Waypoint service request."""
        self.get_logger().info(f"Received waypoint: x={request.x}, y={request.y}, z={request.z}")

        # Set the target waypoint
        self.setpoint = [request.x, request.y, request.z]

        # Respond with a success message
        response.success = True
        response.message = f"Waypoint set to ({request.x}, {request.y}, {request.z})"
        return response

    def pid_control(self):
        """Compute and publish the PID control commands."""
        error = [
            self.setpoint[0] - self.drone_position[0],  # Roll error (x-axis)
            self.setpoint[1] - self.drone_position[1],  # Pitch error (y-axis)
            self.setpoint[2] - self.drone_position[2]   # Throttle error (z-axis)
        ]

        # PID calculations for each axis
        pid_output = []
        for i in range(3):
            # Proportional term
            p_term = self.Kp[i] * error[i]

            # Integral term (with anti-windup)
            self.integral[i] += error[i]
            self.integral[i] = max(min(self.integral[i], 100), -100)  # Limit the integral
            i_term = self.Ki[i] * self.integral[i]

            # Derivative term
            d_term = self.Kd[i] * (error[i] - self.prev_error[i])

            # Compute the PID output
            pid_output.append(p_term + i_term + d_term)

            # Update the previous error
            self.prev_error[i] = error[i]

        # Construct the command message
        cmd = SwiftMsgs()
        cmd.rc_roll = 1500 + pid_output[0]  # Roll
        cmd.rc_pitch = 1500 + pid_output[1]  # Pitch
        cmd.rc_throttle = 1500 + pid_output[2]  # Throttle
        cmd.rc_yaw = 1500  # Default yaw

        # Publish the command
        self.command_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    waypoint_service = WaypointService()
    rclpy.spin(waypoint_service)

    # Clean up and shutdown
    waypoint_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
