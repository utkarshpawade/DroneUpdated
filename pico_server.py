#!/usr/bin/env python3

import time
import math
import numpy as np
from tf_transformations import euler_from_quaternion

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from waypoint_navigation.action import NavToWaypoint
from swift_msgs.msg import SwiftMsgs
from geometry_msgs.msg import PoseArray
from pid_msg.msg import PIDTune, PIDError
from nav_msgs.msg import Odometry

class WayPointServer(Node):

    def __init__(self):
        super().__init__('waypoint_server')

        # Callback groups for safe concurrency
        self.pid_callback_group = ReentrantCallbackGroup()
        self.action_callback_group = ReentrantCallbackGroup()

        # Initialize variables
        self.time_inside_sphere = 0
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None
        self.dtime = 0
        self.drone_position = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)  # x, y, z, yaw
        self.setpoint = np.array([0.0, 0.0, 27.0, 0.0], dtype=np.float64)  # Target position

        self.cmd = SwiftMsgs()
        self.reset_command()

        # PID values (Kp, Ki, Kd) for pitch, roll, yaw, throttle
        self.Kp = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.Ki = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.Kd = np.array([0.0, 0.0, 0.0, 0.0], dtype=np.float64)

        self.pid_error = PIDError()
        self.sample_time = 0.060  # 60 ms

        # Publishers and subscribers
        self.command_pub = self.create_publisher(SwiftMsgs, '/drone_command', 10)
        self.pid_error_pub = self.create_publisher(PIDError, '/pid_error', 10)
        self.create_subscription(PoseArray, '/whycon/poses', self.whycon_callback, 1)
        self.create_subscription(PIDTune, '/throttle_pid', self.altitude_set_pid, 1)
        # self.create_subscription(Odometry, '/rotors/odometry', self.odometry_callback, 10)

        # Action server for waypoint navigation
        self._action_server = ActionServer(
            self,
            NavToWaypoint,
            'waypoint_navigation',
            self.execute_callback,
            callback_group=self.action_callback_group
        )

        self.arm()
        self.timer = self.create_timer(self.sample_time, self.pid, callback_group=self.pid_callback_group)

    def reset_command(self):
        """Reset the drone command values."""
        self.cmd.rc_roll = 1500
        self.cmd.rc_pitch = 1500
        self.cmd.rc_yaw = 1500
        self.cmd.rc_throttle = 1500

    def disarm(self):
        """Disarm the drone by setting all channels to 1000."""
        self.cmd.rc_roll = 1000
        self.cmd.rc_yaw = 1000
        self.cmd.rc_pitch = 1000
        self.cmd.rc_throttle = 1000
        self.cmd.rc_aux4 = 1000
        self.command_pub.publish(self.cmd)

    def arm(self):
        """Arm the drone by setting throttle to neutral and enabling AUX4."""
        self.disarm()
        self.reset_command()
        self.cmd.rc_aux4 = 2000
        self.command_pub.publish(self.cmd)

    def whycon_callback(self, msg):
        """Update drone's position from WhyCon system."""
        if not msg.poses:
            self.get_logger().warn("PoseArray received with no poses.")
            return

        self.drone_position[0] = msg.poses[0].position.x
        self.drone_position[1] = msg.poses[0].position.y
        self.drone_position[2] = msg.poses[0].position.z
        self.dtime = msg.header.stamp.sec

    def altitude_set_pid(self, alt):
        """Tune the altitude PID controller."""
        self.Kp[1] = alt.kp
        self.Ki[1] = alt.ki * 0.001
        self.Kd[1] = alt.kd

    def odometry_callback(self, msg):
        """Update drone's orientation from odometry data."""
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        roll, pitch, yaw = euler_from_quaternion(orientation_list)
        self.drone_position[3] = math.degrees(yaw)

    def pid(self):
        """PID control algorithm for pitch, roll, yaw, and throttle."""
        # PID for throttle
        error_throttle = self.setpoint[2] - self.drone_position[2]
        self.cmd.rc_throttle = int(1500 + (self.Kp[1] * error_throttle))

        # PID for roll and pitch
        error_roll = self.setpoint[0] - self.drone_position[0]
        error_pitch = self.setpoint[1] - self.drone_position[1]
        self.cmd.rc_roll = int(1500 + (self.Kp[0] * error_roll))
        self.cmd.rc_pitch = int(1500 + (self.Kp[1] * error_pitch))

        # Publish the command and error for monitoring
        self.command_pub.publish(self.cmd)
        self.pid_error_pub.publish(self.pid_error)

    def execute_callback(self, goal_handle):
        """Action server callback to handle navigation to waypoints."""
        self.get_logger().info('Executing goal...')
        self.setpoint = np.array([
            goal_handle.request.waypoint.position.x,
            goal_handle.request.waypoint.position.y,
            goal_handle.request.waypoint.position.z,
            0.0  # Assuming yaw isn't set by waypoint
        ], dtype=np.float64)
        
        self.max_time_inside_sphere = 0
        self.point_in_sphere_start_time = None

        # Feedback object for NavToWaypoint action
        feedback_msg = NavToWaypoint.Feedback()

        while True:
            feedback_msg.current_waypoint.pose.position.x = self.drone_position[0]
            feedback_msg.current_waypoint.pose.position.y = self.drone_position[1]
            feedback_msg.current_waypoint.pose.position.z = self.drone_position[2]
            feedback_msg.current_waypoint.header.stamp.sec = self.dtime

            goal_handle.publish_feedback(feedback_msg)

            if self.is_drone_in_sphere(self.drone_position, goal_handle, 0.4):
                if self.point_in_sphere_start_time is None:
                    self.point_in_sphere_start_time = self.dtime

                self.time_inside_sphere = self.dtime - self.point_in_sphere_start_time

                if self.time_inside_sphere > self.max_time_inside_sphere:
                    self.max_time_inside_sphere = self.time_inside_sphere

                if self.max_time_inside_sphere >= 3:
                    break
            else:
                self.point_in_sphere_start_time = None

        goal_handle.succeed()

        # Create a result object and return the time hovered
        result = NavToWaypoint.Result()
        result.hov_time = self.dtime - goal_handle.request.waypoint.header.stamp.sec
        return result

    def is_drone_in_sphere(self, drone_pos, goal_handle, radius):
        """Check if the drone is within the target sphere."""
        distance_squared = np.sum(
            (drone_pos[:3] - np.array([
                goal_handle.request.waypoint.position.x,
                goal_handle.request.waypoint.position.y,
                goal_handle.request.waypoint.position.z
            ], dtype=np.float64)) ** 2
        )
        return distance_squared <= np.float64(radius) ** 2  # Ensure radius is a float


def main(args=None):
    rclpy.init(args=args)

    waypoint_server = WayPointServer()
    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        waypoint_server.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        waypoint_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
