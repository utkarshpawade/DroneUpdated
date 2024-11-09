#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from waypoint_navigation.srv import GetWaypoints
from geometry_msgs.msg import Pose, Point, PoseArray

class WaypointService(Node):
    def __init__(self):
        super().__init__('waypoint_service')
        
        # Create a service for getting waypoints
        self.srv = self.create_service(GetWaypoints, 'waypoints', self.get_waypoints_callback)
        
        # Define a list of predefined waypoints (Pose with x, y, z positions)
        self.waypoints = [
            Pose(position=Point(x=2.0, y=2.0, z=27.0)),
            Pose(position=Point(x=2.0, y=-2.0, z=27.0)),
            Pose(position=Point(x=-2.0, y=-2.0, z=27.0)),
            Pose(position=Point(x=-2.0, y=2.0, z=27.0)),
            Pose(position=Point(x=1.0, y=1.0, z=27.0))
        ]

    def get_waypoints_callback(self, request, response):
        """Handle incoming requests for waypoints."""
        self.get_logger().info('Received a request for waypoints.')

        # Initialize PoseArray in the response
        response.waypoints = PoseArray()
        response.waypoints.header.stamp = self.get_clock().now().to_msg()

        # Fill the PoseArray with the predefined waypoints
        response.waypoints.poses = self.waypoints

        return response


def main(args=None):
    rclpy.init(args=args)

    waypoint_service = WaypointService()

    try:
        rclpy.spin(waypoint_service)
    except KeyboardInterrupt:
        waypoint_service.get_logger().info('KeyboardInterrupt, shutting down.')
    finally:
        waypoint_service.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
