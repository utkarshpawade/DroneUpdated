#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from waypoint_navigation.action import NavToWaypoint
from waypoint_navigation.srv import GetWaypoints


class WayPointClient(Node):

    def __init__(self):
        super().__init__('waypoint_client')
        self.goals = []
        self.goal_index = 0

        # Create an action client for the action 'NavToWaypoint'.
        self.action_client = ActionClient(self, NavToWaypoint, 'waypoint_navigation')

        # Create a client for the service 'GetWaypoints'.
        self.cli = self.create_client(GetWaypoints, 'waypoints')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

        # Create a request object for GetWaypoints service.
        self.request = GetWaypoints.Request()

    ### Action client functions

    def send_goal(self, waypoint):
        # Create a NavToWaypoint goal object.
        goal_msg = NavToWaypoint.Goal()  # Initialize the goal message
        goal_msg.waypoint.position.x = waypoint[0]
        goal_msg.waypoint.position.y = waypoint[1]
        goal_msg.waypoint.position.z = waypoint[2]

        # Wait for the action server to be available.
        self.action_client.wait_for_server()

        self.send_goal_future = self.action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('Goal accepted, waiting for result...')
            self.get_result_future = response.get_result_async()  # Use result handle directly
            self.get_result_future.add_done_callback(self.get_result_callback)
        except Exception as e:
            self.get_logger().error(f'Exception while sending goal: {e}')

    def get_result_callback(self, future):
        result = future.result().result  # Access the result of the action
        self.get_logger().info(f'Result: {result.hov_time}')  # Replace hov_time if incorrect

        self.goal_index += 1

        if self.goal_index < len(self.goals):
            self.send_goal(self.goals[self.goal_index])
        else:
            self.get_logger().info('All waypoints have been reached successfully')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback  # Access the feedback
        x = feedback.current_waypoint.pose.position.x
        y = feedback.current_waypoint.pose.position.y
        z = feedback.current_waypoint.pose.position.z
        t = feedback.current_waypoint.header.stamp.sec
        self.get_logger().info(f'Received feedback! The current waypoint position is: {x}, {y}, {z}')
        self.get_logger().info(f'Max time inside sphere: {t}')

    # Service client functions

    def send_request(self):
        return self.cli.call_async(self.request)

    def receive_goals(self):
        future = self.send_request()
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response is not None:
            self.get_logger().info('Waypoints received by the action client')

            for pose in response.waypoints.poses:
                waypoints = [pose.position.x, pose.position.y, pose.position.z]
                self.goals.append(waypoints)
                self.get_logger().info(f'Waypoints: {waypoints}')

            if self.goals:
                self.send_goal(self.goals[0])  # Start with the first goal


def main(args=None):
    rclpy.init(args=args)

    waypoint_client = WayPointClient()
    waypoint_client.receive_goals()

    try:
        rclpy.spin(waypoint_client)
    except KeyboardInterrupt:
        waypoint_client.get_logger().info('KeyboardInterrupt, shutting down.\n')
    finally:
        waypoint_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

