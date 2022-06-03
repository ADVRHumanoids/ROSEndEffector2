#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rosee_msg.action import ROSEECommand


class RoseeSendGoal(Node):

    def __init__(self):
        super().__init__('rosee_send_goal')
        self._action_client = ActionClient(self, ROSEECommand, 'action_command')

    def send_goal(self):
        goal_msg = ROSEECommand.Goal()
        goal_msg.goal_action.seq = 0
        goal_msg.goal_action.stamp.sec = 0
        goal_msg.goal_action.stamp.nanosec = 0
        goal_msg.goal_action.action_name = 'trig'
        goal_msg.goal_action.action_type = 0
        goal_msg.goal_action.action_primitive_type = 0
        goal_msg.goal_action.selectable_items = ['finger_1']
        goal_msg.goal_action.percentage = 0.8
        goal_msg.goal_action.error_norm = 0.0
        
        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    action_client = RoseeSendGoal()

    future = action_client.send_goal()

    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()
