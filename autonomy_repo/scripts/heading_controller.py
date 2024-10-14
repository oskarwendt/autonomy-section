#!/usr/bin/env python3
import numpy as np
import rclpy
from asl_tb3_lib.control import BaseHeadingController
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState

class HeadingController(BaseHeadingController):
    def __init__(self):
        super().__init__()
        self.kp = 2.0

    def compute_control_with_goal(self, state: TurtleBotState, goal: TurtleBotState) -> TurtleBotControl:
        heading_error = wrap_angle(goal.theta - state.theta)
        omega = self.kp * heading_error
        control_msg = TurtleBotControl()
        control_msg.omega = omega
        return control_msg

if __name__ == "__main__":
    rclpy.init()
    controller = HeadingController()
    rclpy.spin(controller)
    rclpy.shutdown()











