#!/usr/bin/env python3

# Copyright (c) 2025 Franka Robotics GmbH
# Use of this source code is governed by the Apache-2.0 license, see LICENSE

import argparse
import time

import numpy as np

from pylibfranka import ControllerMode, JointPositions, Robot
from example_common import MotionGenerator

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="localhost", help="Robot IP address")
    args = parser.parse_args()

    # Connect to robot
    robot = Robot(args.ip)

    try:
        # Set collision behavior
        lower_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        upper_torque_thresholds = [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]
        lower_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]
        upper_force_thresholds = [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]

        robot.set_collision_behavior(
            lower_torque_thresholds,
            upper_torque_thresholds,
            lower_force_thresholds,
            upper_force_thresholds,
        )

        home_pos = [0,-0.785,0,-2.356,0,1.571,0.785]

        # First move the robot to a suitable joint configuration
        print("WARNING: This example will move the robot!")
        print("Please make sure to have the user stop button at hand!")
        input("Press Enter to continue...")

        q_goal = [0.0, -0.785, 0.0, -2, 0.0, 1.571, 0.785]
        speed_factor = 0.1  
        
        home_motion_generator = MotionGenerator(speed_factor, home_pos)                    
        # dq_max = [2,2,2,2,2.5,2.5,2.5] * speed_factor
        motion_generator = MotionGenerator(speed_factor, q_goal)

        active_control = robot.start_joint_position_control(ControllerMode.JointImpedance)

        # homing
        print("Homing...\n")
        current_pos = robot.read_once()
        joints = current_pos.q
        print(joints)

        home_finished = False

        while not home_finished:
            robot_state, duration = active_control.readOnce()

            joint_positions = home_motion_generator(robot_state, duration.to_sec())
            home_finished = joint_positions.motion_finished

            active_control.writeOnce(joint_positions)

        print("Done homing\n")

        motion_finished = False
        while not motion_finished:
            print("moving robot\n")
            print(robot_state.q)
            robot_state, duration = active_control.readOnce()
            print(robot_state.q)

            joint_positions = motion_generator(robot_state, duration.to_sec())
            motion_finished = joint_positions.motion_finished

            active_control.writeOnce(joint_positions)

        print("\nAll done!\n")

    except Exception as e:
        print(f"Error occurred: {e}")
        if robot is not None:
            robot.stop()
        return -1


if __name__ == "__main__":
    main()
