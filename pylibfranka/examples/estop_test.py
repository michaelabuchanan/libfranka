#!/usr/bin/env python3

import argparse
import time

from pylibfranka import Robot

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, required=True, help="Robot IP address")
    args = parser.parse_args()

    # Connect to robot
    robot = Robot(args.ip)
    print("Connected to robot")

    try:
        print("Sleeping...")
        sleep(10)

    except Exception as e:
        print(f"Error occurred: {e}")
        return -1


if __name__ == "__main__":
    main()