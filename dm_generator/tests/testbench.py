#!/usr/bin/python3
# -*- coding: utf-8 -*-
import json
import rospy
from argparse import ArgumentParser
from std_msgs.msg import String

from helpers import get_testbench_list, get_msgs_list


def main():
    # Get testbench arguments
    parser = ArgumentParser()

    parser.add_argument('--testbench',
                        type=str,
                        required=True,
                        choices=get_testbench_list())

    parser.add_argument('--starts',
                        type=int,
                        default=3,
                        help='Testbench start delay')

    parser.add_argument('--delay',
                        type=int,
                        default=5,
                        help='Delay per testbench message')

    args = parser.parse_args()

    # Initialize ROS
    rospy.init_node('testbench_node',
                    anonymous=True,
                    disable_signals=True)

    pubisher = rospy.Publisher('/taskExecution', String, queue_size=10)

    # Get testbench messages
    msgs_list = get_msgs_list(args.testbench)

    # Execute testbench
    rospy.sleep(args.starts)

    for msg_path in msgs_list:
        with open(msg_path) as msg_file:
            msg = json.load(msg_file)

        pubisher.publish(json.dumps(msg))
        rospy.loginfo(msg)
        rospy.sleep(args.delay)


if __name__ == '__main__':
    main()
