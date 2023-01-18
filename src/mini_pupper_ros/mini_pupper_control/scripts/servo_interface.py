#!/usr/bin/python3

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory
from MangDang.mini_pupper.HardwareInterface import HardwareInterface


# for zenoh
import zenoh
import json
# to serialize ros1 messages and send them via zenoh
import io
import time
import argparse


hardware_interface = HardwareInterface()


def callback(msg):
    joint_positions = msg.points[0].positions
    lf1_position = joint_positions[0]
    lf2_position = joint_positions[1]
    lf3_position = joint_positions[2]
    rf1_position = joint_positions[3]
    rf2_position = joint_positions[4]
    rf3_position = joint_positions[5]
    lb1_position = joint_positions[6]
    lb2_position = joint_positions[7]
    lb3_position = joint_positions[8]
    rb1_position = joint_positions[9]
    rb2_position = joint_positions[10]
    rb3_position = joint_positions[11]

    joint_angles = np.array([
        [rf1_position, lf1_position, rb1_position, lb1_position],
        [rf2_position, lf2_position, rb2_position, lb2_position],
        [rf2_position+rf3_position, lf2_position+lf3_position,
         rb2_position+rb3_position, lb2_position+lb3_position]
    ])

    # print(f"I'm going to set actuator positions to {joint_angles}")
    hardware_interface.set_actuator_postions(joint_angles)


def z_listener(sample):
    try:
        msg = JointTrajectory()
        msg.deserialize(sample.payload)
        callback(msg)
    except:
        pass


def listener():


    # parser = argparse.ArgumentParser()
    # parser.add_argument('-m','--mode', help='Zenoh mode', required=False, type=str, default='client')
    # parser.add_argument('-c','--connect', help='Zenoh connect locator', required=False, type=str, default='tcp/192.168.86.131:7447')

    # args = vars(parser.parse_args())

    z_mode = 'client'
    z_connect = 'tcp/192.168.86.131:7447'

    z_config = zenoh.Config()
    # z_config.insert_json5(zenoh.config.MODE_KEY, json.dumps(args['mode']))
    # z_config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([args['connect']]))
    z_config.insert_json5(zenoh.config.MODE_KEY, json.dumps(z_mode))
    z_config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([z_connect]))
    z_session = zenoh.open(z_config)
    # init zenoh



    # rospy.init_node('servo_interface', anonymous=True)
    # rospy.Subscriber("/joint_group_position_controller/command",
    #                  JointTrajectory, callback, queue_size=1)

    sub = z_session.declare_subscriber("joint_group_position_controller/command", z_listener)


    # rospy.spin()
    while True:
        time.sleep(10)


if __name__ == '__main__':
    listener()
