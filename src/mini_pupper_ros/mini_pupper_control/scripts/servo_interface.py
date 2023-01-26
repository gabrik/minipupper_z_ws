#!/usr/bin/python3

import numpy as np
import rospy
from trajectory_msgs.msg import JointTrajectory
from MangDang.mini_pupper.HardwareInterface import HardwareInterface


# for zenoh
import zenoh
import json
import time

from datetime import datetime
from queue import Queue
from threading import Thread
import argparse
import signal
import time
import io


hardware_interface = HardwareInterface()


msg_queue = Queue(maxsize=1)
flag = True


def handler(signum, frame):
    global flag
    flag = False


def callback(msg, pub, payload):

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

    hardware_interface.set_actuator_postions(joint_angles)

    # publish as feedback
    pub.put(payload)

    # end_time = rospy.get_rostime()

    # print(f'Actuation took {end_time.secs - msg.header.stamp.secs}s and {end_time.nsecs - msg.header.stamp.nsecs}ns')
    # middleware, cmd_s, cmd_ns, act_s, act_ns, delta_s, delta_ns
    # print(f'zenoh,{msg.header.stamp.secs},{msg.header.stamp.nsecs},{end_time.secs},{end_time.nsecs},{end_time.secs - msg.header.stamp.secs},{end_time.nsecs - msg.header.stamp.nsecs}')


def servos_thr(sub, pub):
    print("Servo thread is started!")
    msg = JointTrajectory()
    global flag
    while flag:
        try:
            for sample in sub.receiver:
                msg.deserialize(sample.payload)
                callback(msg, pub, sample.payload)
        except Exception as e:
            print(f'{e}')
            pass
    print('Received Ctrl-C bye!')


def listener():

    rospy.impl.simtime.init_simtime()
    rospy.rostime.set_rostime_initialized(True)

    parser = argparse.ArgumentParser()
    parser.add_argument('-m','--mode', help='Zenoh mode', required=False, type=str, default='client')
    parser.add_argument('-c','--connect', help='Zenoh connect locator', required=False, type=str)

    args, unknown = parser.parse_known_args()
    args = vars(args)

    z_mode = args.get('mode')
    z_connect = args.get('connect', None)

    # init zenoh

    z_config = zenoh.Config()

    z_config.insert_json5(zenoh.config.MODE_KEY, json.dumps(z_mode))
    if z_connect is not None:
        z_config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([z_connect]))
    z_session = zenoh.open(z_config)
    # init zenoh

    sub = z_session.declare_subscriber("joint_group_position_controller/command", zenoh.Queue())
    pub = z_session.declare_publisher("joint_group_position_controller/command/feedback")


    servos = Thread(target=servos_thr, args=(sub,pub,))
    servos.start()

    signal.signal(signal.SIGINT, handler)

    global flag
    while flag:
        time.sleep(1)

    sub.undeclare()
    z_session.close()

    # servos.join()

    sub.undeclare()
    z_session.close()


if __name__ == '__main__':
    listener()
