import rospy
import zenoh
import signal
import argparse
from trajectory_msgs.msg import JointTrajectory
import json
import time



flag = True

def handler(signum, frame):
    global flag
    flag = False

def cb(sample):
    msg = JointTrajectory()
    msg.deserialize(sample.payload)
    end_time = rospy.get_rostime()
    delta = end_time - msg.header.stamp

    #middleware, cmd_s, cmd_ns, act_s, act_ns, delta_s, delta_ns
    print(f'zenoh,{msg.header.stamp.secs},{msg.header.stamp.nsecs},{end_time.secs},{end_time.nsecs},{delta.secs},{delta.nsecs}')


def main():
    rospy.impl.simtime.init_simtime()
    rospy.rostime.set_rostime_initialized(True)

    parser = argparse.ArgumentParser()
    parser.add_argument('-m','--mode', help='Zenoh mode', required=False, type=str, default='client')
    parser.add_argument('-c','--connect', help='Zenoh connect locator', required=False, type=str, default='tcp/192.168.86.131:7447')


    args, unknown = parser.parse_known_args()
    args = vars(args)

    z_mode = args['mode']
    z_connect = args['connect']

    # init zenoh

    z_config = zenoh.Config()

    z_config.insert_json5(zenoh.config.MODE_KEY, json.dumps(z_mode))
    z_config.insert_json5(zenoh.config.CONNECT_KEY, json.dumps([z_connect]))
    z_session = zenoh.open(z_config)
    # init zenoh

    sub = z_session.declare_subscriber("joint_group_position_controller/command/feedback", cb)


    signal.signal(signal.SIGINT, handler)

    global flag
    while flag:
        time.sleep(1)

    sub.undeclare()
    pub.undeclare()
    z_session.close()



if __name__=='__main__':
    main()