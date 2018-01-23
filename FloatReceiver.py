#!/usr/bin/env python
# PKG = 'numpy_tutorial'
# import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import UInt8MultiArray
from rospy_tutorials.msg import Floats


def callback(data):
    print(rospy.get_name(), "I heard %s"%str(data.data))

def listener():
    rospy.init_node('listener')
    rospy.Subscriber("floats", numpy_msg(UInt8MultiArray), callback)
    rospy.spin()

if __name__ == '__main__':
    listener()