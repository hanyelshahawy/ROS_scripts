#!/usr/bin/env python
# PKG = 'numpy_tutorial'
# import roslib; roslib.load_manifest(PKG)

import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import UInt8MultiArray
from rospy_tutorials.msg import Floats

import numpy
def talker():
    a = UInt8MultiArray()
    a.data = [1,43,6,3]
        #numpy.array([1, 1, 3, 4, 5, 6], dtype=numpy.uint8)
    pub = rospy.Publisher('floats', UInt8MultiArray,queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(a)
        r.sleep()

if __name__ == '__main__':
    talker()
