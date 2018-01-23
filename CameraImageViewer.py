#!/usr/bin/env python2
# -*- coding: utf-8 -*-

# /*
# ***************************************************************************
# ***************************************************************************
# (C) 2016 SIGRA Technologies GmbH  All rights reserved.
#
# All data and information contained in or disclosed by this document is
# confidential and proprietary information of SIGRA Technologies GmbH and all
# rights therein are expressly reserved.  By accepting this material the
# recipient agrees that this material and the information contained therein
# is held in confidence and in trust and will not be used, copied, reproduced
# in whole or in part, nor its contents revealed in any manner to others
# without the express written permission of SIGRA Technologies GmbH
#
# SIGRA Technologies GmbH
# Agnes-Pockels-Bogen 1,
# 80992, Munich,
# Germany
#
# File name: main.py
# Author: Hany Elshahawy
# E-mail: helshahawy@sigratech.de
# ***************************************************************************
# ***************************************************************************
# */

## file_description
# starts node "CameraViewer" that reads camera images encoded in message type "sensor_msgs.msg.Image"
# from topic named "/usb_cam/image_raw" and view them
## end


# import libraries
import rospy
import numpy as np,cv2
from std_msgs.msg import UInt8MultiArray
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError


# callback function executed when data is read from topic
def callback(data):
    # CvBridge is a ROS library that provides interface between ROS and
    # openCV
    bridge = CvBridge()
    #If the default value of "passthrough" is given, the destination image encoding will be the same as the image message encoding.
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    cv2.imshow("img received",cv_image )
    cv2.waitKey(1)
    # print(rospy.get_name(), "I heard %s"%str(data.data))

def listener():
    rospy.init_node('CameraViewer')
    # subscribe top topic called "/usb_cam/image_raw" to read msg of type Image
    rospy.Subscriber("camera", Image, callback)
    #rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
