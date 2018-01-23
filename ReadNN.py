#!/usr/bin/env python2.7
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
# File name: ReadNN.py
# Author: Hany Elshahawy
# E-mail: helshahawy@sigratech.de
# ***************************************************************************
# ***************************************************************************
# */

from sys import *
import rospy
from rospy_tutorials.msg import Floats

def callback(data):
    rospy.loginfo('NN commands: %s', data.data)

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for this node
    rospy.init_node('NnSub', anonymous=True)

    rospy.Subscriber('NNcommands', Floats, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()