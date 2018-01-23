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
# File name: ReadLidarPoints.py
# Author: Hany Elshahawy
# E-mail: helshahawy@sigratech.de
# ***************************************************************************
# ***************************************************************************
# */

from sys import *
import numpy as np,configurations as cfg,socket,cv2
import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from geometry_msgs.msg import Point32

def CreateImage(x, y, z,ImType):
    ## function_description
    # takes lidar points and return top or front image according to ImType
    ## end

    # initialize image
    TempImg = np.zeros((cfg.ImageSize, cfg.ImageSize, 3), dtype=np.uint8)
    if ImType.lower() == 'top':
        # init of points
        X_axis_mat = np.linspace(0, cfg.ImageSize - 1, len(x))
        Y_axis_mat = np.linspace(cfg.ImageSize - 1, 0, len(y))
        X_axis_li = np.linspace(cfg.LidarXrange[0], cfg.LidarXrange[1], len(x))
        Y_axis_li = np.linspace(cfg.LidarYrange[0], cfg.LidarYrange[1], len(y))
        # interpolate to convert points to pixels
        X_axis_out = np.round(np.interp(x, X_axis_li, X_axis_mat)).astype(int)
        Y_axis_out = np.round(np.interp(y, Y_axis_li, Y_axis_mat)).astype(int)
        # image values
        TopVal = 255
        Z_axis_show = np.linspace(cfg.ImageMinPixelValue, TopVal, len(z))
        Z_axis_li = np.linspace(cfg.LidarZrange[0], cfg.LidarZrange[1], len(z))
        Z_axis_out = np.round(np.interp(z, Z_axis_li, Z_axis_show, period= None)).astype(int)
        # assigning values to image pixels
        TempImg[Y_axis_out[:], X_axis_out[:],1] = Z_axis_out[:]
    elif ImType.lower() == 'front':
        # init of points
        X_axis_mat = np.linspace(0, cfg.ImageSize - 1, len(x))
        Z_axis_mat = np.linspace(cfg.ImageSize - 1, 0, len(z))
        X_axis_li = np.linspace(cfg.LidarXrange[0], cfg.LidarXrange[1], len(x))
        Z_axis_li = np.linspace(cfg.LidarZrange[0], cfg.LidarZrange[1], len(z))
        # interpolate to convert points to pixels
        X_axis_out = np.round(np.interp(x, X_axis_li, X_axis_mat)).astype(int)
        Z_axis_out = np.round(np.interp(z, Z_axis_li, Z_axis_mat)).astype(int)
        # image values
        TopVal = 255
        Y_axis_show = np.linspace(cfg.ImageMinPixelValue, TopVal, len(y))
        Y_axis_li = np.linspace(cfg.LidarYrange[0], cfg.LidarYrange[1], len(y))
        Y_axis_out = np.round(np.interp(y, Y_axis_li, Y_axis_show, period= None)).astype(int)
        # assigning values to image pixels
        TempImg[Z_axis_out[:], X_axis_out[:],1] = Y_axis_out[:]
    return TempImg
def ReturnImagesFromLidar(X, Y, Z):
    ## global_function_description
    # takes lidar points and show top and front images
    ## end

    # get top view image
    TopViewImage = CreateImage(X, Y, Z, 'top')
    # get front view image
    FrontViewImage = CreateImage(X, Y, Z, 'front')
    # show top and front view images
    cv2.imshow('Top view image', TopViewImage)
    cv2.imshow('Front view image', FrontViewImage)
    cv2.waitKey(1)

def callback(PointCloud):
    X = np.zeros(len(PointCloud.points))
    Y = np.zeros(len(PointCloud.points))
    Z = np.zeros(len(PointCloud.points))
    for i in range(0,len(PointCloud.points)):
        X[i] = PointCloud.points[i].x
        Y[i] = PointCloud.points[i].y
        Z[i] = PointCloud.points[i].z
    ReturnImagesFromLidar(X,Y,Z)
    rospy.loginfo(rospy.get_caller_id() + 'I heard %i', len(X))

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for this node
    rospy.init_node('LiSub', anonymous=True)

    rospy.Subscriber('LiPoints', PointCloud, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()