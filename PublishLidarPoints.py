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
# File name: PublishLidarPoints.py
# Author: Hany Elshahawy
# E-mail: helshahawy@sigratech.de
# ***************************************************************************
# ***************************************************************************
# */

## file_description
#
## end
from sys import *
import numpy as np,configurations as cfg,socket,cv2
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Header
from geometry_msgs.msg import Point32


# local functions #
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
def ParseDataPacket(LidarDataPacket):
    ## function_description
    # takes LidarDataPacket either received from unity or read from file and return back Lidar values (x,y,z,intensity) in vehicle front view
    ## end
    LidarDataPacket = np.frombuffer(LidarDataPacket, dtype=np.uint8) # convert to np.array

    # check for finger print in packet, if not found return false - data packets are corrupted
    FingPrint1 = LidarDataPacket[0] * (2 ** 24)
    FingPrint2 = LidarDataPacket[1] * (2 ** 16)
    FingPrint3 = LidarDataPacket[2] * (2 ** 8)
    FingPrint4 = LidarDataPacket[3]
    sum = FingPrint1 | FingPrint2
    sum = sum | FingPrint3
    sum = sum | FingPrint4
    if sum != cfg.FingerPrintVal:
        print('Finger print not found')
        return

    # reshape lidar data
    LidarDataPacket = LidarDataPacket.reshape((cfg.g_LiPacketPerBuffer, cfg.LiPacketSize))
    LidarDataPacket = LidarDataPacket.transpose()

    # parse to points intensities
    Intensity = LidarDataPacket[cfg.iInt, :]
    Intensity = Intensity.transpose()
    Intensity = Intensity.reshape(Intensity.size)

    # parse to get points positions
    Pos1hb = np.multiply(LidarDataPacket[cfg.iPos, :], 2 ** 8)
    Pos1lb = np.array(LidarDataPacket[cfg.iPos2, :])
    PosTemp = np.bitwise_or(Pos1hb, Pos1lb)
    ScalingFactor = np.matlib.repmat((float(360) / float(10400)), PosTemp.shape[0], PosTemp.shape[1])
    aPos = PosTemp * ScalingFactor
    aPos = aPos.transpose()
    Position = aPos.reshape(aPos.size)
    Pos = np.repeat(Position, cfg.PointsPerFire) # position of lidar points

    # parse to get distance returned for each lidar point
    Disb1 = np.multiply(LidarDataPacket[cfg.iDisR + (cfg.iR * 4), :], 2 ** 24)
    Disb1 = Disb1.astype(int)
    Disb2 = np.multiply(LidarDataPacket[cfg.iDisR + (cfg.iR * 4) + 1, :], 2 ** 16)
    Disb2 = Disb2.astype(int)
    Disb3 = np.multiply(LidarDataPacket[cfg.iDisR + (cfg.iR * 4) + 2, :], 2 ** 8)
    Disb3 = Disb3.astype(int)
    Disb4 = LidarDataPacket[cfg.iDisR + (cfg.iR * 4) + 3, :]
    Disb4 = Disb4.astype(int)
    D1_temp = np.bitwise_or(Disb1, Disb2)
    D2_temp = np.bitwise_or(Disb3, Disb4)
    DisTemp = np.bitwise_or(D1_temp, D2_temp)
    DisTemp = DisTemp.transpose()
    Distance = DisTemp.reshape(DisTemp.size)
    Distance = Distance * 10e-6 # distance returned by each lidar point

    # # remove back points
    # BackPointsIndices = np.where(np.logical_and(Pos >= cfg.BackViewRange[0], Pos <= cfg.BackViewRange[1]))
    # Distance[BackPointsIndices[0]] = 0
    #
    # # remove out of range points
    # OutOfRangeIndices = np.where(Distance > cfg.LIDAR_RANGE)
    # Distance[OutOfRangeIndices] = 0

    # calculate x,y,z
    ThetaOfPointsUsed = Pos * (np.pi / 180)
    CosPhiOfPoints = np.cos(cfg.PhiOfPoints)
    SinPhiOfPoints = np.sin(cfg.PhiOfPoints)
    CosThetaOfPoints = np.cos(ThetaOfPointsUsed)
    SinThetaOfPoints = np.sin(ThetaOfPointsUsed)
    XY_Temp = Distance * CosPhiOfPoints
    x = XY_Temp * CosThetaOfPoints
    y = XY_Temp * SinThetaOfPoints
    z = Distance * SinPhiOfPoints

    # compensate for lidar tilting angle
    t1 = cfg.R11 * x[:] + cfg.R21 * y[:] + cfg.R31 * z[:]
    t2 = cfg.R12 * x[:] + cfg.R22 * y[:] + cfg.R32 * z[:]
    z = cfg.R13 * x[:] + cfg.R23 * y[:] + cfg.R33 * z[:]
    x = t1
    if cfg.DATA_SOURCE == cfg.e_DataSource.Unity: # invert if data is received from unity
        y = -1 * t2
    elif cfg.DATA_SOURCE == cfg.e_DataSource.File:
        y = t2
    return x, y, z, Intensity
def ShowImagesFromLidar(X, Y, Z):
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
def InitializeSocketConnectionWithLidar():
    ## global_function_description
    # initialize socket connection with unity and return socket handler
    ## end

    # start socket connection
    SocketRec = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print("waiting for tcp connection")
    # connect with Lidar
    SocketRec.connect(('10.60.30.110', cfg.LidarPort))
    SocketRec.settimeout(None)
    print("Tcp connection successful")
    return SocketRec
def CreatePointCloudMessage(X, Y, Z, Intensity):
    # initialize point cloud message
    PointCloudTemp = PointCloud()
    MessageHeader = Header()
    MessageHeader.stamp = rospy.Time.now()
    # PointCloudTemp.points = len(X)
    # PointCloudTemp.ChannelFloat32 = len(X)
    for Counter in range(0,len(X)):
        PointTemp = Point32()
        PointTemp.x = X[Counter]
        PointTemp.y = Y[Counter]
        PointTemp.z = Z[Counter]
        PointCloudTemp.points.append(PointTemp)
    PointCloudTemp.header = MessageHeader
    return PointCloudTemp


# end #

if __name__ == "__main__":
    try:
        # counter for frame read every cycle
        FrameCounter = 1
        if cfg.DATA_SOURCE == cfg.e_DataSource.Lidar: # read from lidar
            # socket init
            SocketRec = InitializeSocketConnectionWithLidar()
            # initialize node
            rospy.init_node('LiPub', anonymous=True)
            # publish to topic
            LiPub = rospy.Publisher('LiPoints', PointCloud, queue_size=10)
            # rate of publication
            rate = rospy.Rate(10)  # 10hz
            # keep reading DataPackets
            while not rospy.is_shutdown():
                # read lidar data
                LidarData = SocketRec.recv(cfg.LiPacketSize * cfg.g_LiPacketPerBuffer,
                                              socket.MSG_WAITALL)
                # get lidar points
                [X, Y, Z, Intensity] = ParseDataPacket(LidarData)
                # publish to topic
                PointCloudTemp = CreatePointCloudMessage(X, Y, Z, Intensity)
                LiPub.publish(PointCloudTemp)
                # print frame number
                stdout.write("\rPublishing frame: %i" % FrameCounter)
                stdout.flush()
                FrameCounter += 1
                rate.sleep()
            else:
                SocketRec.shutdown()
                SocketRec.close()
    except rospy.ROSInterruptException:
        print("Interrupted")



