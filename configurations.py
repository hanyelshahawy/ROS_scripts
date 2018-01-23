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
# File name: cfg.py
# Author: Hany Elshahawy
# E-mail: helshahawy@sigratech.de
# ***************************************************************************
# ***************************************************************************
# */

## file_description
# 1- cfgs vars: choose data source (unity or file), max lidar range and binary file name (should be in same directory)
# 2- lidar specific vars: tilting angle of lidar
# 3- viewer vars: view image size, interpolation range for points in xyz, minimum value for pixel
# 4- protocol vars: communication with unity vars
# 5- lidar cfgs: data sheet-related vars
## end

import numpy as np,numpy.matlib
from enum import Enum
class e_DataSource(Enum):
    Unity = 0
    File = 1
    Lidar = 2

# cfgs vars
DATA_SOURCE = e_DataSource.Lidar
LIDAR_RANGE = 20
FILE_NAME = 'iF612center'

# lidar specific vars
if DATA_SOURCE == e_DataSource.File: # car lidar cfg
    g_LiPacketPerBuffer = 108
    BackViewRange = [180, 360]
    LidarAlpha = 0  * (np.pi / 180)# rotation angle around z-axis
    LidarGamma = 0  * (np.pi / 180)# rotation angle around y-axis
    LidarBeta = 0 * (np.pi / 180) # rotation angle around x-axis
elif DATA_SOURCE == e_DataSource.Unity: # unity lidar cfg
    g_LiPacketPerBuffer = 208
    BackViewRange = [0, 180]
    LidarAlpha = 0  * (np.pi / 180)# rotation angle around z-axis
    LidarGamma = 0 * (np.pi / 180) # rotation angle around y-axis
    LidarBeta = 0 * (np.pi / 180)# rotation angle around x-axis
elif DATA_SOURCE == e_DataSource.Lidar: # unity lidar cfg
    g_LiPacketPerBuffer = 108
    BackViewRange = [180, 360]
    LidarAlpha = 0  * (np.pi / 180)# rotation angle around z-axis
    LidarGamma = 0 * (np.pi / 180) # rotation angle around y-axis
    LidarBeta = 0 * (np.pi / 180)# rotation angle around x-axis


# viewer vars
ImageSize = 500
LidarXrange = np.array([-LIDAR_RANGE,LIDAR_RANGE])
LidarYrange = np.array([0,LIDAR_RANGE * 2])
LidarZrange = np.array([-3,3])
ImageMinPixelValue = 20

# protocol vars
SizeRequestCode = np.zeros(8).astype(np.uint8)
SizeRequestCode[0] = 1
SizeRequestCode = SizeRequestCode.tobytes()
DataRequestCode = np.zeros(8).astype(np.uint8)
DataRequestCode[0] = 2
DataRequestCode = DataRequestCode.tobytes()
DataSizedToBeReceived = 4
LidarPort = 4141

# lidar cfgs
PointsPerFire = 8
LiPacketSize = 6632
LiPointsPerPacket = 50
NumberOfPointsInCycle = 10400
FiresPerBuffer = LiPointsPerPacket * g_LiPacketPerBuffer
FingerPrint = '75bd7e97'
FingerPrintVal = 1975352983 # finger print value
AnglesRad = np.array([-0.318505, -0.2692, -0.218009, -0.165195, -0.111003, -0.0557982, 0, 0.0557982])
AnglesRad = AnglesRad.reshape(AnglesRad.size)
PhiOfPoints = np.rad2deg(AnglesRad)
Phis = np.matlib.repmat(PhiOfPoints, FiresPerBuffer, 1)
PhiOfPoints = (Phis.reshape(Phis.size)) * (np.pi / 180)
i = range(8)
iInt = range(120,(132 * 50 + 120),132)
iInt = np.repeat(iInt,8)
Incremental = np.matlib.repmat(i,1,int(len(iInt)/8))
iInt = iInt + Incremental
iPos = range(20, (132 * 50 + 20), 132)
iPos2 = range(21, (132 * 50 + 21), 132)
iDis = range(24, (132 * 50 + 24), 132)
iDisR = np.repeat(iDis, PointsPerFire)
iR = np.matlib.repmat(i, 1, LiPointsPerPacket)
iR = iR[0]
R11 = (np.cos(LidarAlpha) * np.cos(LidarGamma))
R12 = (np.cos(LidarAlpha) * np.sin(LidarGamma) * np.sin(LidarBeta)) - (np.sin(LidarAlpha) * np.cos(LidarBeta))
R13 = (np.cos(LidarAlpha) * np.sin(LidarGamma) * np.cos(LidarBeta)) + (np.sin(LidarAlpha) * np.sin(LidarBeta))
R21 = (np.sin(LidarAlpha) * np.cos(LidarGamma))
R22 = (np.cos(LidarAlpha) * np.sin(LidarGamma) * np.sin(LidarBeta)) + (np.cos(LidarAlpha) * np.cos(LidarBeta))
R23 = (np.cos(LidarAlpha) * np.sin(LidarGamma) * np.cos(LidarBeta)) - (np.cos(LidarAlpha) * np.sin(LidarBeta))
R31 = (-1 * np.sin(LidarGamma))
R32 = (np.cos(LidarGamma) * np.sin(LidarBeta))
R33 = (np.cos(LidarGamma) * np.cos(LidarBeta))
RotMat = np.array([[R11,R12,R13],[R21,R22,R23],[R31,R32,R33]])

