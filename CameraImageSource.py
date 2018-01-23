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
#
## end


import rospy
import numpy as np,threading,cv2
from std_msgs.msg import UInt8MultiArray
from rospy.numpy_msg import numpy_msg

class WebcamVideoStream:
    def __init__(self, src=1, height=480, width=640, fps=30):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        # format = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        # # this pixel format uses less USB bandwith (on expense of size), since all cameras are on same hub thus same bus
        # self.stream.set(6, format)
        self.stream.set(4, height)
        self.stream.set(3, width)
        self.stream.set(5, fps)
        (self.retval, self.frame) = self.stream.read()

        # initialize the variable used to indicate if the thread should
        # be stopped
        self.stopped = False

    def start(self):
        # start the thread to read frames from the video stream
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        while True:
            # if the thread indicator variable is set, stop the thread
            if self.stopped:
                return
            try:
                (self.retval, self.frame) = self.stream.read()
            except:
                pass

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        self.stream.release()

def StartVideoCapture(src):
    CameraHandler = WebcamVideoStream(src, 480, 640, 30)
    return CameraHandler

def ReadFrame(CameraHandler):
    frame = CameraHandler.ReadSingleFrame()
    return frame

def CameraDetect_SeparateLabelingPlusSingleCameraDetection(aruco_dict,aruco_parameters):
    list = []
    vsList = []
    frameList = []
    camera_dict = {}
    camera_dict['left'] = None
    camera_dict['right'] = None
    camera_dict['center'] = None
    height = 480
    width = 640
    fps = 30
    for i in range(0, 20):
        try:
            vs1 = WebcamVideoStream(src=i, height=height, width=width, fps=fps).start()
            frame1 = vs1.read()
            key = cv2.waitKey(10)
            cv2.imshow('ImageTest', frame1)
            # key = cv2.waitKey(10)
            list.append(i)
            vs1.stop()
        except:
            pass
    cv2.destroyAllWindows()
    for k in range(0, len(list)):
        temp = WebcamVideoStream(src=list[k], height=height, width=width, fps=fps).start()
        vsList.append(temp)

    if len(list) == 1:
        camera_dict['center'] = list[0]
        vsList[0].stop()
    elif len(list) == 3:
        AllCamerasAreLabeled = 0 # =3 when all cameras are labeled
        CameraIdCounter = 0
        ids_list = []
        while (AllCamerasAreLabeled != 3):
            # for CameraIdCounter in range(0, len(vsList)):
            LabelNotDetected = True
            while LabelNotDetected == True:
                CameraFrame = vsList[CameraIdCounter].read()
                key = cv2.waitKey(1)
                cv2.imshow('Image' + str(list[CameraIdCounter]), CameraFrame)
                corners, ids, rejectedImgPoints = aruco.detectMarkers(CameraFrame, aruco_dict)
                if ids != None:
                    if len(ids) > 1:
                        ids = None
                ids_list.append(ids)
                markerDraw = aruco.drawDetectedMarkers(CameraFrame, corners)
                if (ids == 10) & (camera_dict['left'] == None):
                    camera_dict['left'] = list[CameraIdCounter]
                    AllCamerasAreLabeled = AllCamerasAreLabeled + 1
                    LabelNotDetected = False
                    vsList[CameraIdCounter].stop()
                    cv2.destroyWindow('Image' + str(list[CameraIdCounter]))
                elif (ids == 20) & (camera_dict['center'] == None):
                    camera_dict['center'] = list[CameraIdCounter]
                    AllCamerasAreLabeled = AllCamerasAreLabeled + 1
                    LabelNotDetected = False
                    vsList[CameraIdCounter].stop()
                    cv2.destroyWindow('Image' + str(list[CameraIdCounter]))
                elif (ids == 30) & (camera_dict['right'] == None):
                    camera_dict['right'] = list[CameraIdCounter]
                    AllCamerasAreLabeled = AllCamerasAreLabeled + 1
                    LabelNotDetected = False
                    vsList[CameraIdCounter].stop()
                    cv2.destroyWindow('Image' + str(list[CameraIdCounter]))
                elif ((camera_dict['left'] == None) & (camera_dict['center'] == None) & (camera_dict['right'] == None)):
                    print("Show label to camera")
            CameraIdCounter = CameraIdCounter + 1

    cv2.destroyAllWindows()
    for k in range(0, len(vsList)):
        vsList[k].stop()

    return camera_dict

def CameraImageSender(Camera_hndlr):
    # declares that this node is publishing to topic called 'CameraImages'
    # using message type UInt8MultiArray. 
    pub = rospy.Publisher('CameraImages', UInt8MultiArray, queue_size=10)
    # starts node of name 'talker', anonymous=True ensures unique names of node
    rospy.init_node('CameraSource', anonymous=True)
    # rate of publication = 10 hz
    rate = rospy.Rate(50)
    # check if no ctrl-c is pressed
    while not rospy.is_shutdown():
        # read camera frame
        CamFrame = ReadFrame(Camera_hndlr)
        CameraFrameArray = UInt8MultiArray()
        CameraFrameArray.data = CamFrame
        # publish string
        pub.publish(CameraFrameArray)
        # combined with rospy.rate() - ensures rate of publication
        rate.sleep()



if __name__ == '__main__': # entry point
    Camera_hndlr = StartVideoCapture(7)
    try:
        CameraImageSender(Camera_hndlr)
    except rospy.ROSInterruptException: # handles in case of lost connection with master node
        pass
