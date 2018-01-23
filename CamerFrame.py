#! /usr/bin/env python2
# -*- coding: utf-8 -*-

import cv2,threading

class WebcamVideoStream:
    def __init__(self, src=1, height=480, width=640, fps=30):
        # initialize the video camera stream and read the first frame
        # from the stream
        self.stream = cv2.VideoCapture(src)
        # format = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')
        # this pixel format uses less USB bandwith (on expense of size), since all cameras are on same hub thus same bus
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
        return self.retval,self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True
        self.stream.release()

def DetectCameraSource():
    sourceID = None
    for i in range(0, 20):
        CamHandler = cv2.VideoCapture(i)
        if CamHandler.isOpened():
            sourceID = i
            return sourceID
    return sourceID


# if __name__ == '__main__':
sourceID = DetectCameraSource()
print("Camera index is:" + str(sourceID))
CameraHandler = WebcamVideoStream(sourceID, 480, 640, 30).start()

# rval,Frame = CameraHandler.read()
rval,Frame = CameraHandler.read()

while rval:
    cv2.imshow('Stream:' + str(sourceID),Frame)
    rval, Frame = CameraHandler.read()
    k = cv2.waitKey(20)
    # cv2.imshow('r', Frame)
    # rval, Frame = CameraHandler.read()
    if k == 27:
        break
cv2.destroyAllWindows()
