#! /usr/bin/env python2
# -*- coding: utf-8 -*-
import cv2
import sys

if __name__ == '__main__':
    resource = 6
    cap = cv2.VideoCapture(4)
    if not cap.isOpened():
        print "Error opening resource: " + str(resource)
        print "Maybe opencv VideoCapture can't open it"
        exit(0)

    print "Correctly opened resource, starting to show feed."
    rval, frame = cap.read()
    while rval:
        cv2.imshow("Stream: " , frame)
        rval, frame = cap.read()
        key = cv2.waitKey(20)
        if key == 27 or key == 1048603:
            break
    cv2.destroyWindow("preview")

