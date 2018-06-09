#!/usr/bin/python

# OBJECT RECOGNITION
#   by tHE iNCREDIBLE mACHINE
#
# Object recognition for Pepper (uses ROS). This script attempts to read a file

import rospy
import argparse
import numpy as np
import cv2
import time

# Entry
if __name__ == "__main__":
    cap = cv2.VideoCapture('/home/tim/Desktop/output.mp4')

    while(cap.isOpened()):
        ret, frame = cap.read()

        if ret:
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
