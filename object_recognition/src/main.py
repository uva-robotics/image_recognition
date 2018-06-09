#!/usr/bin/python

# OBJECT RECOGNITION
#   by tHE iNCREDIBLE mACHINE
#
# Object recognition for Pepper (uses ROS)

import rospy
import argparse
import numpy as np
import cv2
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class Recogniser ():
    # INit
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

    def classify (self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Test", gray)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        cv2.waitKey(1)
        return img

class VideoStreamer ():
    # INit
    def __init__(self, running_time = -1, filetype = "mp4"):
        self.subscriber = rospy.Subscriber("/pepper_robot/camera/front/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.running = False
        self.idle = True
        self.running_time = running_time
        self.extension = filetype
        self.recogniser = Recogniser()
        print("VideoStreamer > Created")

    # Start
    def start (self):
        print("VideoStreamer > Starting...")
        if not self.running:
            self.buffer = []
            self.running = True
            self.first_time = True
            self.start_time = time.time()
            self.out = "None"

            # Setup video reader
            self.fourcc = cv2.VideoWriter_fourcc(*'X264')

            print("VideoStreamer > Started successfully")

            # Now start looping until:
            #   1) Rospy core is shutting down
            #   2) The running_time hasn't passed
            #   3) KeyboardInterrupt
            try:
                while not rospy.core.is_shutdown() and (self.running_time == -1 or time.time() - self.start_time <= self.running_time):
                    rospy.rostime.wallsleep(0.5)
            except KeyboardInterrupt:
                rospy.core.signal_shutdown('keyboard interrupt')
                self.stop()
            except SystemExit:
                self.stop()
            self.stop()
        else:
            print("VideoStreamer > Already started")

    # Handle the callbacks
    def callback (self, data):
        if self.running:
            self.idle = False

            if self.first_time:
                self.first_time = False
                # Finish initing video codec
                self.out = cv2.VideoWriter("/home/tim/Desktop/output.mp4", self.fourcc, 24.0, (data.width, data.height))
                print("VideoStreamer > Successfully inited videowriter")

            # Write away
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # Analyse the frame
            self.out.write(cv_image)
            recogniser_img = self.recogniser.classify(cv_image)
            cv2.imshow("Img", recogniser_img)
            cv2.waitKey(1)
            #self.recogniser.classify(cv_image)

            self.idle = True

    # Stop it
    def stop (self):
        print("VideoStreamer > Closing...")
        if self.running:
            self.running = False
            # Close video stream
            if self.out != "None":
                self.out.release()
            cv2.destroyAllWindows()

            print("VideoStreamer > Closed successfully")
        else:
            print("VideoStreamer > Already closed")

# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--runtime", type=int, help="The time (in seconds) the script will run")
    parser.add_argument("-f", "--filetype", help="The extension of the stream file (without .)")
    args = parser.parse_args()

    runtime = -1
    filetype = "mp4"
    if args.runtime:
        runtime = args.runtime
    if args.filetype:
        filetype = args.filetype

    # Init ROS
    rospy.init_node('object_recognition')

    # Start the streamer
    streamer = VideoStreamer(runtime, filetype)
    streamer.start()
