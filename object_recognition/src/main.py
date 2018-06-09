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
import threading
import decimal

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Class for recognising objects (although it does faces, for now)
class Recogniser ():
    # INit
    def __init__(self):
        self.face_cascade = cv2.CascadeClassifier('/projects/uva-robotics/src/image_recognition/object_recognition/src/haarcascade_frontalface_default.xml')
        self.eye_cascade = cv2.CascadeClassifier('/projects/uva-robotics/src/image_recognition/object_recognition/src/haarcascade_eye.xml')

    # Use both face_cascade and eye_cascade to recognise face and then eyes
    def classify (self, img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        cv2.imshow("Test", img)
        # Try to detect faces
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            # Try to detect eyes
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        return img

    # We'll need 64 bit for this, unfortunately, so let's put a hold on it for now
    def classify_vg16 (self):
        pass

# Class for streaming video from pepper
class VideoStreamer ():
    # INit
    def __init__(self, running_time = -1, framerate = 10.0, frameskip=False):
        self.subscriber = rospy.Subscriber("/pepper_robot/camera/front/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.running = False
        self.idle = True
        self.running_time = running_time
        self.recogniser = Recogniser()
        self.framerate = framerate
        self.frameskip = frameskip
        self.name = "VideoStreamer"

        self.log("Created (mode: Pepper webcam)")

    # Log data
    def log (self, message):
        print("{} > {}".format(self.name, message))

    # Start
    def start (self):
        self.log("Starting...")
        if not self.running:
            self.buffer = []
            self.running = True
            self.first_time = True
            self.start_time = time.time()
            self.out = "None"

            # Setup video reader
            self.fourcc = cv2.VideoWriter_fourcc(*'X264')

            self.log("Started successfully")

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
            self.log("Already started")

    # Handle the callbacks
    def callback (self, data):
        if self.running:
            self.idle = False

            if self.first_time:
                self.first_time = False
                # Finish initing video codec
                self.out = cv2.VideoWriter("/home/tim/Desktop/output.mp4", self.fourcc, self.framerate, (data.width, data.height))
                self.log("Successfully inited videowriters")

            timestamp = data.header.stamp.secs
            if (self.frameskip and time.time() - timestamp >= 1):
                self.log("Skipped frame to keep up")
                self.idle = True
                return

            # Get the cv2 image from ros data
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # Analyse the frame
            face_img = self.recogniser.classify(cv_image)
            # Save frame
            self.out.write(face_img)
            # Show frame
            cv2.imshow("Img", face_img)

            self.idle = True
            cv2.waitKey(1)

    # Stop it
    def stop (self):
        self.log("Closing...")
        if self.running:
            self.running = False
            # Close video stream
            if self.out != "None":
                self.out.release()
            cv2.destroyAllWindows()

            self.log("Closed successfully")
        else:
            self.log("Already closed")

# Reads video from a file and shows it frame-by-frame
class VideoStreamerFile ():
    # INit
    def __init__(self, path):
        self.path = path

        self.log("Created (mode: file)")

    # Start streaming
    def start (self):
        cap = cv2.VideoCapture(self.path)

        self.log("Opened cap (Press 'Q' to stop)")

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                cv2.imshow("Img", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        self.log("Completed successfully")

# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--runtime", type=int, help="The time (in seconds) the script will run")
    parser.add_argument("-f", "--framerate", help="The framerate of the output file")
    parser.add_argument("-i", "--inputfile", help="The file that is to be used. Leave empty to use Pepper's camera")
    args = parser.parse_args()

    runtime = -1
    framerate = 10.0
    mode = "***PEPPER-CAMERA***"
    if args.runtime:
        runtime = args.runtime
    if args.framerate:
        framerate = args.framerate
    if args.inputfile:
        mode = args.inputfile

    # Init ROS
    rospy.init_node('object_recognition')

    # Start the streamer
    if mode == "***PEPPER-CAMERA***":
        streamer = VideoStreamer(runtime, framerate)
    else:
        streamer = VideoStreamerFile(mode)
    streamer.start()
