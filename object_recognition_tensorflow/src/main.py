#!/usr/bin/python

# OBJECT RECOGNITION
#   by tHE iNCREDIBLE mACHINE
#
# Object recognition for Pepper (uses ROS)

##################### CHANGE LOG #####################
## v0.1 (alpha):                                    ##
##  + Begun alpha development                       ##
##  + Copied most content from object_recognition   ##
##  + Begun work on YOLO implementation             ##
######################################################
## v0.1.1 (alpha):                                  ##
##  o Very small bug-fix, but essential (returned   ##
##    'img' in temporary classify function)         ##
######################################################
## v0.2.0 (alpha):                                  ##
##  + Added welcoming message                       ##
##  o Changed program structure to modern (with     ##
##    main function and entry point)                ##
######################################################


import rospy
import argparse
import numpy as np
import cv2
import time
import threading
import decimal
import sys
import subprocess

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Class for recognising objects (although it does faces, for now)
class Recogniser ():
    # INit
    def __init__(self, darknet_path):
        self.name = "YOLO-Recogniser"
        # Begin darknet
        self.log("Preparing darknet (path: {})...".format(darknet_path + "darknet"))
        self.process = subprocess.Popen([darknet_path + "darknet", "detect", darknet_path + "cfg/yolov3-tiny.cfg", darknet_path + "yolov3-tiny.weights"], stdout=subprocess.PIPE, stdin=subprocess.PIPE)
        self.log("Done, created")

    # Try to classify an image using YOLO
    def classify (self, img):
        # Write image to a local buffer
        with open("/home/lut_99/Desktop/to_classify.jpg") as f:
            cv2.imwrite("/home/lut_99/Desktop/to_classify.jpg", img)
        # Communicate with darknet_path to run
        result, errors = self.process.communicate("/home/lut_99/Desktop/to_classify.jpg")
        self.log("Succesfully classified:")
        print(result)
        sys.exit()

    # Log data on the output
    def log (self, text):
        print("{} > {}".format(self.name, text))

# Class for streaming video from pepper
class VideoStreamer ():
    # INit
    def __init__(self, running_time = -1, framerate = 10.0, frameskip=False, darknet_path=""):
        self.subscriber = rospy.Subscriber("/pepper_robot/camera/front/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.running = False
        self.idle = True
        self.running_time = running_time
        self.recogniser = Recogniser(darknet_path)
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
            # Stop recogniser
            self.recogniser.process.kill()
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


# Main
def main (runtime, framerate, mode, darknet_path):
    # Do welcoming message
    print("\n########################")
    print("## OBJECT RECOGNITION ##")
    print("##     USING YOLO     ##")
    print("##   v0.2.0 (alpha)   ##")
    print("########################\n")

    # Show some data
    print("USING:")
    print("  - Runtime:         {}s".format(runtime if runtime > -1 else u"\u221E".encode("utf-8")))
    print("  - Framerate:       {}fps".format(framerate))
    print("  - Mode:            {}".format(mode))
    print("  - Path to darknet: {}\n".format(darknet_path))

    # Init ROS
    rospy.init_node('object_recognition')

    # Start the streamer
    if mode == "***PEPPER-CAMERA***":
        streamer = VideoStreamer(runtime, framerate, darknet_path=darknet_path)
    else:
        streamer = VideoStreamerFile(mode)
    streamer.start()

# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--runtime", type=int, help="The time (in seconds) the script will run")
    parser.add_argument("-f", "--framerate", help="The framerate of the output file")
    parser.add_argument("-i", "--inputfile", help="The file that is to be used. Leave empty to use Pepper's camera")
    parser.add_argument("-p", "--darknet_path", help="The path to darknet (folder)")
    args = parser.parse_args()

    runtime = -1
    framerate = 10.0
    mode = "***PEPPER-CAMERA***"
    darknet_path = "/VirtualShare/darknet/"
    if args.runtime:
        runtime = args.runtime
    if args.framerate:
        framerate = args.framerate
    if args.inputfile:
        mode = args.inputfile
    if args.darknet_path:
        darknet_path = args.darknet_path

    # Make sure darknet_path is concluded with "/"
    if darknet_path[-1] != "/":
        darknet_path += "/"

    try:
        main(runtime, framerate, mode, darknet_path)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit()
