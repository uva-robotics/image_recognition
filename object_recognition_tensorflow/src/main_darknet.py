#!/usr/bin/python

# OBJECT RECOGNITION
#   by tHE iNCREDIBLE mACHINE
#
# Object recognition for Pepper (uses ROS)

##################### CHANGE LOG #####################
## v0.1.0 (alpha):                                  ##
##  + Begun alpha development                       ##
##  + Copied most content from object_recognition   ##
##  + Begun work on YOLO implementation             ##
######################################################
## v0.1.1:                                          ##
##  o Very small bug-fix, but essential (returned   ##
##    'img' in temporary classify function)         ##
######################################################
## v0.2.0:                                          ##
##  + Added welcoming message                       ##
##  o Changed program structure to modern (with     ##
##    main function and entry point)                ##
######################################################
## v0.2.1:                                          ##
##  o Attempt to implement YOLO using subprocess,   ##
##    result pending                                ##
######################################################
## v0.2.2:                                          ##
##  o Attempt to implement YOLO using subprocess,   ##
##    one-at-a-time method (result pending)         ##
######################################################
## v0.2.3:                                          ##
##  o Subprocess got complicated due to relative    ##
##    paths, working on solution                    ##
######################################################
## v0.2.4:                                          ##
##  o Attempted to implement YOLO using lightnet.   ##
##    Result pending                                ##
######################################################
## v0.3.0:                                          ##
##  o YOLO succes                                   ##
######################################################
## v0.4.0:                                          ##
##  o Attempted to implement YOLO using darknet.py  ##
##    This method is advantagous over lightnet due  ##
##    to the possibilities to recompile darknet as  ##
##    required                                      ##
######################################################
## v0.4.1:                                          ##
##  o Success implementing YOLO using darknet.py    ##
##    Will need to have a method to convert numpy   ##
##    list to C list                                ##
##  - Removed obsolete debug counters in both       ##
##    streamers                                     ##
######################################################
## v0.4.2:                                          ##
##  + Added custom darknet path support             ##
##  o Changed directory back after loading darknet  ##
######################################################
## v0.5.0:                                          ##
##  o Changed passing of images to darknet to       ##
##    directly rather than through temporary file   ##
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
import darknet
import os
import ctypes

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

COLORS = [
    (255, 0, 0),
    (0, 255, 0),
    (0, 0, 255),
    (255, 255, 0),
    (255, 0, 255),
    (0, 255, 255)
]

# Class for recognising objects
class Recogniser ():
    # INit
    def __init__(self, darknet_path):
        self.name = "YOLO-Recogniser"

        # Cd to the correct folder
        current_path = os.getcwd()
        os.chdir(darknet_path)

        # Load the darknet path
        self.network = darknet.load_net("cfg/yolov3-tiny.cfg", "yolov3-tiny.weights", 0)
        self.metadata = darknet.load_meta("cfg/coco.data")
        self.path = darknet_path

        # Change back to current_path
        os.chdir(current_path)

        self.log("Created")

    # Try to classify an image using YOLO
    def classify (self, img):
        self.log("Preparing image...")
        # Convert to c image
        c_img = darknet.nparray_to_image(img)

        self.log("Done, classifying...")
        result = darknet.detect(self.network, self.metadata, c_img)
        self.log("Done, drawing boxes...")
        new_image = self.draw_boxes(img, result)
        self.log("Done")
        return new_image

    # Convert the result to draw boxes
    def draw_boxes(self, img, boxes):
        i = 0
        for class_name, prob, xywh in boxes:
            x, y, w, h = xywh
            # Draw the rectangle
            cv2.rectangle(img, (int(x-(w/2)), int(y-(h/2))), (int(x+(w/2)), int(y+(h/2))), COLORS[i], 3)
            cv2.putText(img,class_name,(int(x-(w/2)),int(y-(h/2)-5)), cv2.FONT_HERSHEY_SIMPLEX, 1,COLORS[i],2,cv2.LINE_AA)
            i = (i + 1) % len(COLORS)
        return img

    # Log data on the output
    def log (self, text):
        print("{} > {}".format(self.name, text))

# Class for streaming video from pepper
class VideoStreamer (threading.Thread):
    # INit
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        rospy.Subscriber("/pepper_robot/camera/front/image_raw/compressed", CompressedImage, self.callback, queue_size=1)

        self.bridge = CvBridge()
        self.running = False
        self.idle = True
        self.name = "VideoStreamer"
        self.buffer = np.array([])

        self.log("Created")

    # Log data
    def log (self, message):
        print("{} > {}".format(self.name, message))

    # Start
    def start (self):
        self.log("Starting...")
        if not self.running:
            self.running = True

            threading.Thread.start(self)

            self.log("Started successfully")
        else:
            self.log("Already started")

    # Run
    def run (self):
        # Now start looping until:
        #   1) Rospy core is shutting down
        #   2) The running_time hasn't passed
        #   3) KeyboardInterrupt
        try:
            while self.running and not rospy.core.is_shutdown():
                rospy.rostime.wallsleep(0.5)
        except KeyboardInterrupt:
            rospy.core.signal_shutdown('keyboard interrupt')
            self.stop()
        except SystemExit:
            self.stop()
        if self.running:
            self.stop()

    # Handle the callbacks
    def callback(self, data):
        if self.running:
            self.idle = False

            # Get the cv2 image from ros data
            try:
                image_array = np.fromstring(data.data, np.uint8)
                cv_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)
            except CvBridgeError as e:
                print(e)

            # We got a frame, put the frame in the buffer
            self.buffer = cv_image
            self.idle = True

    # Returns buffer and clears it
    def get_buffer (self):
        return self.buffer

    # Stop it
    def stop (self):
        self.log("Closing...")
        if self.running:
            self.running = False
            # Wait until idle
            while not self.idle:
                pass
            self.log("Closed successfully")
        else:
            self.log("Already closed")

# Class for streaming video from pepper
class VideoStreamerWebcam (threading.Thread):
    # INit
    def __init__(self):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.running = False
        self.idle = True
        self.name = "VideoStreamer"
        self.buffer = np.array([])

        self.log("Created")

    # Log data
    def log (self, message):
        print("{} > {}".format(self.name, message))

    # Start
    def start (self):
        self.log("Starting...")
        if not self.running:
            self.running = True
            self.cap = cv2.VideoCapture(0)

            threading.Thread.start(self)

            self.log("Started successfully")
        else:
            self.log("Already started")

    # Run
    def run (self):
        while self.running and self.cap.isOpened():
            self.idle = False

            ret, frame = self.cap.read()
            if ret:
                # Got a frame
                self.buffer = frame

            self.idle = True
        if not self.cap.isOpened():
            self.stop()

    # Returns buffer and clears it
    def get_buffer (self):
        return self.buffer

    # Stop it
    def stop (self):
        self.log("Closing...")
        if self.running:
            self.running = False
            # Wait until idle
            while not self.idle:
                pass
            self.cap.release()
            self.log("Closed successfully")
        else:
            self.log("Already closed")


# Main
def main (timeout, mode, darknet_path):
    # Do welcoming message
    print("\n########################")
    print("## OBJECT RECOGNITION ##")
    print("##     USING YOLO     ##")
    print("##   v0.4.2 (alpha)   ##")
    print("########################\n")

    # Show some data
    print("USING:")
    print("  - Timeout:      {}s".format(timeout if timeout > -1 else u"\u221E".encode("utf-8")))
    print("  - Mode:         {}".format(mode))
    print("  - Darknet path: {}\n").format(darknet_path)

    if mode == "TEST":
        recogniser = Recogniser(darknet_path)
        img = cv2.imread("data/dog.jpg")
        recogniser.classify(img)
        cv2.imshow("Img", img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return

    # Init ROS
    rospy.init_node('object_recognition')

    # Start the streamer
    if mode != "WEBCAM":
        streamer = VideoStreamer()
    else:
        streamer = VideoStreamerWebcam()
    streamer.start()
    # Get the recogniser
    recogniser = Recogniser(darknet_path)

    print("\nWaiting for the first frame (30 sec timeout)...")
    frame = streamer.get_buffer()
    start = time.time()
    while len(frame) == 0 and time.time() - start <= 30:
        frame = streamer.get_buffer()
    if time.time() - start > 30:
        TimeoutError("Timeout occured while waiting for first frame")
    cv2.imshow("Img", frame)
    cv2.waitKey(1)
    print("Done, starting run" + (time.strftime(" (%H:%M:%S)") if timeout > -1 else "") + "\n")

    frames = 0
    then = time.time()
    while time.time() - then < timeout:
        # Get frame
        frame = streamer.get_buffer()
        # Classify
        classified_frame = recogniser.classify(frame)
        cv2.imshow("Img", frame)
        frames += 1
        cv2.waitKey(1)
    stop = time.time()
    print("\nTimeout, stopping...")
    streamer.stop()
    cv2.destroyAllWindows()

    print("Done (avg fps: {}).".format(frames / (stop - then)))

# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--timeout", type=int, help="The time (in seconds) the script will run")
    parser.add_argument("-m", "--mode", help="The mode in which the program will be loaded. Options are: PEPPER_CAMERA, WEBCAM and TEST")
    parser.add_argument("-d", "--darknet", help="The path to darknet (absolute)")
    args = parser.parse_args()

    timeout = -1
    mode = "PEPPER_CAMERA"
    darknet_path = "/home/tim/uva-robotics/darknet"
    if args.timeout:
        timeout = args.timeout
    if args.mode:
        mode = args.mode
    if args.darknet:
        darknet_path = args.darknet

    # Make sure darknet_path ends with "/"
    if darknet_path[-1] != "/":
        darknet_path += "/"
    # Check if the directory exists
    if not os.path.exists(darknet_path):
        print("Folder '" + darknet_path + "' does not exist, cannot continue")
        sys.exit()

    try:
        main(timeout, mode, darknet_path)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit()
