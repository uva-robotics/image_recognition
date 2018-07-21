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
##  + Added
##  o YOLO succes                                   ##
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
import lightnet

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
    def __init__(self):
        self.name = "YOLO-Recogniser"

        self.log("Loading weights...")
        self.model = lightnet.load("yolo")

        self.log("Created")

    # Try to classify an image using YOLO
    def classify (self, img):
        # For now, let's save the file to a buffer first
        with open("/home/lut_99/Desktop/to_classify.jpg") as f:
            cv2.imwrite("/home/lut_99/Desktop/to_classify.jpg", img)
        image = lightnet.Image.from_bytes(open('/home/lut_99/Desktop/to_classify.jpg', 'rb').read())
        #image = lightnet.Image(img)
        # Classify
        boxes = self.model(image)
        print(boxes)

        return self.draw_boxes(img, boxes)

    def draw_boxes (self, img, boxes):
        i = 0
        for class_id, class_name, prob, xywh in boxes:
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
        self.subscriber = rospy.Subscriber("/pepper_robot/camera/front/image_raw", Image, self.callback)
        self.bridge = CvBridge()
        self.running = False
        self.idle = True
        self.name = "VideoStreamer"
        self.buffer = (np.array([]), -1)
        self.counter = 0

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
    def callback (self, data):
        if self.running:
            self.idle = False

            # Get the cv2 image from ros data
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # We got a frame, put the frame in the buffer
            self.buffer = (cv_image, self.counter)
            self.counter += 1

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
        self.buffer = (np.array([]), -1)
        self.counter = 0

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
                self.buffer = (frame, self.counter)
                self.counter += 1

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
def main (timeout, mode):
    # Do welcoming message
    print("\n########################")
    print("## OBJECT RECOGNITION ##")
    print("##     USING YOLO     ##")
    print("##   v0.3.0 (alpha)   ##")
    print("########################\n")

    # Show some data
    print("USING:")
    print("  - Timeout: {}s".format(timeout if timeout > -1 else u"\u221E".encode("utf-8")))
    print("  - Mode:    {}\n".format(mode))

    # Init ROS
    rospy.init_node('object_recognition')

    # Start the streamer
    if mode != "WEBCAM":
        streamer = VideoStreamer()
    else:
        streamer = VideoStreamerWebcam()
    streamer.start()
    # Get the recogniser
    recogniser = Recogniser()

    print("\nWaiting for the first frame...")
    frame, _ = streamer.get_buffer()
    while len(frame) == 0:
        frame, _ = streamer.get_buffer()
    cv2.imshow("Img", frame)
    cv2.waitKey(1)
    print("Done, starting run" + (time.strftime(" (%H:%M:%S)") if timeout > -1 else "") + "\n")

    frames = 0
    then = time.time()
    while time.time() - then < timeout:
        # Get frame
        frame, counter = streamer.get_buffer()
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
    parser.add_argument("-m", "--mode", help="The mode in which the program will be loaded. Options are: PEPPER_CAMERA and WEBCAM")
    args = parser.parse_args()

    timeout = -1
    mode = "PEPPER_CAMERA"
    if args.timeout:
        timeout = args.timeout
    if args.mode:
        mode = args.mode

    try:
        main(timeout, mode)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit()
