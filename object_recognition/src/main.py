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

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# Checks for timeouts
class TimeoutTracker (threading.Thread):
    def __init__ (self, timeout=3):
        threading.Thread.__init__(self)
        self.setDaemon(True)
        self.running = False
        self.timeout = timeout
        name = "TimeoutTracker"
        id = 0
        # Add the number
        if name + str(id) in threading.enumerate():
            id += 1
        # Register
        self.name = name + str(id)

        self.log("Created")

    # Print a message with the name as init
    def log (self, message):
        print("{} > {}".format(self.name, message))

    def poke (self):
        self.last_poke = time.time()

    def start (self):
        self.log("Starting...")
        if not self.running:
            self.last_poke = time.time()
            self.running = True
            threading.Thread.start(self)
            self.log("Successfully started")
        else:
            self.log("Already started")

    def run (self):
        while self.running:
            if time.time() - self.last_poke > self.timeout:
                print("WARNING: No message received from Pepper in three seconds")

    def stop (self):
        self.log("Stopping...")
        if self.running:
            self.running = False

            # Wait until done
            while self.isAlive():
                pass

            self.log("Successfully stopped")
        else:
            self.log("Already stopped")

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
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        for (x,y,w,h) in faces:
            img = cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
            roi_gray = gray[y:y+h, x:x+w]
            roi_color = img[y:y+h, x:x+w]
            eyes = self.eye_cascade.detectMultiScale(roi_gray)
            for (ex,ey,ew,eh) in eyes:
                cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        return img

# Class for streaming video from pepper
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
        self.timeout_tracker = TimeoutTracker()
        print("VideoStreamer > Created (mode: Pepper webcam)")

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

            # Start timeout timer
            self.timeout_tracker.start()

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

            # Set time the message was received
            self.timeout_tracker.poke()

            if self.first_time:
                self.first_time = False
                # Finish initing video codec
                self.out_raw = cv2.VideoWriter("/home/tim/Desktop/output.mp4", self.fourcc, 20.0, (data.width, data.height))
                self.out_recognise = cv2.VideoWriter("/home/tim/Desktop/output_faces.mp4", self.fourcc, 20.0, (data.width, data.height))
                print("VideoStreamer > Successfully inited videowriters")

            # Write away
            try:
                cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            except CvBridgeError as e:
                print(e)

            # Analyse the frame
            self.out_raw.write(cv_image)
            face_img = self.recogniser.classify(cv_image)
            self.out_recognise.write(face_img)
            cv2.imshow("Img", face_img)

            self.idle = True

            cv2.waitKey(1)

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

# Reads video from a file and shows it frame-by-frame
class VideoStreamerFile ():
    # INit
    def __init__(self, path):
        self.path = path

        print("VideoStreamer > Created (mode: file)")

    # Start streaming
    def start (self):
        cap = cv2.VideoCapture(self.path)

        print("VideoStreamer > Opened cap (Press 'Q' to stop)")

        while cap.isOpened():
            ret, frame = cap.read()
            if ret:
                cv2.imshow("Img", frame)
            if cv2.waitKey(25) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

        print("VideoStreamer > Completed successfully")

# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--runtime", type=int, help="The time (in seconds) the script will run")
    parser.add_argument("-f", "--filetype", help="The extension of the stream file (without .)")
    parser.add_argument("-i", "--inputfile", help="The file that is to be used. Leave empty to use Pepper's camera")
    args = parser.parse_args()

    runtime = -1
    filetype = "mp4"
    mode = "***PEPPER-CAMERA***"
    if args.runtime:
        runtime = args.runtime
    if args.filetype:
        filetype = args.filetype
    if args.inputfile:
        mode = args.inputfile

    # Init ROS
    rospy.init_node('object_recognition')

    # Start the streamer
    if mode == "***PEPPER-CAMERA***":
        streamer = VideoStreamer(runtime, filetype)
    else:
        streamer = VideoStreamerFile(mode)
    streamer.start()
