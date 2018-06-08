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

class VideoStreamer ():
    # INit
    def __init__(self, running_time = -1, filetype = "mp4"):
        self.subscriber = rospy.Subscriber("/pepper_robot/camera/front/image_raw", Image, self.callback)
        self.running = False
        self.idle = True
        self.running_time = running_time
        self.extension = filetype
        print("VideoStreamer > Created")

    # Start
    def start (self):
        print("VideoStreamer > Starting...")
        if not self.running:
            self.buffer = []
            self.running = True
            self.first_time = True
            self.start_time = time.time()

            # Setup video reader
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')

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
                self.out = cv2.VideoWriter(output, self.fourcc, 20.0, (data.width, data.height))

            # Write away
            self.buffer.append(data.data)

            self.idle = True

    # Stop it
    def stop (self):
        print("VideoStreamer > Closing...")
        if self.running:
            self.running = False
            # Wait until idle
            while not self.idle:
                pass
            # Close the file if open
            if not self.file.closed:
                self.file.close()
            print("VideoStreamer > Closed successfully")
        else:
            print("VideoStreamer > Already closed")

# Entry point
if __name__ == '__main__':
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime", type=int, help="The time (in seconds) the script will run")
    parser.add_argument("--filetype", help="The extension of the stream file (without .)")
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
