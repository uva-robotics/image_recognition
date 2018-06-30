#!/usr/bin/python

import argparse
import sys
import rospy
import cv2
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# WEBCAM BRIDGE
#   by tHE iNCREDIBLE mACHINE
#
# Reads WebCam frames, sends them on ROS (Pepper replacement)


##################### CHANGE LOG #####################
## v0.1 (alpha):                                    ##
##  + Begun alpha development                       ##
##  + Started on basic funcitonality: reading       ##
##    webcam frame-by-frame, sending them on ROS    ##
######################################################
## v0.2 (alpha):                                    ##
##  + Added timeout argument, so the function is    ##
##    actually quitabble                            ##
######################################################
## v0.3 (alpha):                                    ##
##  + Added that the program prints the starting    ##
##    time (HH:mm:ss), for reference                ##
######################################################
## v1.0 (release):                                  ##
##  + Finilized code                                ##
##  o Switched dev status to 'release'              ##
######################################################



# Some nice wrapping
def main (topic, timeout):
    # Welcoming message
    print("\n###################")
    print("## WEBCAM BRIDGE ##")
    print("##     v 0.3     ##")
    print("###################\n")

    print("Timeout:             {}s".format(timeout if timeout > -1 else u"\u221E".encode("utf-8")))
    print("Publishing on topic: {}\n".format(topic))

    # Create node, init ROS
    rospy.init_node("WebCamBridge")

    publisher = rospy.Publisher(topic, Image, queue_size=10)

    # Create openCV with webcame
    cap = cv2.VideoCapture(0)
    bridge = CvBridge()

    print("Started capture" + (time.strftime(" (%H:%M:%S)") if timeout > -1 else ""))

    then = time.time()
    last_second = -1
    while cap.isOpened():
        try:
            ret, frame = cap.read()
            if ret:
                # Now use CvBridhe to convert to img
                try:
                    to_send = bridge.cv2_to_imgmsg(frame, 'bgr8')
                    # Publish
                    publisher.publish(to_send)
                except CvBridgeError as e:
                    print(e)
            if timeout > -1 and time.time() - then > timeout:
                print("Running time done, {} seconds passed".format(timeout))
                break
            cv2.waitKey(1)
        except KeyboardInterrupt:
            break

    print("Stopping capture...")
    cap.release()
    cv2.destroyAllWindows()

    print("Done.")

# Entry point
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-to", "--topic", help="The name of the topic that the webcam data will be published on.")
    parser.add_argument("-ti", "--timeout", type=int, help="The amount of seconds the program will run before quitting. To set for endless, use -1 (NOT RECOMMENDED)")
    args = parser.parse_args()

    topic = "/image_raw"
    timeout = 30
    if args.topic:
        topic = args.topic
    if args.timeout:
        timeout = args.timeout

    # Run main with KeyboardInterrupt handler
    try:
        main(topic, timeout)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit()
