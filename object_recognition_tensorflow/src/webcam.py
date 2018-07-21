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
## v1.0.0 (release):                                ##
##  o Finilized code                                ##
##  o Switched dev status to 'release'              ##
######################################################
## v1.1.0:                                          ##
##  + Added option to stream video to file instead  ##
##    of ROS topic                                  ##
######################################################
## v1.1.1:                                          ##
##  o Fixed bug that the script would attempt to    ##
##    close VideoWriter when it didn't use it       ##
######################################################
## v1.1.2:                                          ##
##  + Added feedback for when webcam is             ##
##    suspiciously shortly open                     ##
######################################################



# Some nice wrapping
def main (topic, timeout, path):
    # Welcoming message
    print("\n###################")
    print("## WEBCAM BRIDGE ##")
    print("##    v 1.1.1    ##")
    print("###################\n")

    print("Timeout:             {}s".format(timeout if timeout > -1 else u"\u221E".encode("utf-8")))
    print("Publishing on topic: {}\n".format(topic))

    # Create openCV with webcam
    cap = cv2.VideoCapture(0)

    # Prepare streaming
    if len(path) == 0:
        rospy.init_node("WebCamBridge")
        publisher = rospy.Publisher(topic, Image, queue_size=10)
    else:
        fourcc = cv2.VideoWriter_fourcc(*'X264')
        width = cap.get(3)
        height = cap.get(4)
        writer = cv2.VideoWriter(path, fourcc, 30.0, (int(width), int(height)))

    # Do the bridge
    bridge = CvBridge()

    print("Started capture" + (time.strftime(" (%H:%M:%S)") if timeout > -1 else ""))

    then = time.time()
    while cap.isOpened():
        try:
            ret, frame = cap.read()
            if ret:
                try:
                    # Stream
                    if len(path) == 0:
                        # Use CvBridge to convert to img
                        to_send = bridge.cv2_to_imgmsg(frame, 'bgr8')
                        publisher.publish(to_send)
                    else:
                        # Write instead
                        writer.write(frame)
                except CvBridgeError as e:
                    print(e)
            if timeout > -1 and time.time() - then > timeout:
                print("Running time done, {} seconds passed".format(timeout))
                break
            cv2.waitKey(1)
        except KeyboardInterrupt:
            break
    end = time.time()

    print("Stopping capture...")
    if len(path) > 0:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()

    print("Done.")

    if (timeout == -1 or timeout > 0.1) and end - then < 0.1:
        print("Could not read webcam (is it connected?)")

# Entry point
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-to", "--topic", help="The name of the topic that the webcam data will be published on.")
    parser.add_argument("-ti", "--timeout", type=int, help="The amount of seconds the program will run before quitting. To set for endless, use -1 (NOT RECOMMENDED)")
    parser.add_argument("-p", "--path", help="If given, will output the stream to a file specified in the given path (will not publish)")
    args = parser.parse_args()

    topic = "/image_raw"
    timeout = 30
    path = ""
    if args.topic:
        topic = args.topic
    if args.timeout:
        timeout = args.timeout
    if args.path:
        path = args.path

    # Run main with KeyboardInterrupt handler
    try:
        main(topic, timeout, path)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit()
