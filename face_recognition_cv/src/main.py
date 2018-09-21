#!/usr/bin/python

import collections
import sys
import time
import os
import cPickle as pickle

import numpy as np
import cv2
import rospy
import face_recognition

from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

FACE_ENCODINGS_PATH = './face_encodings.p'

class FaceRecognition():

    def __init__(self):
        self.skip_frame = False

        if os.path.isfile(FACE_ENCODINGS_PATH):
            self.face_encodings = pickle.load(open(FACE_ENCODINGS_PATH, "rb"))
        else:
            self.face_encodings = []
            self.face_labels = []

        self.face_names = []
        self.face_count = 0

        self.is_recognizing = None

        rospy.init_node('face_recognition', anonymous=True)
        rospy.Subscriber("/pepper_robot/camera/front/image_raw/compressed", CompressedImage, self.callback, queue_size=1)
        self.dialogflow = rospy.Publisher('/dialogflow', String, queue_size=10)


    def flow(self, text):
        self.dialogflow.publish(String(text))


    def callback(self, data):
        self.skip_frame = not self.skip_frame
        if self.skip_frame or self.is_recognizing is not None:
            return

        image_array = np.fromstring(data.data, np.uint8)
        cv_image = cv2.imdecode(image_array, cv2.IMREAD_COLOR)

        image = cv_image[:, :, ::-1]

        face_locations = face_recognition.face_locations(image)
        face_encodings = face_recognition.face_encodings(image, face_locations)

        face_names = []
        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.face_encodings, face_encoding, tolerance=0.6)

            if True in matches:
                index = matches.index(True)
                name = str(index)

            else:
                name = str(self.face_count)
                self.face_count += 1
                self.face_encodings.append(face_encoding)
                self.face_names += name
                self.flow('Hello there new face! What is your name?')
                pickle.dump(self.face_encodings, open(FACE_ENCODINGS_PATH, "wb"))
                # self.is_recognizing = face_encoding

            face_names.append(name)

        # self.flow('I see {} right now'.format(', '.join(face_names)))

        for (top, right, bottom, left), name in zip(face_locations, face_names):
           cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 0, 255), 2)

           cv2.rectangle(cv_image, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
           font = cv2.FONT_HERSHEY_DUPLEX
           cv2.putText(cv_image, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

        cv2.namedWindow('Camera', cv2.WINDOW_NORMAL)
        cv2.imshow('Camera', cv_image)
        cv2.waitKey(1)


if __name__ == '__main__':
    fr = FaceRecognition()
    rospy.spin()
