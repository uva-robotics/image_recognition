#!/usr/bin/python

# OBJECT RECOGNITION
#   by tHE iNCREDIBLE mACHINE
#
# A nice script to test in


import cv2

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


print("{}HEADER{}".format(bcolors.HEADER, bcolors.ENDC))
print("{}OKBLUE{}".format(bcolors.OKBLUE, bcolors.ENDC))
print("{}OKGREEN{}".format(bcolors.OKGREEN, bcolors.ENDC))
print("{}WARNING{}".format(bcolors.WARNING, bcolors.ENDC))
print("{}FAIL{}".format(bcolors.FAIL, bcolors.ENDC))
print("{}BOLD{}".format(bcolors.BOLD, bcolors.ENDC))
print("{}UNDERLINE{}".format(bcolors.UNDERLINE, bcolors.ENDC))

print(cv2.__version__)
