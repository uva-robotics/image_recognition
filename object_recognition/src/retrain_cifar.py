#!/usr/bin/python

# RETRAIN
#   by tHE iNCREDIBLE mACHINE
#
# Retrain script for OpenCV. Uses a local DB of cifar-100 instead of online
#   imagenet.

import time
import sys
import os
import argparse
import subprocess

from progress_bar_27 import ProgressBar
from progress_bar_27 import WaitIndicator
from PIL import Image


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


def file_exists (path):
    try:
        f = open(path, "r")
        f.close()
        return True
    except IOError:
        return False

# Main
def main (positives_path, negatives_path):
    # Welcome message
    print("\n#########################")
    print("## RETRAIN w/CIFAR-100 ##")
    print("##        v 1.0        ##")
    print("#########################\n")

    print("Indexing data...")
    positives_files = []
    for file in os.listdir(positives_path):
        if file.split(".")[-1] == "png":
            img = Image.open(positives_path + file)
            width, height = img.size
            if width == 32 and height == 32:
                positives_files.append(file)
    negatives_files = []
    counter = 0
    for file in os.listdir(negatives_path):
        if file.split(".")[-1] == "png":
            img = Image.open(negatives_path + file)
            width, height = img.size
            if width == 32 and height == 32:
                negatives_files.append(file)
        counter += 1
        if counter > float(len(positives_files)) / 2.0:
            break
    print("Done (found {} pictures, from which {} positives and {} negatives)".format(len(positives_files) + len(negatives_files), len(positives_files), len(negatives_files)))

    # Check if there's
    if not file_exists(negatives_path + "bg.txt"):
        # Not given, generate
        bg_path = negatives_path + "bg.txt"

        print("Creating bg file (negatives)... ({})".format(bg_path))

        f = open(bg_path, "w")
        # Loop through all da files
        progress_bar = ProgressBar(max_amount=len(negatives_files))
        for filename in negatives_files:
            # Get image size
            img = Image.open(negatives_path + filename)
            width, height = img.size
            # Assemble the line
            line = [
                filename,
                "1",
                "0",
                "0",
                str(width),
                str(height)
            ]
            # Write the list
            f.write(" ".join(line) + "\n")
            progress_bar.update()
        f.close()
        print("Done")

    # Check if info file is given. Otherwise, generate new one.
    if not file_exists(positives_path + "info.list"):
        # No given, generate:
        info_path = positives_path + "info.lst"

        print("Creating info file (positives)... ({})".format(info_path))

        f = open(info_path, "w")
        # Loop through all files
        progress_bar = ProgressBar(max_amount=len(positives_files))
        for filename in positives_files:
            # Get image size
            img = Image.open(positives_path + filename)
            width, height = img.size
            # Assemble the line
            line = [
                filename,
                "1",
                "0",
                "0",
                str(width),
                str(height)
            ]
            # Write the list
            f.write(" ".join(line) + "\n")
            progress_bar.update()
        f.close()
        print("Done")

    # Time to build the vector if it doesn't exist already
    print("Creating vector...\033[F\r")
    if file_exists(positives_path + "positives.vec"):
        print("\n{}  - Vector already exists, skipping...{}".format(bcolors.OKGREEN, bcolors.ENDC))
    else:
        # Do a nice waitingdicator
        wait_indicator = WaitIndicator(WaitIndicator.CircleAnimation, preceding_text = "Creating vector... ", end_text="Creating vector... Done", refresh_rate = 0.2, automatic = True)
        wait_indicator.start()
        # Well, call the opencv thing
        vector_creation = subprocess.Popen(["opencv_createsamples", "-info", positives_path + "info.lst", "-num", str(len(positives_files)), "-vec", positives_path + "positives.vec"], stdout=subprocess.PIPE, stdin=subprocess.PIPE)
        output_normal, output_error = vector_creation.communicate()
        wait_indicator.stop()
        print("OUTPUT:")
        for line in output_normal.split("\n"):
            print("  > {}".format(line))
        if output_error != None:
            print("{}" + output_error + "{}".format(bcolors.FAIL, bcolors.ENDC))
    print("Done")

    # Starting training...
    print("Everything seems in order, ready to train? (y/n)")
    y_n = None
    while y_n == None:
        result = raw_input()
        y_n = (result == "y") if (result == "y" or result == "n") else None
    if y_n:
        wait_indicator = WaitIndicator(WaitIndicator.CircleAnimation, preceding_text = "Training... ", end_text="Training... Done", refresh_rate = 0.2, automatic = True)
        wait_indicator.start()
        # He ho let's go

        # Create result path
        result_path = positives_path + "data/"
        if not os.path.exists(result_path):
            os.makedirs(result_path)

        # Do the result
        training = subprocess.Popen(["opencv_traincascade", "-data", result_path, "-vec", positives_path + "positives.vec", "-bg", negatives_path + "bg.txt", "-numPos", str(len(positives_files)), "-numNeg", str(len(negatives_files)), "-numStages", "10"], stdout=subprocess.PIPE, stdin=subprocess.PIPE)
        output_normal, output_error = training.communicate()

        # Done, stop wait_indicator and show output
        wait_indicator.stop()

        print("OUTPUT:")
        for line in output_normal.split("\n"):
            print("  > {}".format(line))
        if output_error != None:
            print("{}" + output_error + "{}".format(bcolors.FAIL, bcolors.ENDC))
    else:
        print("Aborted.")
        sys.exit()

    print("\n{}Successfully trained!{}\n".format(bcolors.OKGREEN, bcolors.ENDC))

# Entry point
if __name__ == "__main__":
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("--std_paths", action="store_true", help="Returns all standard paths used in this script.")
    parser.add_argument("-p", "--positives", help="The path to the folder containing the positives. All output will also be generated in this folder.")
    parser.add_argument("-n", "--negatives", help="The path to the folder containing the negatives.")
    args = parser.parse_args()

    positives_path = "/VirtualShare/cifar-100-images/train/household_electrical_devices/keyboard"
    negatives_path = "/VirtualShare/cifar-100-images/train/household_furniture/table"
    info_file = ""
    if args.std_paths:
        print("Standard paths:")
        print("  - Positives: {}".format(positives_path))
        print("  - Negatives: {}".format(negatives_path))
        sys.exit()
    if args.positives:
        positives_path = args.positives
    if args.negatives:
        negatives_path = args.negatives

    # Make sure positives_path closes with /
    if positives_path[-1] != "/":
        positives_path += "/"
    if negatives_path[-1] != "/":
        negatives_path += "/"

    # Run main with KeyboardInterrupt callback
    try:
        main(positives_path, negatives_path)
    except KeyboardInterrupt:
        print("{}\nInterrupted by user{}".format(bcolors.OKBLUE, bcolors.ENDC))
        sys.exit()
