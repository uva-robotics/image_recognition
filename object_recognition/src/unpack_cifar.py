#!/usr/bin/python

# RETRAIN
#   by tHE iNCREDIBLE mACHINE
#
# Retrain script for OpenCV. Uses a local DB of cifar-100 instead of online
#   imagenet.

import time
import sys
import os
import cPickle
import argparse
import numpy

from progress_bar_27 import ProgressBar
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

# Main
def main (input_path, output_path, metadata_path):
    # Welcome message
    print("\n######################")
    print("## UNPACK CIFAR-100 ##")
    print("##       v1.0       ##")
    print("######################\n")

    # Load the file using cPickle
    print("Loading file...")
    try:
        with open(input_path, "rb") as f:
            database = cPickle.load(f)
    except IOError as e:
        print("ERROR: Could not open file:")
        print(e)
        sys.exit()
    print("Done")
    # Load the metadata
    print("Loading file...")
    try:
        with open(metadata_path, "rb") as f:
            metadata = cPickle.load(f)
    except IOError as e:
        print("{}ERROR{}: Could not open file:".format(bcolors.FAIL, bcolors.ENDC))
        print(e)
        sys.exit()
    print("Done")

    # Now start converting to pictures

    # Check if folder exists
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    counter = 0
    progress_bar = ProgressBar(max_amount = len(database["data"]))
    print("Unpacking pictures...")
    for index, picture in enumerate(database["data"]):
        # Let's assemble RGB tuples for each pixel in picture
        RGBs = {}
        for i in range(len(picture)):
            if i % 1024 not in RGBs:
                RGBs[i % 1024] = {}
            colour = "R"
            if i > 1023 and i < 2048:
                colour = "G"
            if i > 2047:
                colour = "B"
            RGBs[i % 1024][colour] = picture[i]
        # Time to convert it to list of tuples
        RGB_tuples = []
        for key in RGBs:
            pixel = RGBs[key]
            RGB_tuples.append((pixel["R"], pixel["G"], pixel["B"]))

        # Alright, let's do it in a picture
        image = Image.new("RGB", (32, 32))
        image.putdata(RGB_tuples)

        # Get the labels to make a directory
        crude_label = metadata["coarse_label_names"][database["coarse_labels"][index]]
        fine_label = metadata["fine_label_names"][database["fine_labels"][index]]
        directory = output_path + crude_label + "/" + fine_label + "/"

        # Make sure directory exists
        if not os.path.exists(directory):
            os.makedirs(directory)

        # Get outselves a filename
        files = os.listdir(directory)
        new_counter = -1
        for file in files:
            counter = int(file.split(".")[0])
            if counter > new_counter:
                new_counter = counter
        filename = str(new_counter + 1) + ".png"

        image.save(directory + filename)
        counter += 1
        progress_bar.update()
    print("Done")

    print("\n{}Successfully unpacked {} pictures.\n{}".format(bcolors.OKGREEN, counter, bcolors.ENDC))


# Entry point
if __name__ == "__main__":
    # Get arguments
    parser = argparse.ArgumentParser()
    # Function arguments
    parser.add_argument("--std_paths", action="store_true", help="Spits out the standard paths.")

    parser.add_argument("-i", "--input_path", help="The path to the cifar-100 database file")
    parser.add_argument("-o", "--output_path", help="The path to the folder in which the pictures will be written")
    parser.add_argument("-m", "--metadata_path", help="The path to the file which contains the labelnames (aka metadata)")
    args = parser.parse_args()

    input_path = "/Users/tim/VirtualShare/cifar-100-python/train"
    output_path = "/Users/tim/VirtualShare/cifar-100-images/train"
    metadata_path = "/Users/tim/VirtualShare/cifar-100-python/meta"
    if args.std_paths:
        print("Standard input path: {}".format(input_path))
        print("Standard output path: {}".format(output_path))
        print("Standard meta path: {}".format(metadata_path))
        sys.exit()
    if args.input_path:
        input_path = args.input_path
    if args.output_path:
        output_path = args.output_path
    if args.metadata_path:
        metadata_path = args.metadata_path

    # Make sure to end output_path = "/"
    if output_path[-1] != "/":
        output_path += "/"

    # Run main with KeyboardInterrupt callback
    try:
        main(input_path, output_path, metadata_path)
    except KeyboardInterrupt:
        print("\n{}Interrupted by user{}".format(bcolors.OKBLUE, bcolors.ENDC))
        sys.exit()
