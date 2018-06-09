#!/usr/bin/python

# Object Recognition - Retrainer
#   by tHE iNCREDIBLE mACHINE
#
# This script ought to train a machine learning algorithm in OpenCV. Data
#   examples are downloaded from ImageNet (image-net.org)

import urllib2
import cv2
import numpy as np
import os
import argparse
import sys

# Fancy, easy to use progress bar
class ProgressBar():
    # Init the class
    def __init__(self, width=-1, min_amount=0, max_amount=99):
        if width < 0:
            _, width = os.popen('stty size', 'r').read().split()
            width = int(width)
        self.width = width - 10
        self.step = min_amount
        self.max_amount = max_amount

    # Draw the progressbar
    def draw (self):
        percentage = self.step / self.max_amount
        bar = " {:5.1f}% ".format(percentage * 100)
        bar += "["
        bar += "=" * (int(self.width * percentage))
        bar += " " * ((self.width) - int(self.width * percentage))
        bar += "]"
        print bar + "\r",
        if percentage >= 1:
            # We're done
            self.end()

    # Only update step with amount
    def update_only (self, amount=1):
        self.step += amount

    # Update step with amount and draw
    def update (self, amount=1):
        self.update_only(amount)
        self.draw()

    # Only sets step to amount
    def set_only (self, amount):
        self.step = amount

    # Sets step to amount and draws
    def set (self, amount):
        self.set_only(amount)
        self.draw()

    # End the program
    def end (self, clean=False, ending_character = "\n"):
        if clean:
            print " " * (self.width + 10) + "\r",
        print ending_character,

def download_urls (urls, amount, width, height, path):
    print("\nPreparing for download...")
    counter = 0
    downloads = 0
    compressed = 0
    _, screen_width = os.popen('stty size', 'r').read().split()
    screen_width = int(screen_width)

    print("Downloading and resizing images...")
    check = False
    while counter < amount:
        for url in urls.split("\n"):
            try:
                counter += 1
                # Check if we're done
                if counter > amount:
                    # Done
                    break
                print(" > Downloading image {}/{}...".format(counter, amount))
                f = urllib2.urlopen(url)
                with open(path + str(counter) + ".jpg", "wb") as file:
                    file.write(f.read())
                downloads += 1

                img = cv2.imread(path + str(counter) + ".jpg",cv2.IMREAD_GRAYSCALE)
                # The size of the image is twice that of the positive size
                print("  - Compressing...")
                try:
                    resized_image = cv2.resize(img, (width, height))
                except Exception as e:
                    print("  - Could not resize image")
                else:
                    print("  - Writing to disk...")
                    cv2.imwrite(path + str(counter) + ".jpg", resized_image)

                compressed += 1
            except Exception as e:
                print("  - {}".format(e))
        if counter < amount:
            # We're not done yet:
            print("Downloaded all images from this url. Please provide another:")
            url = input()
            urls = urllib.urlopen(url).read().decode()
    print("Done (downloaded {} picture".format(downloads) + ("" if downloads == 1 else "s") + ", compressed {})".format(compressed))


# Entry point
if __name__ == "__main__":
    # Parameters
    # Get arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--positives_url", help="The url to which the script will look for positives (should be ImageNet link)")
    parser.add_argument("-n", "--negatives_url", help="The url to which the script will look for negatives (should be ImageNet link)")
    parser.add_argument("-pn", "--positives_amount", type=int, help="The amount of positives that should be downloaded (default: 1)")
    parser.add_argument("-nn", "--negatives_amount", type=int, help="The amount of negatives that should be downloaded (default: 5000)")
    parser.add_argument("-wp", "--width", type=int, help="The image width, in pixels (default: 50)")
    parser.add_argument("-hp", "--height", type=int, help="The image height, in pixels (default: 50)")
    parser.add_argument("-pa", "--path", help="The path to where everything will be downloaded")
    args = parser.parse_args()

    positives_url = ""
    negatives_url = ""
    path = ""
    positives_amount = 1
    negatives_amount = 5000
    width = 50
    height = 50
    if args.positives_url:
        positives_url = args.positives_url
    else:
        # Ask the user
        print("Enter the url to the synset of the positives:")
        positives_url = input()
    if args.negatives_url:
        negatives_url = args.negatives_url
    else:
        # Ask the user
        print("Enter the url to the synset of the negatives:")
        negatives_url = input()
    if args.path:
        path = args.path
    else:
        # Ask the user
        print("Enter the download path of the photos:")
        path = input()

    if not os.path.exists(path):
        os.makedirs(path)
    if not os.path.exists(path + "positives/"):
        os.makedirs(path + "positives/")
    if not os.path.exists(path + "negatives/"):
        os.makedirs(path + "negatives/")

    if path[-1] != "/":
        path += "/"

    # Others
    if args.positives_amount:
        positives_amount = args.positives_amount
    if args.negatives_amount:
        negatives_amount = args.negatives_amount
    if args.width:
        width = args.width
    if args.height:
        height = args.height

    print("Script is using:")
    print(" > Positives URL: {}".format(positives_url))
    print(" > Negatives URL: {}".format(negatives_url))
    print(" > Path: {}".format(path))
    print(" > No. of Positives: {}".format(positives_amount))
    print(" > No. of Negatives: {}".format(negatives_amount))
    print(" > Picture width: {}".format(width))
    print(" > Picture height: {}".format(height))

    # Now we have them, start downloading the URL list of the pictures
    print("Retrieving URL lists...")
    positives_urls = urllib2.urlopen(positives_url).read().decode()
    negatives_urls = urllib2.urlopen(negatives_url).read().decode()
    print("Done")

    # Download the positive urls
    download_urls(positives_urls, positives_amount, width, height, path + "positives/")
    # Download the negative urls
    download_urls(negatives_urls, negatives_amount, width * 2, height * 2, path + "negatives/")
