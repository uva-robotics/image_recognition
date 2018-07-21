#!/usr/bin/python

# OBJECT RECOGNITION
#   by tHE iNCREDIBLE mACHINE
#
# A nice script to test in

import lightnet

print("Loading weights...")
model = lightnet.load("yolo")
print("Loading image...")
image = lightnet.Image.from_bytes(open('/VirtualShare/dog.jpg', 'rb').read())
print("Classifying...")
boxes = model(image)
print(boxes)
print("Done.")
