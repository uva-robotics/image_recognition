#!/usr/bin/python

# C_INTERFACE.py
#
# This script attempts to convert numpy arrays to c_type pointer with floats.

##################### CHANGE LOG #####################
## v0.1.0 (alpha):                                  ##
##  + Begun alpha development                       ##
######################################################

import ctypes
import numpy as np

def convert (array):
    return array.ctypes.data_as(ctypes.POINTER(ctypes.c_float))

if __name__ == "__main__":
    arr = np.array([[[1,2,3,4,5]]])
    print (convert(arr))
