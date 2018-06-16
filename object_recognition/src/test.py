#!/usr/bin/python

# Test Program
#   by tHE iNCREDIBLE mACHINE
#
# A program to test things for Timb

import time
import os
import sys
from progress_bar_27 import ProgressBar

# Print but without newline (technically it goes one up and down, but result is
#   the same)
def write (text, end=""):
    print(text + end + "\033[F")


def main ():
    # Try progress bar
    progress_bar_1 = ProgressBar(max_amount=100, ending_character="")
    progress_bar_2 = ProgressBar(max_amount=100)
    for i in range(200):
        progress_bar_1.update()
        if i % 2 == 0:
            print("")
            progress_bar_2.update()
            write("\033[F", end="")
        # Wait .1 sec
        time.sleep(.1)
    print("\nDone.")




# Entry point
if __name__ == "__main__":
    # Run main() with keyboard interrupt handler
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit()
