import time
import os
import threading

# Progress Bar
class ProgressBar():
    # Init the class
    def __init__(self, width=-1, min_amount=0, max_amount=99, refresh_rate=0.5, preceding_text="", ending_character="\n"):
        if width == -1:
            _, screen_width = os.popen('stty size', 'r').read().split()
            width = int(screen_width)
        self.total_width = width
        self.preceding_text = preceding_text
        self.ending_character = ending_character
        self.width = self.total_width - 10 - len(self.preceding_text)
        self.step = min_amount
        self.max_amount = max_amount
        self.last_refresh = -1
        self.refresh_rate = refresh_rate
        self.ended = False

    # Print without newline (necessarily)
    def write (self, text, end=""):
        print(text + end + "\033[F")

    # Draw the progressbar
    def draw (self):
        percentage = float(self.step) / float(self.max_amount)
        bar = self.preceding_text
        bar += " {:5.1f}% ".format(percentage * 100)
        bar += "["
        bar += "=" * (int(self.width * percentage))
        bar += " " * ((self.width) - int(self.width * percentage))
        bar += "]"
        self.write(bar,end="\r")
        if percentage >= 1 and not self.ended:
            # We're done
            self.end(ending_character = self.ending_character)
            self.ended = True

    # Only draw once in refresh_rate
    def draw_timed (self):
        if time.time() - self.last_refresh > self.refresh_rate or self.last_refresh == -1:
            if self.last_refresh == -1:
                self.last_refresh = time.time()
            else:
                self.last_refresh += self.refresh_rate
            self.draw()

    # Only update step with amount
    def update_only (self, amount=1):
        self.step += amount
        if self.step > self.max_amount:
            self.step = self.max_amount
        if self.step >= self.max_amount:
            self.draw()

    # Update step with amount and draw
    def update (self, amount=1):
        self.update_only(amount)
        if self.step < self.max_amount:
            self.draw_timed()

    # Only sets step to amount
    def set_only (self, amount):
        self.step = amount
        if self.step > self.max_amount:
            self.step = self.max_amount
        if self.step >= self.max_amount:
            self.draw()

    # Sets step to amount and draws
    def set (self, amount):
        self.set_only(amount)
        if self.step < self.max_amount:
            self.draw_timed()

    # Updates preceding text
    def update_preceding_text (self, new_text):
        self.preceding_text = new_text
        self.width = self.total_width - 10 - len(self.preceding_text)
        self.draw()


    # End the program
    def end (self, clean=False, ending_character = "\n"):
        if clean:
            self.write("\033[K",end="\r")
        self.write("",end=ending_character)

class WaitIndicator (threading.Thread):
    # Animations
    class DotsAnimation ():
        # SIZE: No. of dots that is displayed at maximum depth
        def __init__(self, size=3):
            self.step = 0
            self.size = size + 1

        # Returns the maximum length
        def __len__(self):
            return self.size

        # Gets the next to be printed step
        def next_step (self):
            to_return = "." * self.step
            self.step = (self.step + 1) % self.size
            return to_return

        # Get the current step value
        def get_step (self):
            return self.step

    class CircleAnimation ():
        # SIZE: ignored
        def __init__(self, size=0):
            self.step = 0
            self.steps = [
                "|",
                "/",
                "-",
                "\\"
            ]

        # Return the steps of the animation
        def next_step (self):
            to_return = self.steps[self.step % len(self.steps)]
            self.step += 1
            return to_return

    # Init WaitIndicator
    def __init__(self, animation, width=-1, preceding_text = "", end_text = "", refresh_rate = 1, automatic = False):
        self.running = False
        self.width = width
        self.refresh_rate = refresh_rate
        self.preceding_text = preceding_text
        self.end_text = end_text

        if animation == self.DotsAnimation:
            if self.width == -1:
                self.animation = self.DotsAnimation()
            else:
                self.animation = self.DotsAnimation(width)
        if animation == self.CircleAnimation:
            self.animation = self.CircleAnimation()

        self.automatic = automatic
        if self.automatic:
            threading.Thread.__init__(self)
            # Devise a unique name
            name = "WaitIndicator-"
            i = 0
            while True:
                if name + str(i) not in threading.enumerate():
                    break
                i += 1
            self.name = name + str(i)


    def write (self, text, end=""):
        print(text + end + "\033[F")

    # The poke function for non-threaded version
    def draw (self):
        if not self.automatic:
            self.run()

    # Update the preceding text
    def update_preceding_text (self, new_text):
        self.preceding_text = new_text

    # Start the indicator
    def start (self):
        self.last_refresh = time.time()
        self.running = True
        if self.automatic:
            threading.Thread.start(self)

    # The actual work function
    def run (self):
        while self.running:
            if time.time() - self.last_refresh > self.refresh_rate:
                self.last_refresh += self.refresh_rate
                self.write("\033[K",end="\r")
                self.write(self.preceding_text + self.animation.next_step(),end="\r")
            if not self.automatic:
                break

    # The stop function
    def stop (self):
        self.running = False
        # If automated, wait until dead
        if self.automatic:
            while self.isAlive():
                pass
        # If given, replace current line by end_text
        if self.end_text != "":
            self.write("\033[K",end="\r")
            self.write(self.end_text)
        # Add a newline for ends
        self.write("\n")
        # Done.
