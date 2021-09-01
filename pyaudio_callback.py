#
# Sinewave generator using pyaudio callback
# Dependencies: Portaudio, pyaudio - see dependency_instructions.txt
#
# Author: Ben Perche, Faculty of Engineering, University of Sydney
# Initial Code: (ME) 2015 Marc Groenewegen
# https://www.dinkum.nl/software/python/audio/basic_sine.py
#

import pyaudio
import numpy as np
import array

from audio_devices import *

# ROS
# import tf2_ros
# import tf2_geometry_msgs
#
# from nav_msgs.msg import Odometry, Path
# from std_msgs.msg import UInt8MultiArray, Int32, Bool
# from geometry_msgs.msg import PoseStamped

# Audio output constants
WIDTH = 2  # sample size in bytes
CHANNELS = 2
RATE = 44100
FRAMES_PER_BUFFER = 1024
INITIAL_FREQUENCY = 200

# Temporary sweep parameters
UPPER_FREQ = 250
LOWER_FREQ = 175
STEP = 0.25


class Tone:
    # Store data for each tone to be generated:
    # ratio = harmonic ratio (float) to fundamental frequency
    # amplitude = relative loudness of this tone (float 0-0.5 usually)
    #
    # automatically updated:
    # phase, frequency (both variable)

    # Create shared class/static variable for fundamental frequency
    fundFreq = INITIAL_FREQUENCY

    # Constructor
    def __init__(self, ratio, amplitude):
        self.ratio = ratio
        self.amplitude = amplitude
        self.phase = 0

        # Check if this is the first Tone that has been instantiated
        # if isinstance(Tone, type):
        if ratio == 1:
            # If not, apply ratio to determine initial frequency
            self.frequency = INITIAL_FREQUENCY * self.ratio
        else:
            # If it's the first, set initial frequency
            self.frequency = INITIAL_FREQUENCY

    def updatePhase(self):
        self.phase += 2 * np.pi * self.frequency/RATE

    # Update frequency with respect to fundamental
    def updateFrequency(self):
        self.frequency = Tone.fundFreq * self.ratio

        # If this tone is the fundamental, update the shared fundFreq variable
        if self.ratio == 1:
            Tone.fundFreq = self.frequency


# Instantiate a list of tone objects with relative harmonic ratios and
# amplitudes
ClassTones = [Tone(1, 0.3), Tone(1.5, 0.2), Tone(2, 0.05)]

# Global index for output device
outputDevice = 0

# Create array of signed ints to hold one sample buffer
# Make it global so it doesn't get re-allocated for every frame
outbuf = array.array('h', range(FRAMES_PER_BUFFER*CHANNELS))


# Callback function which is called by pyaudio
#   whenever it needs output-data or has input-data
def callback(in_data, frame_count, time_info, status):
    global ClassTones
    global outbuf

    # Index for samples to store in output buffer
    s = 0

    # Loop through number of frames to output
    for f in range(frame_count):

        # Loop through number of output channels
        for c in range(CHANNELS):

            # Clear any existing stored value in the output buffer
            outbuf[s] = 0

            # Loop through the number of Tones
            for currentTone in ClassTones:
                # print(range(len(Tones)-1))

                # Calculate the value to store in the outbuf
                next_val = int(32767 * currentTone.amplitude *
                               np.sin(currentTone.phase))

                # Add this to existing values (eg previous tones in loop)
                outbuf[s] += next_val

                # Update phase of this tone to compute the next value
                currentTone.updatePhase()

            # Move along to next sample in outbuf once worked through all the
            # tones to be output
            s += 1

            # Put the same sample in all channels

    # Convert output buffer to immutable bytes array
    out = bytes(outbuf)

    return (out, pyaudio.paContinue)


#########################
# Start of main program #
#########################

def main():
    global ClassTones

    # get a handle to the pyaudio interface
    paHandle = pyaudio.PyAudio()

    showDevices(paHandle)

    # select a device
    outputDevice = setDefaultOutputDevice(paHandle)
    devinfo = paHandle.get_device_info_by_index(outputDevice)
    print("Selected device name: ", devinfo.get('name'))
    print(devinfo)

    support = paHandle.is_format_supported(rate=RATE,input_device=None,
                input_format=None,output_device=outputDevice, output_channels=2,
                output_format=paHandle.get_format_from_width(WIDTH))

    print('Is format supported: ', support)

    # open a stream with some given properties
    stream = paHandle.open(format=paHandle.get_format_from_width(WIDTH),
                           channels=CHANNELS,
                           rate=RATE,
                           frames_per_buffer=FRAMES_PER_BUFFER,
                           input=False,  # no input
                           output=True,  # only output
                           output_device_index=outputDevice,
                           stream_callback=callback)

    stream.start_stream()

    # Setup frequency sweep values to be used in loop
    count = 0
    ascending = True

    # Loop while new info comes in
    while stream.is_active():

        # Gradually change direction of sine wave
        count += 1
        if count > 10000:
            count = 0

            # Change direction when out of bounds
            if Tone.fundFreq > UPPER_FREQ:
                ascending = False
            elif Tone.fundFreq < LOWER_FREQ:
                ascending = True

            # Update frequency of fundamental
            if ascending:
                Tone.fundFreq += STEP
            else:
                Tone.fundFreq -= STEP

            # Update frequencies of all other tones
            for currentTone in ClassTones:
                currentTone.updateFrequency()
                # = ClassTones[0].frequency * currentTone.ratio

    stream.stop_stream()
    stream.close()
    print("Stopped")

    paHandle.terminate()


if __name__ == '__main__':
    main()
