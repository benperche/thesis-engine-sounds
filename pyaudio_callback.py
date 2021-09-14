#! /usr/bin/env python
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

import time

import audiodevices
import roslistener
from tone import Tone

import config

# ROS
import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8MultiArray, Int32, Bool
# from geometry_msgs.msg import PoseStamped


AUDIOTIME = time.time()

# Instantiate a list of tone objects with relative harmonic ratios and
# amplitudes
ClassTones = [Tone(1, 0.3), Tone(1.5, 0.2), Tone(2, 0.05)]


# Create array of signed ints to hold one sample buffer
# Make it global so it doesn't get re-allocated for every frame
# outbuf2 = array.array('h', range(config.FRAMES_PER_BUFFER*config.CHANNELS))
# outbuf = np.zeros(config.FRAMES_PER_BUFFER*config.CHANNELS,dtype=np.int16)


# Callback function which is called by pyaudio
#   whenever it needs output-data or has input-data
def audioCallback(in_data, frame_count, time_info, status):
    global ClassTones
    global outbuf
    global AUDIOTIME

    # print('Audio Callback')
    since_last = time.time() - AUDIOTIME
    print('Since last call ', since_last)
    AUDIOTIME = time.time()
    # Index for samples to store in output buffer
    s = 0

    # Loop through number of frames to output
    for f in range(frame_count):

        # Clear any existing stored value in the output buffer
        outbuf[s] = 0

        # Loop through the number of Tones
        for currentTone in ClassTones:

            # Calculate the value to store in the outbuf
            next_val = int(32767 * currentTone.amplitude *
                           np.sin(currentTone.phase))

            # Update phase of this tone to compute the next value
            currentTone.updatePhase()

        # Loop through number of output channels to put the same value in each
        for c in range(config.CHANNELS):

            # Add this to existing values (eg previous tones in loop)
            outbuf[s] += next_val

            # Move along to next sample in outbuf once worked through all the
            # tones to be output
            s += 1

            # print("s ", s, "value ", next_val)

    # Convert output buffer to immutable bytes array
    out = bytes(outbuf)

    since_start = time.time() - AUDIOTIME
    print('Callback ', since_start)
    AUDIOTIME = time.time()

    return (out, pyaudio.paContinue)


# Callback function which is called by pyaudio
#   whenever it needs output-data or has input-data
def audioCallback2(in_data, frame_count, time_info, status):
    global ClassTones
    # global outbuf
    global AUDIOTIME

    # print('Audio Callback')
    since_last = time.time() - AUDIOTIME
    print('Since last call ', since_last)
    AUDIOTIME = time.time()
    # Index for samples to store in output buffer
    s = 0



    # Clear any existing stored value in the output buffer
    outbuf = np.zeros(config.FRAMES_PER_BUFFER*config.CHANNELS,dtype=np.int16)
    next_val = np.zeros(config.FRAMES_PER_BUFFER,dtype=np.int16)

    # Loop through the number of Tones
    for currentTone in ClassTones:

        # Calculate the value to store as integer array
        next_val += (32767 * currentTone.amplitude *
                       np.sin(currentTone.phaseArray)).astype(int)

        # Update phase of this tone to compute the next value
        currentTone.updatePhaseArray()

    # Loop through number of output channels to put the same value in each
    # for c in range(config.CHANNELS):
    #
    #
    #     # outbuf += next_val
    #
    #     # Move along to next sample in outbuf once worked through all the
    #     # tones to be output
    #     s += 1
    #
    #     # print("s ", s, "value ", next_val)

    # Add this to existing values (eg previous tones in loop)
    # Odd numbers only
    outbuf[::2]  += next_val[:]

    # Even numbers only
    outbuf[1::2]  += next_val[:]

    # Convert output buffer to immutable bytes array
    out = bytes(outbuf)

    since_start = time.time() - AUDIOTIME
    print('Callback ', since_start)
    AUDIOTIME = time.time()

    return (out, pyaudio.paContinue)

#########################
# Start of main program #
#########################

def main():
    global ClassTones
    # AUDIOTIME = datetime.datetime.now()

    # Setup ROS Node
    # rospy.init_node('audio_listener')
    audio_autonomous = roslistener.AudioAutonomous()

    # get a handle to the pyaudio interface
    paHandle = pyaudio.PyAudio()

    audiodevices.showDevices(paHandle)

    # select a device
    audiodevices.outputDevice = audiodevices.setDefaultOutputDevice(paHandle)
    devinfo = paHandle.get_device_info_by_index(audiodevices.outputDevice)
    print("Selected device name: ", devinfo.get('name'))
    print(devinfo)

    support = paHandle.is_format_supported(rate=config.RATE,input_device=None,
                input_format=None,output_device=audiodevices.outputDevice, output_channels=2,
                output_format=paHandle.get_format_from_width(config.WIDTH))

    print('Is format supported: ', support)

    print('AUDIOTIME', AUDIOTIME)

    # open a stream with some given properties
    stream = paHandle.open(format=paHandle.get_format_from_width(config.WIDTH),
                           channels=config.CHANNELS,
                           rate=config.RATE,
                           frames_per_buffer=config.FRAMES_PER_BUFFER,
                           input=False,  # no input
                           output=True,  # only output
                           output_device_index=audiodevices.outputDevice,
                           stream_callback=audioCallback2)

    stream.start_stream()

    # Setup frequency sweep values to be used in loop
    count = 0
    ascending = True

    # Temporary sweep parameters
    UPPER_FREQ = 250
    LOWER_FREQ = 175
    STEP = 0.25

    # Loop while new info comes in
    while stream.is_active():

        # rospy.spin()

        # Print Timing info
        # print('Audio rate ', 1/AUDIOTIME, ' ROS rate ', 1/AudioAutonomous.ROSTIME)

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

    stream.stop_stream()
    stream.close()
    print("Stopped")

    paHandle.terminate()


if __name__ == '__main__':
    main()
