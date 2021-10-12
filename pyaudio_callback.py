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
import multiprocessing as mp

import time

import config
import audio_devices
import ros_interface
from tone import Tone, ClassTones

# ROS
import rospy
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import UInt8MultiArray, Int32, Bool
# from geometry_msgs.msg import PoseStamped

AUDIOTIME = time.time()
# stream = 0

# Initialise an array to compute the values to be output in each frame
sound_array = np.zeros(config.FRAMES_PER_BUFFER,dtype=np.int16)

# Initialise an output buffer for raw data
outbuf = np.zeros(config.FRAMES_PER_BUFFER*config.CHANNELS,dtype=np.int16)



# Callback function which is called by pyaudio
#   whenever it needs output-data or has input-data
def audioCallback(in_data, frame_count, time_info, status):
    global AUDIOTIME
    global ClassTones
    global sound_array
    global outbuf

    # Reset arrays
    sound_array.fill(0)
    # outbuf.fill(0)

    print(' ')

    # Timer Profiling of Callback
    since_last = time.time() - AUDIOTIME
    print('Instantaneous Call Rate ', 1/since_last)
    AUDIOTIME = time.time()

    # Check for overruns/underruns
    if status is not 0:
        print('Callback status ', status)


    # Loop through the number of Tones to compute their relevant contribution
    for currentTone in ClassTones:

        # print('PhaseArray ',currentTone.phaseArray)
        # Calculate the value to store as integer array
        sound_array += (32767 * currentTone.amplitude *
                       np.sin(currentTone.phaseArray)).astype(int)

        # Update phase of this tone to compute the next value
        currentTone.updatePhaseArray()

    # Copy the generated sounds into the output buffer array, which stores the
    # values for each channel interleaved LRLRLRLR etc

    # Odd numbers only
    outbuf[::2]  = sound_array[:]

    # Even numbers only
    outbuf[1::2]  = sound_array[:]

    # Output new values of ClassTones to the pipe to be read
    # ros_pipe.send(ClassTones)

    # Convert output buffer to immutable bytes array
    out = bytes(outbuf)

    # More timing calculations
    since_start = time.time() - AUDIOTIME
    print('Time to execute callback ', since_start)
    AUDIOTIME = time.time()

    return (out, pyaudio.paContinue)



#########################
# Start of main program #
#########################

def main():

    # Create single element queue to pass the frequency between the ROS reader
    # process and this process
    shared_fundFreq = mp.Queue(1)
    shared_fundFreq.put(config.INITIAL_FREQUENCY)

    # Start a separate process to read ROS velocity messages
    r = mp.Process(target = ros_interface.ros_start, args=[shared_fundFreq])
    r.start()

    # Update all tones with initial frequency
    for currentTone in ClassTones:
            currentTone.updateFrequency()

    # get a handle to the pyaudio interface
    paHandle = pyaudio.PyAudio()

    audio_devices.showDevices(paHandle)

    # select a device
    audio_devices.outputDevice = audio_devices.setDefaultOutputDevice(paHandle)
    devinfo = paHandle.get_device_info_by_index(audio_devices.outputDevice)
    print("Selected device name: ", devinfo.get('name'))

    support = paHandle.is_format_supported(rate=config.RATE,input_device=None,
                input_format=None,output_device=audio_devices.outputDevice, output_channels=2,
                output_format=paHandle.get_format_from_width(config.WIDTH))

    print('Is format supported: ', support)

    # open a stream with some given properties
    stream = paHandle.open(format=paHandle.get_format_from_width(config.WIDTH),
                           channels=config.CHANNELS,
                           rate=config.RATE,
                           frames_per_buffer=config.FRAMES_PER_BUFFER,
                           input=False,  # no input
                           output=True,  # only output
                           output_device_index=audio_devices.outputDevice,
                           stream_callback=audioCallback)

    stream.start_stream()
    print('Stream Started')

    # Setup count to periodically print stats during main loop
    count = 0


    # Main Loop
    while not rospy.is_shutdown(): #stream.is_active():

        # Periodically check for new speed values from ROS and update the phase arrays
        if shared_fundFreq.full():
            print('Received New Freq')
            # Update the fundamental frequency for all the tones
            Tone.fundFreq=shared_fundFreq.get()

            # Fix all the other frequencies
            for currentTone in ClassTones:
                    currentTone.updateFrequency()
            print('Updated')

        # Every so often print CPU Load
        count += 1
        if count > 500000:
            count = 0

            print('CPU Load ', stream.get_cpu_load())

    stream.stop_stream()
    stream.close()
    print("Stopped")

    paHandle.terminate()


if __name__ == '__main__':
    main()
