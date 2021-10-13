#! /usr/bin/env python
#
# Engine Sound Generator
#
# Synthesise engine sound and write to sound hardware using PortAudio/PyAudio
# Callbacks. Get vehicle speed in m/s from shared queue vel_queue, which can
# be set by a ROS listener.
#
# Author: Ben Perche, Faculty of Engineering, University of Sydney

import pyaudio
import numpy as np
import multiprocessing as mp
import time

# ROS
import rospy

import config
import audio_devices
import ros_interface
from tone import Tone, ClassTones

## Global Variables - mostly for use in audio callback

# Keep track of running time for profiling audio callback function
audio_time = time.time()

# Initialise an array to compute the values to be output in each frame
sound_array = np.zeros(config.FRAMES_PER_BUFFER,dtype=np.int16)

# Initialise an output buffer for raw data
outbuf = np.zeros(config.FRAMES_PER_BUFFER*config.CHANNELS,dtype=np.int16)


# Callback function which is called by pyaudio
#   whenever it needs output-data or has input-data
def audioCallback(in_data, frame_count, time_info, status):
    global ClassTones
    global sound_array
    global outbuf
    global audio_time

    # Reset array
    sound_array.fill(0)

    # print(' ')

    # Timer Profiling of Callback
    # since_last = time.time() - audio_time
    # print('Instantaneous Call Rate ', 1/since_last)
    # audio_time = time.time()

    # Check for overruns/underruns
    if status is not 0:
        print('Callback status ', status)


    # Loop through the number of Tones to compute their relevant contribution
    for current_tone in ClassTones:

        # Calculate the value to store as integer array
        sound_array += (32767 * current_tone.amplitude *
                       np.sin(current_tone.phaseArray)).astype(int)

        # Update phase of this tone to compute the next value
        current_tone.updatePhaseArray()

    # Copy the generated sounds into the output buffer array, which stores the
    # values for each channel interleaved LRLRLRLR etc

    # Odd numbers only
    outbuf[::2]  = sound_array[:]

    # Even numbers only
    outbuf[1::2]  = sound_array[:]

    # Convert output buffer to immutable bytes array
    out = bytes(outbuf)

    # More timing calculations
    # since_start = time.time() - audio_time
    # print('Time to execute callback ', since_start)
    # audio_time = time.time()

    return (out, pyaudio.paContinue)



#########################
# Start of main program #
#########################

def main():

    # Create single element queue to pass the velocity between the ROS reader
    # process and this process
    vel_queue = mp.Queue(1)
    vel_queue.put(0)

    # Start a separate process to read ROS velocity messages
    r = mp.Process(target = ros_interface.ros_start, args=[vel_queue])
    r.start()

    # Update all tones with initial frequency
    for current_tone in ClassTones:
            current_tone.updateFrequency()

    # get a handle to the pyaudio interface
    paHandle = pyaudio.PyAudio()

    # audio_devices.showDevices(paHandle)

    # select a device
    audio_devices.output_device = audio_devices.setDefaultOutputDevice(paHandle)
    devinfo = paHandle.get_device_info_by_index(audio_devices.output_device)
    print("Selected device name: ", devinfo.get('name'))

    # support = paHandle.is_format_supported(rate=config.RATE,input_device=None,
    #             input_format=None,output_device=audio_devices.output_device, output_channels=2,
    #             output_format=paHandle.get_format_from_width(config.WIDTH))

    # print('Is format supported: ', support)

    # open a stream with some given properties
    stream = paHandle.open(format=paHandle.get_format_from_width(config.WIDTH),
                           channels=config.CHANNELS,
                           rate=config.RATE,
                           frames_per_buffer=config.FRAMES_PER_BUFFER,
                           input=False,  # no input
                           output=True,  # only output
                           output_device_index=audio_devices.output_device,
                           stream_callback=audioCallback)

    stream.start_stream()
    print('Stream Started')

    # Setup count to periodically print stats during main loop
    count = 0

    # Main Loop
    while not rospy.is_shutdown(): #stream.is_active():

        # Periodically check for new speed values from ROS and update the phase arrays
        if vel_queue.full():

            # Calculate tone frequency based on speed
            new_vel=vel_queue.get()

            # Check for velocity of 0
            if new_vel < 0.01:

                # Set fundamental frequency to set minimum
                Tone.fund_freq = config.MIN_FREQUENCY

                # Add an extra note to the chord if not already present
                if len(ClassTones) < 4:
                    # Reduce the volume/amplitude of the highest tone
                    ClassTones[2].ratio = 0.1

                    # Add a fourth and fifth tone
                    ClassTones.append(Tone(1.77, 0.15))
                    ClassTones.append(Tone(1.33, 0.1))

            else:
                # Remove stationary note if present
                if len(ClassTones) > 3:
                    # Remove last tones
                    ClassTones.pop()
                    ClassTones.pop()

                    # Replace volume of last tone
                    ClassTones[2].ratio = 0.3

                Tone.fund_freq = 25 * new_vel + config.MIN_FREQUENCY

            # Adjust frequencies of tones based on new fund_freq
            for current_tone in ClassTones:
                    current_tone.updateFrequency()


        # Every so often print CPU Load and current fundamental frequency
        count += 1
        if count > 500000:
            count = 0
            print(' ')
            # Last read velocity from ROS
            print(f'Last Vel  = {new_vel:.3f} m/s')
            # Tone fundamental frequency
            print(f'Fund Freq = {Tone.fund_freq:.2f} Hz')
            # PyAudio alleged CPU load
            print(f'CPU Load  = {100*(stream.get_cpu_load()):.2f}%')

    stream.stop_stream()
    stream.close()
    print("Stopped")

    paHandle.terminate()


if __name__ == '__main__':
    main()
