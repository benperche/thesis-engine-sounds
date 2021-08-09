#
# Basic pyaudio program playing a real time mono sine wave
#
# (ME) 2015 Marc Groenewegen
# https://www.dinkum.nl/software/python/audio/basic_sine.py
#

import pyaudio
# import time
import numpy as np
import array

WIDTH = 2  # sample size in bytes
CHANNELS = 2  # number of samples in a frame
RATE = 44100
FRAMES_PER_BUFFER = 1024

# Temporary sweep parameters
UPPER_FREQ = 350
LOWER_FREQ = 200
STEP = 0.5

# Set up frequency relationships between sine waves
# Each element of this array will represent a sine wave, with the value
# to represent the ratio to the fundamental
# i.e. [1, 2] would create an octave, 2:1 ratio between intervals
Tones = [1, 1.25, 1.5]

# Store the relative amplitudes of each sine wave
Amplitudes = [0.3, 0.2, 0.1]

# Store the current posiiton in each sine wave in a Phase array
# Also store the current frequencies of this number of sine waves
# Create array to be the same size as the tones
Phases = [0] * len(Tones)
Frequencies = [300] * len(Tones)


class Tone:
    # Constructor
    def __init__(self, ratio, amplitude):
        self.ratio = ratio
        self.amplitude = amplitude

    phase = 0
    frequency = 300


ClassTones = [Tone(1, 0.3), Tone(1.5, 0.2)]

outputDevice = 0


#
# Function showDevices() lists available input- and output devices
#
def showDevices(p):
    # Print Defaul output device
    print("Default output device id: ", p.get_default_output_device_info().get(
            'PaHostApiIndex'), " - ", p.get_default_output_device_info().get('name'))

    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    for i in range(0, numdevices):
        if p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels') > 0:
            print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))
        if p.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels') > 0:
            print("Output Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))


# Select first reasonable audio device
def setOutputDevice(p):
    global outputDevice
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    for i in range(0, numdevices):
        # only pick a device with audio channels
        if p.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels') > 0:
            print("checking ", p.get_device_info_by_host_api_device_index(0, i).get('name'))
            # if p.get_device_info_by_host_api_device_index(0, i).get('name').find("pulse") >= 0:
            outputDevice = i
            print("Selected device number: ", str(outputDevice))
            break


def setDefaultOutputDevice(p):
    global outputDevice

    # Capture the name of the default device
    defaultName = p.get_default_output_device_info().get('name')

    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    for i in range(0, numdevices):
        # only pick output devices - some devices have both inputs and outputs
        if p.get_device_info_by_host_api_device_index(0, i).get('maxOutputChannels') > 0:

            print("checking ", p.get_device_info_by_host_api_device_index(0, i).get('name'))

            # Check if this is the default device
            if (p.get_device_info_by_host_api_device_index(0, i).get('name')
                    == defaultName):
                outputDevice = i
                print("Selected device number: ", str(outputDevice))
                break


#
# Create array of signed ints to hold one sample buffer
# Make it global so it doesn't get re-allocated for every frame
outbuf = array.array('h', range(FRAMES_PER_BUFFER*CHANNELS))


# Callback function which is called by pyaudio
#   whenever it needs output-data or has input-data
def callback(in_data, frame_count, time_info, status):
    global phase
    global phase2
    global outbuf

    # Keep track of samples to store
    s = 0

    # Loop through number of frames to output
    for f in range(frame_count):

        # For each frame store one sample

        # In each frame put CHANNELS number of 32bit floats.
        for c in range(CHANNELS):

            outbuf[s] = 0
            # Loop through the number of sine waves in Tones
            for t, currentTone in enumerate(ClassTones):
                # print(range(len(Tones)-1))

                outbuf[s] = outbuf[s] + int(32767 * currentTone.amplitude * np.sin(currentTone.phase))

                # Update phase of this tone
                currentTone.phase += 2*np.pi*currentTone.frequency/RATE

                # Move to next sample to store
            s += 1
        # Update phase of both sine waves
        # phase += 2*np.pi*sineFrequency/RATE
        # phase2 += 2*np.pi*sineFrequency2/RATE

    # Convert output buffer to immutable bytes array
    out = bytes(outbuf)

    return (out, pyaudio.paContinue)


#########################
# Start of main program #
#########################

def main():
    # global sineFrequency
    # global sineFrequency2

    # print(ClassTones[0].ratio)
    #
    # get a handle to the pyaudio interface
    #
    paHandle = pyaudio.PyAudio()

    # select a device
    setDefaultOutputDevice(paHandle)
    devinfo = paHandle.get_device_info_by_index(outputDevice)
    print("Selected device name: ", devinfo.get('name'))

    # Set nunber of channels
    CHANNELS = devinfo.get('maxOutputChannels')

    # open a stream with some given properties
    stream = paHandle.open(format=paHandle.get_format_from_width(WIDTH),
                           channels=CHANNELS,
                           rate=RATE,
                           frames_per_buffer=FRAMES_PER_BUFFER,
                           input=False,  # no input
                           output=True,  # only output
                           output_device_index=outputDevice,  # choose outputdevice
                           stream_callback=callback)

    stream.start_stream()

    count = 0
    ascending = True
    # Make sure that the main program doesn't finish until all
    #  audio processing is done
    while stream.is_active():

        # Gradually change direction of sine wave

        count += 1
        if count > 10000:
            count = 0

            # Change direction when out of bounds
            if ClassTones[0].frequency > UPPER_FREQ:
                ascending = False
            elif ClassTones[0].frequency < LOWER_FREQ:
                ascending = True

            if ascending:
                ClassTones[0].frequency += STEP
            else:
                ClassTones[0].frequency -= STEP

            for t, val in enumerate(ClassTones):
                val.frequency = ClassTones[0].frequency * val.ratio
                # print('Frequencies ', t, ' = ', Frequencies[t])

    stream.stop_stream()
    stream.close()
    print("Stopped")

    paHandle.terminate()


if __name__ == '__main__':
    main()
