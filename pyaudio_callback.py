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
FRAMESPERBUFFER = 1024

UPPERFREQ = 350
LOWERFREQ = 200
STEP = 0.5

FIRSTHARMONICRATIO = 1.345

INITIALFREQ = 300


sineFrequency = INITIALFREQ

outputDevice = 0

phase = 0  # sine phase
phase2 = 0


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
#
outbuf = array.array('h', range(FRAMESPERBUFFER*CHANNELS))
# outbuf = array.array(range(FRAMESPERBUFFER))


#
# Create the callback function which is called by pyaudio
#   whenever it needs output-data or has input-data
def callback(in_data, frame_count, time_info, status):
    global phase
    global phase2
    global outbuf
    # print(f"Callback, frame_count is {frame_count}")

    # Keep track of samples to store
    s = 0
    # Loop through number of frames to output
    for f in range(frame_count):
        # outbuf[n] = int(32767 * 0.5 * np.sin(phase))
        # outbuf[n] = int(0.5 * np.sin(phase))

        # For each frame store one sample
        # s+=1
        # In each frame put CHANNELS number of 32bit floats.
        for c in range(CHANNELS):
            outbuf[s] = int(32767 * 0.3 * np.sin(phase)
                            + 32767 * 0.2 * np.sin(phase2))

            s += 1

        phase += 2*np.pi*sineFrequency/RATE
        phase2 += 2*np.pi*sineFrequency2/RATE

    # Convert output buffer to immutable bytes array
    out = bytes(outbuf)

    return (out, pyaudio.paContinue)


#########################
# Start of main program #
#########################

def main():
    global sineFrequency
    global sineFrequency2

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
                           frames_per_buffer=FRAMESPERBUFFER,
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
            # print("Waiting")
            count = 0

            # Change direction when out of bounds
            if sineFrequency > UPPERFREQ:
                ascending = False
            elif sineFrequency < LOWERFREQ:
                ascending = True

            if ascending:
                sineFrequency += STEP
            else:
                sineFrequency -= STEP

        sineFrequency2=sineFrequency*FIRSTHARMONICRATIO


    # in this example you'll never get here
    stream.stop_stream()
    stream.close()
    print("Stopped")

    paHandle.terminate()


if __name__ == '__main__':
    main()
