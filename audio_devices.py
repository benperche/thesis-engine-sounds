
# Global index for output device
output_device = 0

# Function showDevices() lists available input- and output devices
def showDevices(p):
    # Print Defaul output device
    print("Default output device id: ", p.get_default_output_device_info().get(
            'PaHostApiIndex'), " - ", p.get_default_output_device_info().get(
            'name'))

    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')
    for i in range(0, numdevices):
        if p.get_device_info_by_host_api_device_index(0, i).get(
                        'maxInputChannels') > 0:
            print("Input Device id ", i, " - ",
                  p.get_device_info_by_host_api_device_index(0, i).get('name'))
        if p.get_device_info_by_host_api_device_index(0, i).get(
                        'maxOutputChannels') > 0:
            print("Output Device id ", i, " - ",
                  p.get_device_info_by_host_api_device_index(0, i).get('name'))

# Select first reasonable audio device
def setOutputDevice(p):
    global output_device
    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    for i in range(0, numdevices):
        # only pick a device with audio channels
        if p.get_device_info_by_host_api_device_index(0, i).get(
                        'maxOutputChannels') > 0:
            print("checking ",
                  p.get_device_info_by_host_api_device_index(0, i).get('name'))
            output_device = i
            print("Selected device number: ", str(output_device))
            break
    return output_device

# Set output to the system default output device
def setDefaultOutputDevice(p):
    global output_device

    # Capture the name of the default device
    defaultName = p.get_default_output_device_info().get('name')

    info = p.get_host_api_info_by_index(0)
    numdevices = info.get('deviceCount')

    for i in range(0, numdevices):
        # only pick output devices - some devices have both inputs and outputs
        if p.get_device_info_by_host_api_device_index(0, i).get(
                        'maxOutputChannels') > 0:

            print("checking ",
                  p.get_device_info_by_host_api_device_index(0, i).get('name'))

            # Check if this is the default device
            if (p.get_device_info_by_host_api_device_index(0, i).get('name')
                    == defaultName):
                output_device = i
                print("Selected device number: ", str(output_device))
                break
    return output_device
