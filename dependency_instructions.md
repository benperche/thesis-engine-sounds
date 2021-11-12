# Dependency Installation Instructions

This page will help you install some dependencies required for running the program.

## Ubuntu
The program is designed to run on Ubuntu Linux. It was written and tested on Ubuntu version 18.04.5 LTS.

## ROS
ROS Melodic must be installed for communication between this program and the USyd Campus Dataset.

Follow the steps at http://wiki.ros.org/melodic/Installation/Ubuntu

## USyd Campus Dataset
The USyd Campus Dataset Playback Tools should be installed, and at least one of the dataset rosbag files downloaded. Docker will need to be installed as well to run the playback tools. This step is quite complicated.

For download and setup, follow instructions here: https://gitlab.acfr.usyd.edu.au/its/dataset_metapackage


## Python 3.6.9
The whole program is written in Python 3. The program was written and tested on Python version 3.6.9: https://www.python.org/downloads/release/python-369/

## NumPy
NumPy is used for array operations.
```
pip3 install numpy
  ```

## Portaudio
PortAudio is used to communicate with the sound card. The installation is slightly complicated:

1. Download files: http://files.portaudio.com/download.html

2. In a terminal window, install the ALSA Dev kit:
```
sudo apt-get install libasound2-dev
```

3. In a terminal, run:
```
./configure && make
```
in the location you installed the tgz

More info: http://files.portaudio.com/docs/v19-doxydocs/tutorial_start.html

## PyAudio
PyAudio is the python interface to PortAudio.

Install Pyaudio:  
```
sudo apt-get install python-pyaudio python3-pyaudio
  ```

More info: http://people.csail.mit.edu/hubert/pyaudio/

----------------------------------------------
# Other Setup

Removing some ALSA Warnings:
https://blog.yjl.im/2012/11/pyaudio-portaudio-and-alsa-messages.html

More help: https://askubuntu.com/questions/736238/how-do-i-install-and-setup-the-environment-for-using-portaudio?noredirect=1&lq=1

Fix ALSA Configuration to remove warnings:

Comment out following lines from '/usr/share/alsa/alsa.conf' - use sudo
```
pcm.rear cards.pcm.rear
pcm.center_lfe cards.pcm.center_lfe
pcm.side cards.pcm.side
  ```

ROS Python3 Bridge (from https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674)
Particularly the second line
```
sudo apt-get install python3-pip python3-yaml
sudo pip3 install rospkg catkin_pkg
  ```
