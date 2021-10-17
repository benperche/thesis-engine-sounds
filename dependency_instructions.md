# Dependency Installation Instructions

## Portaudio
Download files: http://files.portaudio.com/download.html
Install ALSA Dev kit:
  sudo apt-get install libasound2-dev
Install: run
  ./configure && make
in the location you installed the tgz
(http://files.portaudio.com/docs/v19-doxydocs/tutorial_start.html)

## PyAudio
Install Pyaudio:  
  sudo apt-get install python-pyaudio python3-pyaudio
(http://people.csail.mit.edu/hubert/pyaudio/)

## Numpy
  pip3 install numpy

----------------------------------------------

Remove ALSA Warnings
https://blog.yjl.im/2012/11/pyaudio-portaudio-and-alsa-messages.html

More help: https://askubuntu.com/questions/736238/how-do-i-install-and-setup-the-environment-for-using-portaudio?noredirect=1&lq=1

Fix ALSA Configuration to remove warnings:

Comment out following lines from '/usr/share/alsa/alsa.conf' - use sudo
  pcm.rear cards.pcm.rear
  pcm.center_lfe cards.pcm.center_lfe
  pcm.side cards.pcm.side

ROS Python3 Bridge (from https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674)
Particularly the second line
  sudo apt-get install python3-pip python3-yaml
  sudo pip3 install rospkg catkin_pkg
