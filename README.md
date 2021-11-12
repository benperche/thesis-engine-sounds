# Sounds

Python code to generate synthetic engine sounds to be used on an Autonomous Vehicle running ROS

Part of 2021 Undergraduate Thesis, supervised by Stewart Worrall at the University of Sydney

# Installation
Check [dependency_instructions.md](dependency_instructions.md)

# Use
1. Start Dataset Container
In terminal:
```
  xhost +
  docker run --privileged --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --ipc host  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -it -v /Thesis/Data:/data --name dataset-tools acfr/its-dataset-tools
  ```
(Replace /Thesis/Data with location of dataset)
  ```
  roslaunch dataset_playback run.launch
  ```
Select Dataset file in GUI

(Week22 works well)

2. Open RVIZ in new terminal
```
  rviz
  ```
Switch from map frame to base_link (or any other frame)

An rviz setup file is included in this repository, which opens the front camera and LIDAR points only: [rviz_setup.rviz](rviz_setup.rviz)

3. Start Python script in new terminal
```
  python3 pyaudio_callback.py
```
To stop:
    ctrl+c in Python terminal
    Pause/stop dataset in GUI

# Changing the Sounds
The program has been written to facilitate easy changes to the tones used. To change the basic engine tone, you can adjust the initial declaration of tones in the last line of [tone.py](tone.py), with different amplitudes or harmonic ratios.

To change the relationship between the tone and the velocity, adjust line 177 in [pyaudio_callback.py](pyaudio_callback.py).

To change the sound when stopped, adjust the tones added at line 165 in[pyaudio_callback.py](pyaudio_callback.py) (which should then be removed again near line 171.)

To change other properties such as the number of frames per buffer, edit [config.py](config.py)
