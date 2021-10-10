# Sounds

Python code to generate synthetic engine sounds to be used on an Autonomous Vehicle running ROS

Part of 2021 Undergraduate Thesis, supervised by Stewart Worrall at the University of Sydney

INSTALLATION
Check dependency_instructions.md

USE
1. Start Dataset Container
In terminal:
  xhost +
  docker run --privileged --net=host --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --ipc host  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" -it -v /Thesis/Data:/data --name dataset-tools acfr/its-dataset-tools
(Replace /Thesis/Data with location of dataset)
  roslaunch dataset_playback run.launch
Select Dataset file in GUI
(Week22 works well)

2. Open RVIZ in new terminal
  rviz
Switch from map frame to base_link (or any other frame)
Open rviz_setup.rviz file to plot LIDAR points and open camera feed

3. Start Python script in new terminal
  python3 pyaudio_callback.py

To stop:
ctrl+c in Python terminal
Pause/stop dataset in GUI
