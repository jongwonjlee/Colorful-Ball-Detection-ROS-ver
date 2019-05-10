#A version for remote control (i.e. cannot monitor on OpenCV window) and printing out balls' positions in an image coordinate

1. Clone this repository.

2. A message package *core_msgs* is included in this repository. If you already have the same package, please remove it.

3. Move to your workspace directory and type ```catkin_make --pkg core_msgs```

4. ```catkin_make```

5. It is done. Just execute the installed package by typing ```rosrun ball_detection ball_detect``` and check whether it works. Of course, a camera lauch file should be executed in advance. Some basic parameters can be adjusted by running ```rosrun rqt_reconfigure rqt_reconfigure```
