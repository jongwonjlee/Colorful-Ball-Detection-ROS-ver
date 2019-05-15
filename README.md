# A version for remote control (i.e. cannot monitor on OpenCV window) and printing out balls' positions relative to a robot coordinate.


## To run tf_listener


** Caution: this calibration procedure is essential! **

1. Before get started, please follow the instruction and run a extrinsic calibration package in syntax: [this link](https://github.com/jongwonleeKAIST/tuw_marker_detection)

2. While running the package above, execute a node by typing ```rosrun ball_detection tf_listener```

3. Rotation and translation matrix will be printed out on a terminal. Then, please copy and paste it onto two float type array *rotation[9]* and *translation[3]* at *ball_detect.h* in an include folder.

4. Recompile the whole workspace by commanding ```catkin_make```


## To run a main node (ball_detect)

1. Move to your workspace and clone this repository.

2. ```catkin_make --pkg core_msgs```

3. ```catkin_make```

4. Execute the main node: ```rosrun ball_detection ball_detect``` OpenCV windows will be printed out. (Of course, a camera launch file should be executed first.) Please check balls are detected well. 

5. To change parameters related to the detection (i.e. HSV value...) please run ```rosrun rqt_reconfigure rqt_reconfigure``` and change them in your favor.

