# A version for monitoring whether a camera detects ball or not, and printing out balls' location in an image coordinate.

1. Move to your workspace and clone this repository.

2. ```catkin_make --pkg core_msgs```

3. ```catkin_make```

4. Execute the main node: ```rosrun ball_detection ball_detect``` OpenCV windows will be printed out. (Of course, a camera launch file should be executed first.) Please check balls are detected well. 

5. To change parameters related to the detection (i.e. HSV value...) please run ```rosrun rqt_reconfigure rqt_reconfigure``` and change them in your favor.
