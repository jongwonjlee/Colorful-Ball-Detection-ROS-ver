#ifndef BALL_DETECT_H
#define BALL_DETECT_H


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>

using namespace cv;
using namespace std;

Mat buffer(640,480,CV_8UC3);
ros::Publisher pub_red;
ros::Publisher pub_blue;
ros::Publisher pub_markers;


void ball_detect();
void imageCallback(const sensor_msgs::ImageConstPtr& msg);

/* Declaration of functions that changes data types */
string intToString(int n);
string floatToString(float f);
void morphOps(Mat &thresh); // Declaration of functions that calculates the ball position from pixel position
vector<float> pixel2point(Point2f center, float radius);
string type2str(int type);
void remove_trashval(vector<Point2f> &center, vector<float> &radius, int pixel_radius);

/* trackbar part */
void on_low_h_thresh_trackbar_red(int, void *);
void on_high_h_thresh_trackbar_red(int, void *);
void on_low_h2_thresh_trackbar_red(int, void *);
void on_high_h2_thresh_trackbar_red(int, void *);
void on_low_s_thresh_trackbar_red(int, void *);
void on_high_s_thresh_trackbar_red(int, void *);
void on_low_v_thresh_trackbar_red(int, void *);
void on_high_v_thresh_trackbar_red(int, void *);
int low_h_r=0, high_h_r=6, low_h2_r=167, high_h2_r=180;
int low_s_r=90, low_v_r=102;
int high_s_r=255, high_v_r=255;

void on_low_h_thresh_trackbar_blue(int, void *);
void on_high_h_thresh_trackbar_blue(int, void *);
void on_low_s_thresh_trackbar_blue(int, void *);
void on_high_s_thresh_trackbar_blue(int, void *);
void on_low_v_thresh_trackbar_blue(int, void *);
void on_high_v_thresh_trackbar_blue(int, void *);
int low_h_b=91, low_s_b=247, low_v_b=47;
int high_h_b=119, high_s_b=255, high_v_b=255;

void on_canny_edge_trackbar_red(int, void *);
int lowThreshold_r = 100;
int ratio_r = 3;
int kernel_size_r = 3;

void on_canny_edge_trackbar_blue(int, void *);
int lowThreshold_b = 100;
int ratio_b = 3;
int kernel_size_b = 3;


/* setup default parameters */
float fball_radius = 0.073;

/* Initialization of variable for camera calibration paramters >>change to our own value!!!! */
Mat distCoeffs;
float intrinsic_data[9] = {614.9002685546875, 0.0, 324.05169677734375, 0.0, 615.0999145507812, 236.6910858154297, 0.0, 0.0, 1.0};
float distortion_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

/* Initialization of variable for text drawing */
double fontScale = 2;
int thickness = 3;
String text;
int iMin_tracking_ball_size = 10;


#endif
