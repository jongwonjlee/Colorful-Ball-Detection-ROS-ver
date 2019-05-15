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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <dynamic_reconfigure/server.h>
#include "ball_detection/BallDetectionConfig.h"


#include <opencv2/imgproc.hpp>
#include <iostream>
#include <string>
#include <stdlib.h>
#include <math.h>


struct balls_info
{
    int num_r;
    int num_b;
    std::vector<cv::Point2i>center_r;
    std::vector<cv::Point2i>center_b;
    std::vector<int>radius_r;
    std::vector<int>radius_b;
    std::vector<short int>distance_r;
    std::vector<short int>distance_b;

};


class BallDetectNode
{
public:
    BallDetectNode();    

private:
    cv::Mat buffer_color;
    cv::Mat buffer_depth;
    ros::Publisher pub;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private_;

    ball_detection::BallDetectionConfig config_;

    const int lowThreshold_r = 100;
    const int ratio_r = 3;
    const int kernel_size_r = 3;

    const int lowThreshold_b = 100;
    const int ratio_b = 3;
    const int kernel_size_b = 3;

    /* setup default parameters */
    const float fball_radius = 0.073/2;

    /* Initialization of variable for camera calibration paramters >>change to our own value!!!! */
    cv::Mat distCoeffs;
    float intrinsic_data[9] = {614.9002685546875, 0.0, 324.05169677734375, 0.0, 615.0999145507812, 236.6910858154297, 0.0, 0.0, 1.0};
    float distortion_data[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

    /*** EXTRINSIC PARAMETERS SHOULD BE HERE!!! ***/
    float rotation[9] = {0.99948, -0.0278864, 0.0161993, -0.0258717, -0.393425, 0.918993, -0.0192542, -0.918934, -0.393942};
    float translation[3] = {-0.0711494, -0.0441139, 0.338856};

    /* Initialization of variable for text drawing */
    const double fontScale = 2;
    const int thickness = 3;
    const int iMin_tracking_ball_size = 10;


    dynamic_reconfigure::Server<ball_detection::BallDetectionConfig>* reconfigureServer_; ///< parameter server stuff
    dynamic_reconfigure::Server<ball_detection::BallDetectionConfig>::CallbackType reconfigureFnc_;///< parameter server stuff

    void callbackConfig (ball_detection::BallDetectionConfig &_config); ///< callback function on incoming parameter changes
    void imageCallback(const sensor_msgs::ImageConstPtr& msg_color, const sensor_msgs::ImageConstPtr& msg_depth);
    balls_info ball_detect();
    void pub_msgs(balls_info &ball_information);

    std::vector<float> pixel2point_depth(cv::Point2i pixel_center, int distance);
    std::vector<float> transform_coordinate( std::vector<float> input );
    void morphOps(cv::Mat &thresh);
    void remove_trashval(std::vector<cv::Point2f> &center, std::vector<float> &radius, int pixel_radius);
    short int calibrate_rangeinfo(short x);
    short int lookup_range(int y, int x, cv::Mat& range_frame);

};


/* Declaration of functions that changes data types */
std::string intToString(int n);
std::string floatToString(float f);
std::string type2str(int type);

#endif
