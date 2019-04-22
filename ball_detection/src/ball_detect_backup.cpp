#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "core_msgs/ball_position.h"
#include "opencv2/opencv.hpp"
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

using namespace cv;
using namespace std;

Mat buffer(320,240,CV_8UC3);
ros::Publisher pub;
ros::Publisher pub_markers;




/*Very useful function for cv::Mat type variable!
  The usage looks like this;
     string ty =  type2str( edges.type() );
     ROS_INFO("edges: %s %dx%d \n", ty.c_str(), edges.cols, edges.rows );

*/
string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}



void ball_detect(){
    Mat edges;  //assign a memory to save the edge images
    Mat frame;  //assign a memory to save the images. This variable to be shown below like a 'paper'.

    //Copy buffer image to frame. If the size of the orignal image(cv::Mat buffer) is 320x240, then resize the image to save (cv::Mat frame) it to 640x480
    if(buffer.size().width==320){
        cv::resize(buffer, frame, cv::Size(640, 480));
    }
    else{
        frame = buffer;
    }

    Canny(frame,edges,50,200); //proceed edge detection. Whereas frame has 3 channels, edges has 1 channel. (grayscale)


    vector<Vec3f> circles; //assign a memory to save the result of circle detection
    HoughCircles(edges,circles,HOUGH_GRADIENT, 1, 50, 200, 20, 3, 25); //proceed circle detection
    Vec3f params; //assign a memory to save the information of circles
    float cx,cy,r;
    cout<<"circles.size="<<circles.size()<<endl;  //print the number of circles detected

    core_msgs::ball_position msg;  //create a message for ball positions
    msg.size =circles.size(); //adjust the size of message. (*the size of message is varying depending on how many circles are detected)
    msg.img_x.resize(circles.size());  //adjust the size of array
    msg.img_y.resize(circles.size());  //adjust the size of array

    visualization_msgs::Marker ball_list;  //declare marker
    ball_list.header.frame_id = "/camera_link";  //set the frame
    ball_list.header.stamp = ros::Time::now();   //set the header. without it, the publisher may not publish.
    ball_list.ns = "balls";   //name of markers
    ball_list.action = visualization_msgs::Marker::ADD;
    ball_list.pose.position.x=0; //the transformation between the frame and camera data, just set it (0,0,0,0,0,0) for (x,y,z,roll,pitch,yaw)
    ball_list.pose.position.y=0;
    ball_list.pose.position.z=0;
    ball_list.pose.orientation.x=0;
    ball_list.pose.orientation.y=0;
    ball_list.pose.orientation.z=0;
    ball_list.pose.orientation.w=1.0;

    ball_list.id = 0; //set the marker id. if you use another markers, then make them use their own unique ids
    ball_list.type = visualization_msgs::Marker::SPHERE_LIST;  //set the type of marker

    double radius = 0.10;
    ball_list.scale.x=radius; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
    ball_list.scale.y=radius;
    ball_list.scale.z=radius;

    for(int k=0;k<circles.size();k++){
        params = circles[k];  //the information of k-th circle
        cx=cvRound(params[0]);  //x position of k-th circle
        cy=cvRound(params[1]);  //y position
        r=cvRound(params[2]); //radius
        // 원 출력을 위한 원 중심 생성
        Point center(cx,cy);  //declare a Point
        circle(frame,center,r,Scalar(0,0,255),5); //draw a circle on 'frame' based on the information given,   r = radius, Scalar(0,0,255) means color, 10 means lineWidth

        cy = 3.839*(exp(-0.03284*cy))+1.245*(exp(-0.00554*cy));   //convert the position of the ball in camera coordinate to the position in base coordinate. It is related to the calibration process. You shoould modify this.
        cx = (0.002667*cy+0.0003)*cx-(0.9275*cy+0.114);

        msg.img_x[k]=cx;  //input the x position of the ball to the message
        msg.img_y[k]=cy;

        geometry_msgs::Point p;
        p.x = cx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
        p.y = cy;
        p.z = 0.1;
        ball_list.points.push_back(p);

        std_msgs::ColorRGBA c;
        c.r = 0.0;  //set the color of the balls. You can set it respectively.
        c.g = 1.0;
        c.b = 0.0;
        c.a = 1.0;
        ball_list.colors.push_back(c);
    }
    cv::imshow("view", frame);  //show the image with a window
    cv::waitKey(1);
    pub.publish(msg);  //publish a message
    pub_markers.publish(ball_list);  //publish a marker message

}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if(msg->height==480&&buffer.size().width==320){  //check the size of the image received. if the image have 640x480, then change the buffer size to 640x480.
        std::cout<<"resized"<<std::endl;
        cv::resize(buffer,buffer,cv::Size(640,480));
    }

    else{
        //do nothing!
   }

    try
    {
        buffer = cv_bridge::toCvShare(msg, "bgr8")->image;  //transfer the image data into buffer
    }
    catch (cv_bridge::Exception& e)
    {
         ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }

    ball_detect(); //proceed ball detection
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detect_node"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    image_transport::ImageTransport it(nh); //create image transport and connect it to node handler

    //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //create subscriber and callback
    image_transport::Subscriber sub = it.subscribe("/RGBD_camera/color/image_raw", 1, imageCallback); //create subscriber and callback
    pub = nh.advertise<core_msgs::ball_position>("/position", 100); //setting publisher
    pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

    ros::spin(); //spin.

    return 0;
}
