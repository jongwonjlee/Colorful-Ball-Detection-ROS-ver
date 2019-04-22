#include "ball_detection/ball_detect.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ball_detect_node"); //init ros nodd
    ros::NodeHandle nh; //create node handler
    image_transport::ImageTransport it(nh); //create image transport and connect it to node handler

    //image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback); //create subscriber and callback
    image_transport::Subscriber sub = it.subscribe("/camera/color/image_raw", 1, imageCallback); //create subscriber and callback
    pub_red = nh.advertise<core_msgs::ball_position>("/position_red", 100);
    pub_blue = nh.advertise<core_msgs::ball_position>("/position_blue", 100);
    pub_markers = nh.advertise<visualization_msgs::Marker>("/balls",1);

    ros::spin(); //spin.

    return 0;
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


void ball_detect(){
    Mat frame, bgr_frame, hsv_frame, hsv_frame_red, hsv_frame_red1, hsv_frame_red2, hsv_frame_blue, hsv_frame_red_blur, hsv_frame_blue_blur, hsv_frame_red_canny, hsv_frame_blue_canny, result; //declare matrix for frames and result
    Mat calibrated_frame;

    //Copy buffer image to frame. If the size of the orignal image(cv::Mat buffer) is 320x240, then resize the image to save (cv::Mat frame) it to 640x480
    if(buffer.size().width==320){
        cv::resize(buffer, frame, cv::Size(640, 480));
    }
    else{
        frame = buffer;
    }

    //declare matrix for calibrated frame
    Mat intrinsic = Mat(3,3, CV_32FC1); //set intrinsic matrix as 3x3 matrix with 32bit float 1 channel
    Mat distCoeffs; //declare matrix distCoeffs
    intrinsic = Mat(3, 3, CV_32F, intrinsic_data); //put intrinsic_data to intrinsic matrix
    distCoeffs = Mat(1, 5, CV_32F, distortion_data); //put distortion_data to disCoeffs matrix
    vector<Vec4i> hierarchy_r; //declare hierachy_r as 4 element vector(line)
    vector<Vec4i> hierarchy_b; //declare hierachy_b as 4 element vector(line)
    vector<vector<Point> > contours_r; //declare contours_r as point vector
    vector<vector<Point> > contours_b; //declare contours_b as point vector

    VideoCapture cap; //get image from video, May be changed to 0 for NUC


    namedWindow("Video capture", WINDOW_NORMAL);
    namedWindow("Detect red in HSV", WINDOW_NORMAL);
    namedWindow("Detect blue in HSV", WINDOW_NORMAL);
    namedWindow("Canny edge - red", WINDOW_NORMAL);
    namedWindow("Canny edge - blue", WINDOW_NORMAL);
    namedWindow("Result", WINDOW_NORMAL);

    /*
    moveWindow("Video capture",50, 0);
    moveWindow("Detect red in HSV", 50,370);
    moveWindow("Detect blue in HSV",470,370);
    moveWindow("Canny edge - red",50,730);
    moveWindow("Canny edge - blue", 470,730);
    moveWindow("Result", 470, 0);
    */

    createTrackbar("Low H","Detect red in HSV", &low_h_r, 180,on_low_h_thresh_trackbar_red);
    createTrackbar("High H","Detect red in HSV", &high_h_r, 180,on_high_h_thresh_trackbar_red);
    createTrackbar("Low H2","Detect red in HSV", &low_h2_r, 180,on_low_h2_thresh_trackbar_red);
    createTrackbar("High H2","Detect red in HSV", &high_h2_r, 180,on_high_h2_thresh_trackbar_red);
    createTrackbar("Low S","Detect red in HSV", &low_s_r, 255,on_low_s_thresh_trackbar_red);
    createTrackbar("High S","Detect red in HSV", &high_s_r, 255,on_high_s_thresh_trackbar_red);
    createTrackbar("Low V","Detect red in HSV", &low_v_r, 255,on_low_v_thresh_trackbar_red);
    createTrackbar("High V","Detect red in HSV", &high_v_r, 255,on_high_v_thresh_trackbar_red);

    createTrackbar("Low H","Detect blue in HSV", &low_h_b, 180,on_low_h_thresh_trackbar_blue);
    createTrackbar("High H","Detect blue in HSV", &high_h_b, 180,on_high_h_thresh_trackbar_blue);
    createTrackbar("Low S","Detect blue in HSV", &low_s_b, 255,on_low_s_thresh_trackbar_blue);
    createTrackbar("High S","Detect blue in HSV", &high_s_b, 255,on_high_s_thresh_trackbar_blue);
    createTrackbar("Low V","Detect blue in HSV", &low_v_b, 255,on_low_v_thresh_trackbar_blue);
    createTrackbar("High V","Detect blue in HSV", &high_v_b, 255,on_high_v_thresh_trackbar_blue);

    createTrackbar("Min Threshold:","Canny edge - red",&lowThreshold_r, 100,on_canny_edge_trackbar_red);
    createTrackbar("Min Threshold:","Canny edge - blue",&lowThreshold_b, 100,on_canny_edge_trackbar_blue);

    //undistort(frame, calibrated_frame, intrinsic, distCoeffs);
    //result = calibrated_frame.clone(); //deep copy calibrated_frame to result

    calibrated_frame = frame.clone();
    result = calibrated_frame.clone();

    /* step 1: blur it */
    medianBlur(calibrated_frame, calibrated_frame, 3);

    /* step 2: convert bgr colorspace to hsv and apply threshold filter */
    cvtColor(calibrated_frame, hsv_frame, cv::COLOR_BGR2HSV);
    // From now on, all matrices are 8UC1 encoded.
    inRange(hsv_frame,Scalar(low_h_r,low_s_r,low_v_r),Scalar(high_h_r,high_s_r,high_v_r),hsv_frame_red1);
    inRange(hsv_frame,Scalar(low_h2_r,low_s_r,low_v_r),Scalar(high_h2_r,high_s_r,high_v_r),hsv_frame_red2);
    inRange(hsv_frame,Scalar(low_h_b,low_s_b,low_v_b),Scalar(high_h_b,high_s_b,high_v_b),hsv_frame_blue);
    addWeighted(hsv_frame_red1, 1.0, hsv_frame_red2, 1.0, 0.0,hsv_frame_red); //merge two frames(hsv_frame_red1, hsv_frame_red2) ratio of 1:1 add scalar 0, output to hsv_frame_red


    /* step 3: apply morphOps function to the colorwise-filtered images */
    //morphOps(hsv_frame_red); //apply function morphOps to hsv_frame_red
    //morphOps(hsv_frame_blue); //apply function morphOps to hsv_frame_blue


    Mat hsv_red_result, hsv_blue_result;
    bitwise_and(calibrated_frame, calibrated_frame, hsv_red_result, hsv_frame_red); // input1, input2, result, mask (hsv_frame is a return of inRange, which prints out binary array
    bitwise_and(calibrated_frame, calibrated_frame, hsv_blue_result, hsv_frame_blue); // input1, input2, result, mask (hsv_frame is a return of inRange, which prints out binary array


    /* step 4: apply gaussian blur to the binary image ;  */
    GaussianBlur(hsv_frame_red, hsv_frame_red_blur, cv::Size(9, 9),2, 2); //gaussian blur; input: hsv_frame_red, output: hsv_frame_red_blur, gaussian kernel
    GaussianBlur(hsv_frame_blue, hsv_frame_blue_blur, cv::Size(9,9), 2, 2); //gaussian blur; input: hsv_blue_red, output: hsv_frame_blue_blur, gaussian kernel

    /* step 5: edge detection */
    Canny(hsv_frame_red_blur, hsv_frame_red_canny, lowThreshold_r,lowThreshold_r*ratio_r, kernel_size_r);
    Canny(hsv_frame_blue_blur, hsv_frame_blue_canny, lowThreshold_b, lowThreshold_b*ratio_b, kernel_size_b);

    /* step 6: Detect contours. Note that each contour is stored as a vector of points (e.g. std::vector<std::vector<cv::Point> >).
    the number of detected contours : n, its number of pixels: p , then the variable 'contours_r's size become n by p. (i.e. contours_r[n-1][p-1])
    */
    findContours(hsv_frame_red_canny, contours_r, hierarchy_r,RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0)); //find contour from hsv_frame_red_canny to contours_r, optional output vector(containing image topology) hierarchy_r, contour retrieval mode: RETER_CCOMP, contour approximationmethod: CHAIN_APPROX_SIMPLE, shift point (0,0)(don't shift the point)
    findContours(hsv_frame_blue_canny, contours_b, hierarchy_b,RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(0, 0));



    /* step 7: With detected contours above, estimate each contour's center and radius. */
    vector<vector<Point> > contours_r_poly( contours_r.size() );
    vector<vector<Point> > contours_b_poly( contours_b.size() );
    vector<Point2f>center_r( contours_r.size() );
    vector<Point2f>center_b( contours_b.size() );
    //set contours_r_poly as a vector of points size of contours_r
    //set contours_b_poly as a vector of points size of contours_b
    //set center_r as point2f vector size of contours_r
    //set center_b as point2f vector size of contours_b

    vector<float>radius_r( contours_r.size() ); //set radius_r as float type vector size of contours_r
    vector<float>radius_b( contours_b.size() ); //set radius_b as float type vector size of contours_b
    for( size_t i = 0; i < contours_r.size(); i++ ){
        approxPolyDP( contours_r[i], contours_r_poly[i], 3, true ); //approximate contours_r[i] to a smoother polygon and put output in contours_r_poly[i] with approximation accuracy 3 (pixelwise distance between the original and approximated point)
        minEnclosingCircle( contours_r_poly[i], center_r[i], radius_r[i] ); //get position of the polygon's center and its radius from minimum enclosing circle from contours_r_poly[i]
    }
    for( size_t i = 0; i < contours_b.size(); i++ ){
        //run the loop while size_t type i from 0 to size of contours_b-1 size by increasing i 1
        approxPolyDP( contours_b[i], contours_b_poly[i], 3, true );
        minEnclosingCircle( contours_b_poly[i], center_b[i], radius_b[i] );
    }

    /*****code should be here!******/

    remove_trashval(center_r, radius_r, iMin_tracking_ball_size);
    remove_trashval(center_b, radius_b, iMin_tracking_ball_size);


    cout << "detected red balls: " << center_r.size() << endl;
    cout << "detected blue balls: " << center_b.size() << endl;


    core_msgs::ball_position msg_r;
    msg_r.size =center_r.size();
    msg_r.img_x.resize(center_r.size());
    msg_r.img_y.resize(center_r.size());
    core_msgs::ball_position msg_b;
    msg_b.size =center_b.size();
    msg_b.img_x.resize(center_b.size());
    msg_b.img_y.resize(center_b.size());

    visualization_msgs::Marker ball_list;  //declare marker
    ball_list.header.frame_id = "/RGBD_camera_color_frame";  //set the frame
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

    double radius = 0.10; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
    ball_list.scale.x=radius;
    ball_list.scale.y=radius;
    ball_list.scale.z=radius;

    /* step 8: draw detected balls */
    for( size_t i = 0; i< center_r.size(); i++ ){
        Scalar color = Scalar(0 , 0, 255); //set scalar color as 255 red

        vector<float> ball_position_r; //declare float vector named ball_position_r
        ball_position_r = pixel2point(center_r[i], radius_r[i]);
        float isx = ball_position_r[0];
        float isy = ball_position_r[1];
        float isz = ball_position_r[2];
        string sx = floatToString(isx);
        string sy = floatToString(isy);
        string sz = floatToString(isz);
        text = "Red ball:" + sx + "," + sy + "," + sz;
        putText(result, text, center_r[i],2,1,color,2);
        circle(result, center_r[i], static_cast<int>(radius_r[i]), color, 3, 8, 0 );

        //msg_r.img_x[i]=isx;
        //msg_r.img_y[i]=isy;

        geometry_msgs::Point p;
        p.x = isx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
        p.y = isy;
        p.z = isz;
        ball_list.points.push_back(p);

        std_msgs::ColorRGBA c;
        c.r = 1.0;  //set the color of the balls. You can set it respectively.
        c.g = 0.0;
        c.b = 0.0;
        c.a = 1.0;
        ball_list.colors.push_back(c);

    }

    for( size_t i = 0; i< center_b.size(); i++ ){ //run the loop while size_t type i from 0 to size of contours_b-1 size by increasing i 1
        Scalar color = Scalar(255, 0, 0); //set scalar color as 255 blue

        vector<float> ball_position_b; //declare float vector named ball_position_b
        ball_position_b = pixel2point(center_b[i], radius_b[i]);
        float isx = ball_position_b[0];
        float isy = ball_position_b[1];
        float isz = ball_position_b[2];
        string sx = floatToString(isx);
        string sy = floatToString(isy);
        string sz = floatToString(isz);
        text = "Blue ball:" + sx + "," + sy + "," + sz; //put message(Blue ball: sx,sy,sz) to text
        putText(result, text, center_b[i],2,1,color,2);
        circle(result, center_b[i], static_cast<int>(radius_b[i]), color, 3, 8, 0 );

        //msg_b.img_x[i]=isx;
        //msg_b.img_y[i]=isy;

        geometry_msgs::Point p;
        p.x = isx;   //p.x, p.y, p.z are the position of the balls. it should be computed with camera's intrinstic parameters
        p.y = isy;
        p.z = isz;
        ball_list.points.push_back(p);

        std_msgs::ColorRGBA c;
        c.r = 0.0;  //set the color of the balls. You can set it respectively.
        c.g = 0.0;
        c.b = 1.0;
        c.a = 1.0;
        ball_list.colors.push_back(c);
    }

    imshow("Video capture",calibrated_frame);
    imshow("Detect red in HSV",hsv_red_result);
    imshow("Detect blue in HSV",hsv_blue_result); //show image hsv_frame_blue on "Object Detection_HSV_Blue"
    imshow("Canny edge - red", hsv_frame_red_canny); //show image hsv_frame_red_canny on "Canny Edge for Red Ball"
    imshow("Canny edge - blue", hsv_frame_blue_canny);
    imshow("Result", result);

    cv::waitKey(1);

    pub_red.publish(msg_r);
    pub_blue.publish(msg_b);

    pub_markers.publish(ball_list);  //publish a marker message


    /*
    Canny(frame,edges,50,200); //proceed edge detection. Whereas frame has 3 channels, edges has 1 channel. (grayscale)


    vector<Vec3f> circles; //assign a memory to save the result of circle detection
    HoughCircles(edges,circles,HOUGH_GRADIENT, 1, 50, 200, 20, 3, 25); //proceed circle detection
    Vec3f params; //assign a memory to save the information of circles
    float cx,cy,r;
    cout<<"circles.size="<<circles.size()<<endl;  //print the number of circles detected
    */
    /*
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

    double radius = 0.10; //set the radius of marker   1.0 means 1.0m, 0.001 means 1mm
    ball_list.scale.x=radius;
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
    */

}


// functions which change data type
string intToString(int n){
    stringstream s;
    s << n;
    return s.str();
}


string floatToString(float f){
    ostringstream buffer;
    buffer << f;
    return buffer.str();
}


vector<float> pixel2point(Point2f center, float radius){
    //get center of ball in 2d image plane and transform it into ball position in 3d camera coordinate
    vector<float> position; //declare float type vetor na med position
    float x, y, u, v, Xc, Yc, Zc;
    //decalre float type values
    x = center.x;//.x;// .at(0);
    y = center.y;//.y;//

    cout << "x: " << x << endl;
    cout << "y: " << y << endl;

    u = (x-intrinsic_data[2])/intrinsic_data[0];
    //calculate x position of image plane
    v = (y-intrinsic_data[5])/intrinsic_data[4];
    //calculate y position of image plane
    /*
    Zc=(intrinsic_data[0]*fball_radius)/(2*(float)radius) ; //compute Z value of the ball center
    Xc=u*Zc ; //calculate real x position from camera coordinate
    Yc=v*Zc ; //calculate real y position from camera coordinate
    */
    Yc= (intrinsic_data[0]*fball_radius)/(2*(float)radius) ; //compute Z value of the ball center
    Xc= u*Yc ; //calculate real x position from camera coordinate
    Zc= - v*Yc ; //calculate real y position from camera coordinate
    Xc=roundf(Xc * 1000) / 1000; //make Xc to 4digit
    Yc=roundf(Yc * 1000) / 1000; //make Yc to 4digit
    Zc=roundf(Zc * 1000) / 1000; //make Zc to 4digit
    position.push_back(Xc); //insert Xc to vector position
    position.push_back(Yc); //insert Yc to vector position
    position.push_back(Zc); //insert Zc to vector position
    return position;
    //return vector position
}


string type2str(int type) {
    /* usage:
    string ty =  type2str( M.type() );
    printf("Matrix: %s %dx%d \n", ty.c_str(), M.cols, M.rows );
    */

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


void morphOps(Mat &thresh){ //dilate and erode image
    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle
    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));
    erode(thresh,thresh,erodeElement);
    //erode(thresh,thresh,erodeElement);
    dilate(thresh,thresh,dilateElement);
    //dilate(thresh,thresh,dilateElement);
}


void remove_trashval(vector<Point2f> &center, vector<float> &radius, int pixel_radius){

    vector<Point2f> temp_center = center;
    vector<float> temp_radius = radius;

    size_t i = 0;
    while (i < temp_radius.size()){ //when there’s some contours in the field of view
        bool something = true;
        //assign dummy Boolean
        for (size_t j = 0; j < temp_radius.size() ; j++){
            if ( (i!=j&&(norm(temp_center[i] - temp_center[j]) < temp_radius[j])) || (temp_radius[i] < pixel_radius) ){
                temp_center.erase(temp_center.begin()+i); //remove ith element from vector
                temp_radius.erase(temp_radius.begin()+i);
                something = false;
                break;
            }
        }
        if(something){
            i++;
        }
    }

    radius = temp_radius;
    center = temp_center;

}


// Trackbar for image threshodling in HSV colorspace : Red
void on_low_h_thresh_trackbar_red(int, void *){
    low_h_r = min(high_h_r-1, low_h_r); //set low_h_r as minimum value of high_h_r-1 and low_h_r
    setTrackbarPos("Low H","Object Detection_HSV_Red", low_h_r);
}
void on_high_h_thresh_trackbar_red(int, void *){
    high_h_r = max(high_h_r, low_h_r+1);
    setTrackbarPos("High H", "Object Detection_HSV_Red", high_h_r);
}
void on_low_h2_thresh_trackbar_red(int, void *){
    low_h2_r = min(high_h2_r-1, low_h2_r);
    setTrackbarPos("Low H2","Object Detection_HSV_Red", low_h2_r);
}
void on_high_h2_thresh_trackbar_red(int, void *){
    high_h2_r = max(high_h2_r, low_h2_r+1);
}
void on_low_s_thresh_trackbar_red(int, void *){
    low_s_r = min(high_s_r-1, low_s_r); //set low_s_r as minimum value of high_s_r-1 and low_s_r
    setTrackbarPos("Low S","Object Detection_HSV_Red", low_s_r);
}
void on_high_s_thresh_trackbar_red(int, void *){
    high_s_r = max(high_s_r, low_s_r+1);
    setTrackbarPos("High S", "Object Detection_HSV_Red", high_s_r); //set trackbar position as high_s_r on trackbar "High S" on window "Object Detection_HSV_Red"
}
void on_low_v_thresh_trackbar_red(int, void *){
    low_v_r= min(high_v_r-1, low_v_r); //set low_v_r as minimum value of high_v_r-1 and low_v_r
    setTrackbarPos("Low V","Object Detection_HSV_Red", low_v_r);
}
void on_high_v_thresh_trackbar_red(int, void *){
    high_v_r = max(high_v_r, low_v_r+1);
}

// Trackbar for image threshodling in HSV colorspace : Blue
void on_low_h_thresh_trackbar_blue(int, void *){
    setTrackbarPos("Low H","Object Detection_HSV_Blue", low_h_b);
}
void on_high_h_thresh_trackbar_blue(int, void *){
    high_h_b = max(high_h_b, low_h_b+1);
    setTrackbarPos("High H", "Object Detection_HSV_Blue", high_h_b);
}
void on_low_s_thresh_trackbar_blue(int, void *){
    low_s_b = min(high_s_b-1, low_s_b); //set low_s_b as minimum value of high_s_b-1 and low_s_b
    setTrackbarPos("Low S","Object Detection_HSV_Blue", low_s_b);
}
void on_high_s_thresh_trackbar_blue(int, void *){
    high_s_b = max(high_s_b, low_s_b+1);
    setTrackbarPos("High S", "Object Detection_HSV_Blue", high_s_b);
}
void on_low_v_thresh_trackbar_blue(int, void *){
    low_v_b= min(high_v_b-1, low_v_b); //set low_v_b as minimum value of high_v_b-1 and low_v_b
    setTrackbarPos("Low V","Object Detection_HSV_Blue", low_v_b);
}
void on_high_v_thresh_trackbar_blue(int, void *){
    high_v_b = max(high_v_b, low_v_b+1);
    setTrackbarPos("High V", "Object Detection_HSV_Blue", high_v_b);
}


// Trackbar for Canny edge algorithm
void on_canny_edge_trackbar_red(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Red Ball",lowThreshold_r); //set trackbar position as lowThreshold_r on trackbar "Min Threshold" on window
}
void on_canny_edge_trackbar_blue(int, void *){
    setTrackbarPos("Min Threshold", "Canny Edge for Blue Ball",lowThreshold_b); //
}
