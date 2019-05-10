#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_listener");

  ros::NodeHandle node;

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
        listener.lookupTransform("/base", "/camera_color_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    tf::Matrix3x3 rotation_mtx = transform.getBasis();
    tf::Vector3 translation_mtx = transform.getOrigin();

    std::cout << "-Rotation: {";
    for (int i=0;i<3;i++) {
        tf::Vector3 column_vec = rotation_mtx.getRow(i);
        std::cout << column_vec.getX() << ", " << column_vec.getY() << ", " << column_vec.getZ() << ", ";

    }
    std::cout << "}" << std::endl;

    std::cout << "- Translation: {" << translation_mtx.getX() << ", " << translation_mtx.getY() << ", " << translation_mtx.getZ() << "}" << std::endl;

    rate.sleep();
  }
  return 0;
};
