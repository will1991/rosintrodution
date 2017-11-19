#include <ros/ros.h>
#include <tom_braitenberg/node_braitenberg.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "braitenberg");
  ros::NodeHandle nh("~");

  NodeBraitenberg2 test;
  test.run();

  ROS_INFO("Hello world!");
}
