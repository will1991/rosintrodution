#ifndef NODEBRAITENBERG2_H
#define NODEBRAITENBERG2_H
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

class NodeBraitenberg2
{
public:
  NodeBraitenberg2();
  ~NodeBraitenberg2(){}
  /*the main part for run the program
   *
   * excacute publishMessage()to publsih twits message to robot
   * while(ros::ok) contiue run the process
   */

  void run();

private:
  /* This method publishes commands for robot.
   *
   * Commands are generated from data, which are stored in variables
   * (distMinLeft, distMinRight). Robot turns to direction, which has higher
   * value. Robot turns sharper, if higher value >> lower value.
   */
  void publisMessage();

  /* This method reads data from sensor and processes them to variables.
    *
    * This method finds minimal distances on the left and right side
    * and saves them to variables distMinLeft, distMinRight.
    *
    * @param msg Message, which came from robot and contains data from
    * laser scan.
    */
  void messageCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

private:
  ros::NodeHandle nh;

  ros::NodeHandle nh_private;

  std::string pubTopicName;
  std::string laserTopicName;

  ros::Subscriber subScan;
  ros::Publisher pubMessage;


  double angleCoef;       // Coeficient for transfering angles to speed.
  double robotSpeed;        // Speed of robot [m/s].
  double angleMinLeft;       // Angle, at which was measured the shortest distance on the left.
  double distMinLeft;        // Minimum distance masured by sensor on the left.
  double angleMinRight;      // Angle, at which was measured the shortest distance on the right.
  double distMinRight;       // Minimum distance masured by sensor on the right.

};

#endif // NODEBRAITENBERG2_H
