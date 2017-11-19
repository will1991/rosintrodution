#include "tom_braitenberg/node_braitenberg.h"

NodeBraitenberg2::NodeBraitenberg2():
  nh(ros::NodeHandle()),
  nh_private(ros::NodeHandle("~"))
{
  nh_private.param<std::string>("laserTopic",laserTopicName,"/base_scan");
  nh_private.param<std::string>("pubTopic",pubTopicName,"/cmd_vel");
  nh_private.param<double>("angleCoef",angleCoef,2);
  nh_private.param<double>("robotSpeed",robotSpeed,0.3);

  subScan = nh.subscribe(laserTopicName,1000,&NodeBraitenberg2::messageCallback,this);
  pubMessage = nh.advertise<geometry_msgs::Twist>(pubTopicName,1000);

}
void NodeBraitenberg2::run()
{
  while(ros::ok())
  {
    ros::spinOnce();
  }
}
void NodeBraitenberg2::publisMessage()
{
  geometry_msgs::Twist msg;
  if(distMinLeft >= distMinRight)
  {
    msg.angular.z = angleCoef*distMinLeft/distMinRight - angleCoef;
  }
  else
  {
    msg.angular.z = -(angleCoef*distMinRight/distMinLeft - angleCoef);
  }
  msg.linear.x = robotSpeed;

  if (distMinLeft < 0.25 && distMinRight < 0.25 && angleMinLeft < 0.7 && angleMinRight < 0.7)
    {
      msg.angular.z*=50;
      msg.linear.x*=0.5;
    }

  pubMessage.publish(msg);

}
void NodeBraitenberg2::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  //Calculation of array size from angle range
  int size = msg->ranges.size();
  int minIndexLeft = 0;
  int minIndexRight = size/2;

  // goes throgh  array and find minimum  on the right

  for(int i=0;i<size/2;i++){
    if(msg->ranges[i] < msg->ranges[minIndexRight] && msg->ranges[i] > 0.05)
      minIndexRight = i;
  }

// goes throgh  array and find minimum  on the left

  for(int i=size/2;i<size;i++){
    if(msg->ranges[i] < msg->ranges[minIndexLeft] && msg->ranges[i] > 0.05)
      minIndexLeft = i;
  }

  //Calculaion of angle
  angleMinLeft = (minIndexLeft-size/2)*msg->angle_increment;
  distMinLeft = msg->ranges[minIndexLeft];
  angleMinRight = (minIndexRight-size/2)*msg->angle_increment;
  distMinRight = msg->ranges[minIndexRight];

  publisMessage();
}
