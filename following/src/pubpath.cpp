#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"publishpath");
  ros::NodeHandle n;
  ros::Publisher pubPath = n.advertise<nav_msgs::Path>("followingpath",1000);

  ros::Rate loop_rate(1);

  nav_msgs::Path gui_path;
  geometry_msgs::PoseStamped pose;

  std::vector<geometry_msgs::PoseStamped> plan ;

  for (int i=0; i<5; i++){
        pose.pose.position.x = i;
        pose.pose.position.y = -i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
      }
  gui_path.poses.resize(plan.size());

  if(!plan.empty()){
    gui_path.header.frame_id = "/map";
    gui_path.header.stamp = plan[0].header.stamp;
  }

  for(unsigned int i=0; i < plan.size(); i++){
        gui_path.poses[i] = plan[i];
  }

  while (1){
    pubPath.publish(gui_path);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0 ;

}
