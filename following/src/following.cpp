#include "trajectoryFollow.h"
#define SUBSCRIBER_BUFFER_SIZE 1   // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1    // Size of buffer for publisher.

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "following");
  ros::NodeHandle n("~");
  FollowingParams params;
  if (!n.getParam("tolerance", params.tolerance))
    params.tolerance = 0.05;
  if (!n.getParam("max_linear_velocity", params.max_linear_velocity))
    params.max_linear_velocity = 0.1;
  if (!n.getParam("max_angular_velocity", params.max_angular_velocity))
    params.max_angular_velocity = 0.5;
  if (!n.getParam("look_ahead", params.look_ahead))
    params.look_ahead = 0.2;
  if (!n.getParam("angle_to_velocity", params.angle_to_velocity))
    params.angle_to_velocity = 2.0;
  if (!n.getParam("skip_sensor_dist", params.skip_sensor_dist))
    params.skip_sensor_dist = 0.06;
  if (!n.getParam("avoid_distance_front", params.avoid_distance_front))
    params.avoid_distance_front = 0.2;
  if (!n.getParam("avoid_distance_side", params.avoid_distance_side))
    params.avoid_distance_side = 0.14;
  if (!n.getParam("approx_iterations", params.approx_iterations))
    params.approx_iterations = 10;
  if (!n.getParam("stop_at_target", params.stop_at_target))
    params.stop_at_target = true;
  if (!n.getParam("path", params.path))
    params.path = "/path";
  if (!n.getParam("scan", params.scan))
    params.scan = "/scan";
  if (!n.getParam("cmd_vel", params.cmd_vel))
    params.cmd_vel = "/cmd_vel";
  if (!n.getParam("path_frame", params.path_frame))
    params.path_frame = "/map";
  if (!n.getParam("robot_frame", params.robot_frame))
    params.robot_frame = "/base_link";
  if (!n.getParam("odom_frame", params.odom_frame))
    params.odom_frame = "/odom";

  ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(params.cmd_vel, PUBLISHER_BUFFER_SIZE);

  TrajectoryFollow *trajectoryFollow = new TrajectoryFollow(pubMessage, params);

  ros::Subscriber sub2 = n.subscribe(params.path, SUBSCRIBER_BUFFER_SIZE, &TrajectoryFollow::trajectoryCallBack, trajectoryFollow);
  ros::Subscriber sub3 = n.subscribe(params.scan, SUBSCRIBER_BUFFER_SIZE, &TrajectoryFollow::laserCallback, trajectoryFollow);

  ros::spin();
  return 0;
}
