#include "trajectoryFollow.h"
#include <math.h>
#define PI 3.141592

//Constructor and destructor
TrajectoryFollow::TrajectoryFollow(ros::Publisher pub, FollowingParams params) :
  pubMessage(pub), parameters(params)
{
  if (parameters.look_ahead < 2*parameters.tolerance){
    ROS_INFO("look_ahead must be bigger than 2*tolerance");
    exit(1);
  }
  lastRemoved = new MyPoint(0.0, 0.0);
  target = new MyPoint(0.0, 0.0);
  danger_angle = 0;
  danger_distance = 0;
  listener.setExtrapolationLimit(ros::Duration(0.1));
}

TrajectoryFollow::~TrajectoryFollow()
{
}

void TrajectoryFollow::trajectoryCallBack(const nav_msgs::Path::ConstPtr& msg)
{
  while (trajectory.size()>0)
  {
    trajectory.pop();
  }
  // Trajectory must be converted from its frame to odometry frame,
  // otherwise the following might be very shaky and unstable
  // (due jumps in path frame)
  ros::Duration(0.1).sleep();
  tf::StampedTransform transform;
  try{
    //listener.waitForTransform(parameters.path_frame, parameters.robot_frame, ros::Time(0), ros::Duration(3.0) );
    listener.lookupTransform(parameters.path_frame, parameters.odom_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  double phi = 2.0*asin(transform.getRotation().z());
  std::vector<geometry_msgs::PoseStamped> data = msg->poses;
  for(std::vector<geometry_msgs::PoseStamped>::iterator it = data.begin(); it != data.end(); ++it) {
    double x_t = it->pose.position.x - transform.getOrigin().x(); //translation
    double y_t = it->pose.position.y - transform.getOrigin().y(); //translation
    double x_r = x_t*cos(phi) + y_t*sin(phi);
    double y_r = -x_t*sin(phi) + y_t*cos(phi);
    trajectory.push(new MyPoint(x_r,y_r));
  }
}

void TrajectoryFollow::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //Calculation of array size from angle range and angle increment.
  int size = msg->ranges.size();
  //Variables whith index of the lowest value in array.
  int minIndex = 0;
  //This cycle goes through array and finds minimum (too small values are skipped)
  for(int i=minIndex; i<size; i++)
  {
    if (msg->ranges[i] < msg->ranges[minIndex] && msg->ranges[i] > parameters.skip_sensor_dist){
      minIndex = i;
    }
  }
  //Calculation of angles from indexes and storing data to class variables.
  danger_angle = (minIndex-size/2)*msg->angle_increment;
  danger_distance = msg->ranges[minIndex];

  //Transformation
  ros::Duration(0.1).sleep();
  tf::StampedTransform transform;
  try{
    listener.lookupTransform(parameters.odom_frame, parameters.robot_frame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    return;
  }
  robotLocation.position.x = transform.getOrigin().x();
  robotLocation.position.y = transform.getOrigin().y();
  robotLocation.position.z = transform.getOrigin().z();
  robotLocation.orientation.x = transform.getRotation().x();
  robotLocation.orientation.y = transform.getRotation().y();
  robotLocation.orientation.z = transform.getRotation().z();
  robotLocation.orientation.w = transform.getRotation().w();

  /* Start of code, which was in odom callback.
   * Now rewritten for tf.
   */
  double angleCommand = 0;
  double speedCommand = 0;
  MyPoint* actual = new MyPoint(robotLocation.position.x, robotLocation.position.y);

  // If trajectory is still empty and there is nowhere to go, robot should stay, where it is
  if (fabs(lastRemoved->x) < 0.0001 && fabs(lastRemoved->y) < 0.0001 && trajectory.size()>0)
  {
    lastRemoved->x = actual->x;
    lastRemoved->y = actual->y;
    return;
  }
  findTarget(actual);
  if (closeEnough(actual) == true && trajectory.empty())
  {
    publishMessage(0.0,0.0);
    if(parameters.stop_at_target){
      exit(0);
      }
  }
  angleCommand = calculateAngErr(actual, tf::getYaw(robotLocation.orientation));
  if (fabs(angleCommand) > 1.0)
  {
    speedCommand = 0;
  }
  else if (fabs(angleCommand) > 0.5)
  {
    speedCommand = parameters.max_linear_velocity/2;
  }
  else
  {
    speedCommand = parameters.max_linear_velocity;
  }

  //Invoking method for publishing message
  publishMessage(avoidCollisionAngle(angleCommand, speedCommand),avoidCollisionSpeed(angleCommand, speedCommand));
}

//Publisher
void TrajectoryFollow::publishMessage(double angleCommand, double speedCommand)
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.linear.x = speedCommand;
  msg.angular.z = msg.angular.z = copysignf(fmin(parameters.max_angular_velocity, fabs(angleCommand*parameters.angle_to_velocity)), angleCommand);

  //publishing message
  pubMessage.publish(msg);
}

double TrajectoryFollow::calculateAngErr(MyPoint* actual, double angle)
{
  double angleCalc;
  angleCalc = actual->getAngle(target)-angle;
  //normalizing angle to <-pi; pi>
  if (fabs(angleCalc)>PI)
  {
    angleCalc = angleCalc - copysign(2*PI,angleCalc);
  }
  return angleCalc;
}

bool TrajectoryFollow::closeEnough(MyPoint* actual)
{
  double distance;
  distance = actual->getDistance(target);
  if (distance > parameters.tolerance)
  {
    return false;
  }
  return true;
}

void TrajectoryFollow::findTarget(MyPoint* actual)
{
  // Removing points from the queue until point with higher distance
  // from robot than parameters.look_ahead is found.
  if (trajectory.empty() == false)
  {
  while (actual->getDistance(trajectory.front()) < parameters.look_ahead)
  {
    lastRemoved = trajectory.front();
    trajectory.pop();
    if(trajectory.empty() == true){
      break;
    }
  }
  }
  if (trajectory.empty() == true)
  {
    target->x = lastRemoved->x;
    target->y = lastRemoved->y;
  }
  else
  {
    //vector: FRONT - LAST REMOVED
    MyPoint* s = new MyPoint(trajectory.front()->x - lastRemoved->x, trajectory.front()->y - lastRemoved->y);

    //vector which will be added to lastRemoved
    MyPoint* v = new MyPoint(0.0,0.0);
    if (actual->getDistance(lastRemoved) >= parameters.look_ahead && actual->getDistance(trajectory.front()) >= parameters.look_ahead &&
    (lastRemoved->x != trajectory.front()->x || lastRemoved->y != trajectory.front()->y))  //two intersections
    {
      //finding point between intersections
      //calculating line
      double a, b, c, dist;
      a = lastRemoved->y - trajectory.front()->y;
      b = trajectory.front()->x - lastRemoved->x;
      c = lastRemoved->x * trajectory.front()->y - trajectory.front()->x * lastRemoved->y;
      if (a * a + b * b < 0.001)  //front and lastRemoved are the same
      {
        target->x = lastRemoved->x;
        target->y = lastRemoved->y;
        return;
      }
      dist = fabs(a*actual->x + b*actual->y + c)/sqrt(a * a + b * b);  //distance between point and line
      double distFromLast;
      distFromLast = sqrt(pow(lastRemoved->getDistance(actual),2.0) - pow(dist,2.0));
      v->x = (distFromLast/s->getAbs())*s->x;
      v->y = (distFromLast/s->getAbs())*s->y;
      s->x = s->x - v->x;
      s->y = s->y - v->y;
    }

    for (int i = 1; i < parameters.approx_iterations; i++)
    {
      if ((*lastRemoved + s->times(pow(2,-i)) + *v - *actual).getAbs() < parameters.look_ahead)
      {
        *v = *v + s->times(pow(2,-i));
      }

    }
    target->x = lastRemoved->x + v->x;
    target->y = lastRemoved->y + v->y;
  }
}

double TrajectoryFollow::avoidCollisionAngle(double angleCommand, double speedCommand)
{
  if (speedCommand < 0.001)  //It does not move
  {
    return angleCommand;
  }
  else if (danger_distance < parameters.avoid_distance_front && fabs(danger_angle) < PI/6.0)  //Obstacle in front
  {
    return copysign(0.3,-danger_angle);
  }
  else if (danger_distance < parameters.avoid_distance_side && fabs(danger_angle) < 4.0*PI/6.0)  //Obstacle on the side.
  {
    return copysign(0.2,-danger_angle);
  }
  else
  {
    return angleCommand;
  }
}

double TrajectoryFollow::avoidCollisionSpeed(double angleCommand, double speedCommand)
{
  if (speedCommand < 0.001)  //It does not move
  {
    return speedCommand;
  }
  else if (danger_distance < parameters.avoid_distance_front && danger_distance >= parameters.avoid_distance_side && fabs(danger_angle) < PI/6.0)  //Obstacle far, front
  {
    return 0.8*speedCommand;
  }
  else if (fabs(danger_angle) < 4.0*PI/6.0 && danger_distance < parameters.avoid_distance_side)  //Obstacle close
  {
    return 0.6*speedCommand;
  }
  else
  {
    return speedCommand;
  }
}
