#include "node_pid.h"
#include <math.h>
#define PI 3.141592
#define F_KP 2.58  // P constant for PSD translation controller
#define F_KD 0.047  // D constant for PSD translation controller
#define F_KI 0.0  // S constant for PSD translation controller
#define R_KP 2.0  // P constant for PSD rotation controller
#define R_KD 0.1  // D constant for PSD rotation controller
#define R_KI 0.0  // S constant for PSD rotation controller

NodePID::NodePID(ros::Publisher pub, double tol, double tolA, double dist, double ang, double mSpeed, double mASpeed)
{
  targetDistance = dist;
  tolerance = tol;
  toleranceAngle = tolA;
  targetAngle = ang;
  maxSpeed = mSpeed;
  maxASpeed = mASpeed;
  pubMessage = pub;
  iterations = 0;
  sumDistance = 0;
  sumAngle = 0;
  start = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());
  last = new MyPoint(0.0, 0.0, 0.0, ros::Time::now());
}

NodePID::~NodePID()
{
}

//Publisher
void NodePID::publishMessage(double angleCommand, double speedCommand)
{
  //preparing message
  geometry_msgs::Twist msg;

  msg.linear.x = speedCommand;
  msg.angular.z = angleCommand;

  //publishing message
  pubMessage.publish(msg);
}

//Subscriber
void NodePID::messageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  double angleCommand = 0;
  double speedCommand = 0;
  MyPoint* actual = new MyPoint(msg->pose.pose.position.x, msg->pose.pose.position.y, 2.0*asin(msg->pose.pose.orientation.z), msg->header.stamp);
  if (closeEnough(actual) == true)
  {
    ROS_INFO("GOAL ACHIEVED");
    publishMessage(0.0,0.0);
    exit(0);
  }
  if (iterations == 0)
  {
    start->x = actual->x;
    start->y = actual->y;
    start->time = actual->time;
    start->angle = actual->angle;
    last->x = actual->x;
    last->y = actual->y;
    last->time = actual->time;
    last->angle = actual->angle;
  }
  iterations++;

  //Calculation of action intervention.
  if (fabs(targetDistance) > tolerance)
  {
    speedCommand = calculatePSD(actual,start->getDistance(actual)*copysign(1.0, targetDistance),start->getDistance(last)*copysign(1.0, targetDistance),targetDistance,F_KP,F_KD,F_KI,&sumDistance);
  }

  if (actual->angle-last->angle < -PI)
  {
    actual->angle += 2*PI;
  }
  else if (actual->angle-last->angle > PI)
  {
    actual->angle -= 2*PI;
  }

  angleCommand = calculatePSD(actual,actual->angle-start->angle, last->angle-start->angle,targetAngle,R_KP,R_KD,R_KI,&sumAngle);

  //Saving position to last
  last->x = actual->x;
  last->y = actual->y;
  last->time = actual->time;
  last->angle = actual->angle;

  //Invoking method for publishing message
  publishMessage(fmin(maxASpeed,angleCommand), fmin(maxSpeed,speedCommand));
}

bool NodePID::closeEnough(MyPoint* actual)
{
  double distance;
  distance = start->getDistance(actual)*copysign(1.0, targetDistance);
  if (fabs(distance-targetDistance) > tolerance)
  {
    return false;
  }
  if (fabs(targetAngle - (actual->angle - start->angle)) > toleranceAngle &
    fabs(targetAngle - (actual->angle - start->angle) + 2*PI) > toleranceAngle &
    fabs(targetAngle - (actual->angle - start->angle) - 2*PI) > toleranceAngle)
  {
    return false;
  }
  return true;
}

double NodePID::calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum)
{
  double speed = 0;
  double error = reference - actualValue;
  double previousError = reference - lastValue;
  double dt = actual->time.toSec() - last->time.toSec();
  double derivative = (error - previousError)/dt;
  *sum = *sum + error*dt;
  speed = kP*error + kD*derivative + kS*(*sum);
  return speed;
}
