#ifndef NODE_PID_H
#define NODE_PID_H

#include "myPoint.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <queue>

class NodePID
{
public:

  /* Constructor:
   *
   * pub     Publisher, which can send commands to robot.
   * tol     Position tolerance [link](#m).
   * tolA    Angle tolerance [link](#m).
   * dist    Robot will go forward this distance.
   * ang     Robot will turn by this angle [link](#rad).
   * mSpeed  Maximum speed of robot.
   * mASpeed Maximum angular speed of robot.
   */
  NodePID(ros::Publisher pub, double tol, double tolA, double dist, double ang, double mSpeed, double mASpeed);

  ~NodePID();

  /* This method publishes commands for robot.
   *
   * angleCommand   Angular velocity.
   * speedCommand   Velocity.
   */
  void publishMessage(double angleCommand, double speedCommand);

  /* This method reads data from sensor and processes them to variables.
   * It saves actual position, calls methods for evaluating
   * speed and calls method for publishing message.
   *
   * msg   Message, which contains odometry data.
   */
  void messageCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /* This method calculates, if robot is close enough from target point.
   * If distance between robot and target point is shorter than
   * tolerance, target is accomplished and method returns true.
   *
   * actual   Actual position of robot.
   */
  bool closeEnough(MyPoint* actual);

  /* This method calculates action intervention from PSD controller.
   *
   * actual      Actual position of robot.
   * actualValue Actual output value.
   * lastValue   Output value from one step ago.
   * reference   Reference value.
   * kP          P constant for controller PSD.
   * kD          D constant for controller PSD.
   * kS          S constant for controller PSD.
   * sum         sum of errors.
   */
  double calculatePSD(MyPoint* actual, double actualValue, double lastValue, double reference, double kP, double kD, double kS, double *sum);

//variables
  MyPoint *start;     // Start position. Distance will be measured from here
  MyPoint *last;      // Last position of robot.
  double tolerance;   // Tolerated deviation from target distance.
  double maxSpeed;    // Maximal velocity.
  double maxASpeed;   // Maximal angular velocity.
  ros::Publisher pubMessage; // Object for publishing messages.
  double targetDistance; // Robot will go forward this distance.
  double targetAngle;    // Robot will turn by this angle.
  int iterations;        // Number of received messages.
  double sumDistance;    // Sum of distance errors for PSD controller.
  double sumAngle;       // Sum of angle errors for PSD controller.
  double toleranceAngle; // Tolerated deviation from target angle.
};

#endif
