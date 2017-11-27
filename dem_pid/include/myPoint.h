#ifndef MYPOINT_H
#define MYPOINT_H

#include <math.h> 
#include "ros/ros.h"

class MyPoint
{
public:

  /* Constructor:
   * xPos   Position in x.
   * yPos   Position in y.
   * alpha  Robot rotation.
   * t      Time of measurement
   */
  MyPoint(double xPos, double yPos, double alpha, ros::Time t);

  ~MyPoint();

  /* Returns angle between this and target point.
   */
  double getAngle(MyPoint* target);

  /* Returns distance between this and target point.
   */
  double getDistance(MyPoint* target);

//variables
  double x;       // Position in x.
  double y;       // Position in y.
  ros::Time time; // Time, when the position was measured.
  double angle;   // Robot's angle.
};

#endif
