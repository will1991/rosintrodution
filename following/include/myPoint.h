#ifndef MYPOINT_H
#define MYPOINT_H

#include <math.h>

class MyPoint
{
public:

  MyPoint(double xPos, double yPos);
  ~MyPoint();

  /* Returns angle between this and target point.
  */
  double getAngle(MyPoint* target);

  /* Returns distance between this and target point.
  */
  double getDistance(MyPoint* target);

  /* Sum of vectors
  */
  MyPoint operator+(const MyPoint &other) const;

  /* Difference of vectors
  */
  MyPoint operator-(const MyPoint &other) const;

  /* Multiplication of vector by real number
  */
  MyPoint times(double r);

  /* Absolute value of vector.
  */
  double getAbs();

//variables
  double x; //!<Position in x.
  double y; //!<Position in y.
};

#endif
