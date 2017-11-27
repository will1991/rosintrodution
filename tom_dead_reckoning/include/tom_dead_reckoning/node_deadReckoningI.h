#ifndef NODE_DEADRECKONINGL_H
#define NODE_DEADRECKONINGL_H

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <string>

using namespace std;

class NodeDeadReckoningI
{
public:

  /* Constructor
   * pub Publisher, which can send commands to robot.
   * fw  Variable, which will be stored in forward.
   * an  Variable, which will be stored in angle.
   * num Variable, which will be stored in numberOfTrips.
   */
  NodeDeadReckoningI(int num, double fw, double an);

  ~NodeDeadReckoningI();

  /* This method receives global odometry data.
   * If #evaluate = false, than this method processes
   * start position.
   * If #evaluate = true, than this method processes
   * end position and start method evaluateResults()
   *
   * msg   Message with global odometry data
   */
  void messageCallback(const nav_msgs::Odometry::ConstPtr& msg);

  /* This method is similar to messageCallback,
   * but receives local odometry data.
   *
   * msg   Message with local odometry data.
   */
  void messageCallback2(const nav_msgs::Odometry::ConstPtr& msg);

  /* This method controls movement of robot (Trips forward
   * and turn around by using PID controller).
   */
  void commander();

  /* Evaluate difference between start and end
   * position in global odometry and compare
   * global with local odometry.
   */
  void evaluateResults();

  //variables
  double forward;     // Distance to go forward
  double angle;       // Angle to turn around
  int numberOfTrips;  // How many times will robot go forward and turn.
  //Initial position (Loc - local odometry)
  geometry_msgs::Point start;
  double startAngle;
  geometry_msgs::Point startLoc;
  double startLocAngle;
  //Final position (Loc - local odometry)
  geometry_msgs::Point end;
  double endAngle;
  geometry_msgs::Point endLoc;
  double endLocAngle;
  bool evaluate; // True if robot finished its trajectory
  bool localOdomReceived;  // True, if local odometry is received
  bool globalOdomReceived; // True, if global odometry is received
};

#endif
