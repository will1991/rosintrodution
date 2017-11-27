#include "tom_dead_reckoning/node_deadReckoningI.h"
#include <cstdlib>
#define PI 3.141592654

//Constructor and destructor
NodeDeadReckoningI::NodeDeadReckoningI(int num, double fw, double an)
{
  numberOfTrips = num;
  forward = fw;
  angle = an;
}

NodeDeadReckoningI::~NodeDeadReckoningI()
{
}

//Subscriber
void NodeDeadReckoningI::messageCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (globalOdomReceived == false && evaluate == false)
  {
    globalOdomReceived = true;
    start = msg->pose.pose.position;
    startAngle = 2.0*asin(msg->pose.pose.orientation.z);
    if (localOdomReceived == true)
    {
      commander();
    }

  }
  else if (evaluate)
  {
    globalOdomReceived = true;
    end = msg->pose.pose.position;
    endAngle = 2.0*asin(msg->pose.pose.orientation.z);
    if (localOdomReceived == true)
    {
      evaluateResults();
    }
  }
}

//Subscriber
void NodeDeadReckoningI::messageCallback2(const nav_msgs::Odometry::ConstPtr& msg)
{
  if (localOdomReceived == false && evaluate == false)
  {
    localOdomReceived = true;
    startLoc = msg->pose.pose.position;
    startLocAngle = 2.0*asin(msg->pose.pose.orientation.z);
    if (globalOdomReceived == true)
    {
      commander();
    }

  }
  else if (evaluate)
  {
    localOdomReceived = true;
    endLoc = msg->pose.pose.position;
    endLocAngle = 2.0*asin(msg->pose.pose.orientation.z);
    if (globalOdomReceived == true)
    {
      evaluateResults();
    }
  }
}

void NodeDeadReckoningI::commander(){
    stringstream sf;
    sf << "./cmd -f " << forward;
    stringstream sr;
    sr << "./cmd -r " << angle;

    for (int tripsDone = 0; tripsDone < numberOfTrips ; tripsDone++)
    {
      std::system(sf.str().c_str());
      std::system(sr.str().c_str());
    }
    evaluate = true;
    localOdomReceived = false;
    globalOdomReceived = false;
}

void NodeDeadReckoningI::evaluateResults(){
  //evaluating results
    double diffx = end.x - start.x;
    double diffy = end.y - start.y;
    double diffAngle = endAngle - startAngle;

    //TRANSFORMATION
    double angleTransStart = startAngle - startLocAngle;    //How much is local angle different from global
    double xStartLocNew = startLoc.x*cos(angleTransStart)-startLoc.y*sin(angleTransStart);
    double yStartLocNew = startLoc.x*sin(angleTransStart)+startLoc.y*cos(angleTransStart);
    double xEndLocNew = endLoc.x*cos(angleTransStart)-endLoc.y*sin(angleTransStart);
    double yEndLocNew = endLoc.x*sin(angleTransStart)+endLoc.y*cos(angleTransStart);
    double diffxLoc = xEndLocNew  - xStartLocNew ;
    double diffyLoc = yEndLocNew - yStartLocNew ;
    double diffAngleLoc = endLocAngle - startLocAngle;
    while (fabs(diffAngle) > PI)
    {
      diffAngle -= copysign(2*PI,diffAngle);
    }
    //sending message
    ROS_INFO("DIFFERENCE BETWEEN LOCAL AND GLOBAL ODOMETRY:\nDIFFERENCES IN AXIS:\nDIFF_X = %f\nDIFF_Y = %f\nDIFF_ANGLE = %f",diffxLoc-diffx,diffyLoc-diffy,diffAngleLoc-diffAngle);
    ROS_INFO("DIFFERENCE FROM START POSITION:\nDIFERENCES IN AXIS:\nDIFF_X = %f\nDIFF_Y = %f\nDIFF_ANGLE = %f",diffx, diffy, diffAngle);
    exit(0);
}
