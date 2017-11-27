#include "tom_dead_reckoning/node_deadReckoningI.h"
#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define NUMBER_OF_TRIPS 8 // How many times will robot go forward and turn
#define FORWARD 0.6  // Distance to go forward.
#define ANGLE 1.570796 // Robot will turn around by this angle
#define SUBSCRIBER_TOPIC "/syros/global_odom"
#define SUBSCRIBER2_TOPIC "/syros/base_odom"

int main(int argc, char **argv)
{
  //Initialization of node
  ros::init(argc, argv, "deadReckoningI");
  ros::NodeHandle n;

  //Creating object, which stores data from sensors and has methods for subscribing
  NodeDeadReckoningI *nodeDeadReckoningI = new NodeDeadReckoningI(NUMBER_OF_TRIPS, FORWARD, ANGLE);

  //Creating subscribers
  ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeDeadReckoningI::messageCallback, nodeDeadReckoningI);
  ros::Subscriber sub2 = n.subscribe(SUBSCRIBER2_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeDeadReckoningI::messageCallback2, nodeDeadReckoningI);

  ros::spin();
  return 0;
}
