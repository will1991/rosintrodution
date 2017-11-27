#ifndef TRAJECTORYFOLLOW_H
#define TRAJECTORYFOLLOW_H

#include "myPoint.h"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Path.h"
#include <queue>
#include "tf/tf.h"
#include "tf/transform_listener.h"

/* Parameters for the following
 *
 * This structure contains parameters obtained from ROS
 * parameter server.
 */
struct FollowingParams{
  double tolerance, max_linear_velocity, max_angular_velocity,
    look_ahead, angle_to_velocity, skip_sensor_dist,
    avoid_distance_front, avoid_distance_side;
  int approx_iterations;
  /*! If this is true, the planning will end when
   *  target is reached. Otherwise it will wait for another one.
   */
  bool stop_at_target;
  std::string path, scan, cmd_vel, path_frame, robot_frame, odom_frame;
};

/* Demonstration task: "Trajectory Following 1"
 * This class controls robot. Robot goes along defined trajectory
 * using "follow the carrot" algorithm.
 */
class TrajectoryFollow
{
public:

    /*
     * pub Publisher, which can send commands to robot.
     * params Structure containing parameters from rosparam server.
     */
    TrajectoryFollow(ros::Publisher pub, FollowingParams params);

    ~TrajectoryFollow();

    /* This method publishes commands for robot.
     *
     * angleCommand Angular velocity.
     * speedCommand Velocity.
     */
    void publishMessage(double angleCommand, double speedCommand);

    /* This method reads data from sensor and searches for obstacles.
     *
     * msg Message, which came from robot and contains data from
     * laser scan.
     */
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);

    /* This method calculates angle error.
     *
     * actual Actual position of robot.
     * angle  Actual orientation of robot.
     */
    double calculateAngErr(MyPoint* actual, double angle);

    /* Check, if robot achieved its destination.
     * If distance between robot and target point is shorter than
     * tolerance, target is accomplished and method returns true.
     *
     * actual Actual position of robot.
     */
    bool closeEnough(MyPoint* actual);

    /* This method finds control target point on trajectory.
     * Target point is on the trajectory in @targetDistance from actual
     * position of robot.
     *
     * actual Actual position of robot.
     */
    void findTarget(MyPoint* actual);

    /* Loads new trajectory from planner.
     *
     * msg Path message
     */
    void trajectoryCallBack(const nav_msgs::Path::ConstPtr& msg);

    /* Simple obstacle avoidance, returns result velocity
     *
     * angleCommand Original command for angular velocity
     * speedCommand Original command for velocity.
     */
    double avoidCollisionSpeed(double angleCommand, double speedCommand);

    /* Simple obstacle avoidance, returns result angle
     *
     * angleCommand Original command for angular velocity
     * speedCommand Original command for velocity.
     */
    double avoidCollisionAngle(double angleCommand, double speedCommand);

    std::queue<MyPoint*> trajectory;    //<Defined trajectory.
    MyPoint* lastRemoved;   //<Last point, removed frome the queue.
    MyPoint* target;        //<Target point, which is actual destination of robot.
    ros::Publisher pubMessage;  //<Object for publishing messages.
    double danger_angle;    //<Angle of point, which is closest to robot.
    double danger_distance; //<Distance of closest point from laser scan.
    tf::TransformListener listener;
    geometry_msgs::Pose robotLocation;
    FollowingParams parameters; // Structure with parameters from rosparam server.
};
#endif
