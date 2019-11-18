#ifndef INCLUDE_CLEANERBOT_HPP
#define INCLUDE_CLEANERBOT_HPP

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class CleanerBot {
public :
  CleanerBot();
  ~CleanerBot();
  void scanSensorCallback(
       const sensor_msgs::LaserScan::ConstPtr& msgs);
  void walkCleaner();
  bool getObstacle();
  bool setObstacle(bool obs);

private :
  bool obstacle_;
  geometry_msgs::Twist msgInput_;
  ros::NodeHandle nh_;
  ros::Publisher pub_vel_;
  ros::Subscriber sub_;
  float lin_vel_x_;
  float ang_vel_z_;
};

#endif  //  INCLUDE_CLEANERBOT_HPP

