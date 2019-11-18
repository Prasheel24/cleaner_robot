#include <iostream>
#include "cleanerBot.hpp"

CleanerBot::CleanerBot() {
bool obs = false;
setObstacle(obs);
lin_vel_x_ = 0.9;
ang_vel_z_ = 1.0;
msgInput_.linear.x = 0.0;
msgInput_.linear.y = 0.0;
msgInput_.linear.z = 0.0;
msgInput_.angular.x = 0.0;
msgInput_.angular.y = 0.0;
msgInput_.angular.z = 0.0;
}

CleanerBot::~CleanerBot() {
msgInput_.linear.x = 0.0;
msgInput_.linear.y = 0.0;
msgInput_.linear.z = 0.0;
msgInput_.angular.x = 0.0;
msgInput_.angular.y = 0.0;
msgInput_.angular.z = 0.0;
}

bool CleanerBot::setObstacle(bool obs) {
obstacle_ = obs;
return true;
}

bool CleanerBot::getObstacle() {
return obstacle_;
}

ros::Rate getLoopRate() {
return 10;
}

void CleanerBot::scanSensorCallback(
      const sensor_msgs::LaserScan::ConstPtr& msgs) {
  for (std::vector<float>::const_iterator it = msgs->ranges.begin(); 
       it != msgs->ranges.end(); ++it) {
    if (*it <= 0.90) {
      setObstacle(true);
      return;  
    }
  }
  setObstacle(false);
}

void CleanerBot::walkCleaner() {
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>
                ("/cmd_vel_mux/input/navi", 1000);
  sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan",
                 300, &CleanerBot::scanSensorCallback, this);
  ros::Rate loop_rate(10);

  while (ros::ok()) {
  if (getObstacle()) {
    ROS_INFO_STREAM("Obstacle has been detected");    
    msgInput_.linear.x = 0.0;
    msgInput_.angular.z = ang_vel_z_;
  } else {
    ROS_INFO_STREAM("Going ahead");
    msgInput_.angular.z = 0.0;
    msgInput_.linear.x = lin_vel_x_;
  }
  pub_vel_.publish(msgInput_);
  ros::spinOnce();
  loop_rate.sleep();
  }
}
