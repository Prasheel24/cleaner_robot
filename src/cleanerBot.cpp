/*****************************************************************************************
 Copyright (C) 2019 Prasheel Renkuntla
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
   * Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
     contributors may be used to endorse or promote products derived from
     this software without specific prior written permission.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
******************************************************************************************/

/**
 *  @copyright MIT License 2019 Prasheel Renkuntla
 *  @file    cleanerBot.cpp
 *  @author  Prasheel Renkuntla
 *  @date    11/18/2019
 *  @version 1.0
 *
 *  @brief Cleaner bot program that runs the simulation
 *
 *  @section DESCRIPTION
 *  
 *  Program for Autonomous navigation of turtlebot in a simulated world
 *  with obstacle avoidance. Here, /scan topic messages are used to detect
 *  and /cmd_vel_mux messages are used to command the turtlebot
 */
#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <cleanerBot.hpp>

/**
 *   @brief Constructor of class CleanerBot
 *
 *   @param none
 *   @return none
 */
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

/**
 *   @brief Destructor of class CleanerBot
 *
 *   @param none
 *   @return none
 */
CleanerBot::~CleanerBot() {
msgInput_.linear.x = 0.0;
msgInput_.linear.y = 0.0;
msgInput_.linear.z = 0.0;
msgInput_.angular.x = 0.0;
msgInput_.angular.y = 0.0;
msgInput_.angular.z = 0.0;
}

/**
   *   @brief Getter function to call obstacle flag
   *
   *   @param none
   *   @return bool, returns the obstacle flag
   */
bool CleanerBot::getObstacle() {
return obstacle_;
}

/**
 *   @brief Setter function to call obstacle flag
 *
 *   @param bool, send obstacle flag to set the program
 *   @return bool, returns true if flag is set.
 */
bool CleanerBot::setObstacle(bool obs) {
obstacle_ = obs;
return true;
}

/**
 *   @brief Callback function for /scan topic messages
 *
 *   @param const sensor_msgs::LaserScan::ConstPtr& msgs, pointer to
 *                  laserscan msgs
 *   @return none
 */
void CleanerBot::scanSensorCallback(
      const sensor_msgs::LaserScan::ConstPtr& msgs) {
//  This is C++ 03 version usage.
//  for (std::vector<float>::const_iterator it = msgs->ranges.begin();
//       it != msgs->ranges.end(); ++it) {
//     if (*it <= 0.90) {
  for (auto it : msgs->ranges) {
    if (it <= 0.90) {
      setObstacle(true);
      return;
    }
  }
  setObstacle(false);
}

/**
 *   @brief Main algorithm for the module
 *
 *   @param none
 *   @return none
 */
void CleanerBot::walkCleaner() {
  //  initialise the publisher navi
  pub_vel_ = nh_.advertise<geometry_msgs::Twist>
                ("/cmd_vel_mux/input/navi", 1000);
  //  initialise the subscriber scan
  sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan",
                 400, &CleanerBot::scanSensorCallback, this);

  ros::Rate loop_rate(10);

  //  loop through until the node shuts down
  //  If there is an obstacle, then rotate
  //  If there is no obstacle, then move ahead.
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
  //  publish the input messages with a loop rate.
  pub_vel_.publish(msgInput_);
  ros::spinOnce();
  loop_rate.sleep();
  }
}
