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
 *  @file    cleanerBot.hpp
 *  @author  Prasheel Renkuntla
 *  @date    11/18/2019
 *  @version 1.0
 *
 *  @brief header file for the class to be used for autonomous navigation
 *
 *  @section DESCRIPTION
 *  
 *  Class has members that can be used directly into simulated world.
 */
#ifndef INCLUDE_CLEANERBOT_HPP_
#define INCLUDE_CLEANERBOT_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

class CleanerBot {
 public :
  /**
   *   @brief Constructor of class CleanerBot
   *
   *   @param none
   *   @return none
   */
  CleanerBot();

  /**
   *   @brief Destructor of class CleanerBot
   *
   *   @param none
   *   @return none
   */
  ~CleanerBot();

  /**
   *   @brief Callback function for /scan topic messages
   *
   *   @param const sensor_msgs::LaserScan::ConstPtr& msgs, pointer to
   *                  laserscan msgs
   *   @return none
   */
  void scanSensorCallback(
       const sensor_msgs::LaserScan::ConstPtr& msgs);

  /**
   *   @brief Main algorithm for the module
   *
   *   @param none
   *   @return none
   */
  void walkCleaner();

  /**
   *   @brief Getter function to call obstacle flag
   *
   *   @param none
   *   @return bool, should return the obstacle flag
   */
  bool getObstacle();

  /**
   *   @brief Setter function to call obstacle flag
   *
   *   @param bool, should obstacle flag to set the program
   *   @return bool, returns true if flag is set.
   */
  bool setObstacle(bool obs);

 private :

  bool obstacle_;  //  obstacle flag
  geometry_msgs::Twist msgInput_;  //  Input messages to the turtlebot
  ros::NodeHandle nh_;   //  nodehandle object
  ros::Publisher pub_vel_;  //  publisher object
  ros::Subscriber sub_;  //  subscriber
  float lin_vel_x_;  //  linear velocity in the x direction
  float ang_vel_z_;  //  angular velocity in the y direction
};

#endif  //  INCLUDE_CLEANERBOT_HPP_

