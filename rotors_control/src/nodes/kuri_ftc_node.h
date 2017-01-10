/***************************************************************************
 *   Copyright (C) 2016 - 2017 by                                          *
 *      Fatima Al Khoori, KURI  <fatima.alkhoori@kustar.ac.ae>             *
 *                                                                         *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Steet, Fifth Floor, Boston, MA  02111-1307, USA.          *
 ***************************************************************************/

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/CommandAttitudeThrust.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandTrajectory.h>
#include <mav_msgs/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "rotors_control/common.h"
#include "rotors_control/lee_position_controller.h"

namespace rotors_control {

class FTControllerNode {
 public:
  FTControllerNode();
  ~FTControllerNode();

  void InitializeParams();
  void Publish();

 private:

  FTController ftController;

  std::string namespace_;
  ros::Time startTime;
  // subscribers
  ros::Subscriber cmd_trajectory_sub_;
  ros::Subscriber odometry_sub_;

  ros::Publisher motor_velocity_reference_pub_;
  
  void CommandTrajectoryCallback(
      const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg);

  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_NODE_H
