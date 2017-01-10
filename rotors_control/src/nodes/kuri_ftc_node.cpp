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

#include "lee_position_controller_node.h"

#include "rotors_control/parameters_ros.h"


namespace rotors_control 
{

FTControllerNode::FTControllerNode() 
{
  google::InitGoogleLogging("rotors_control_glogger");
  InitializeParams();
  ros::NodeHandle nh;
  cmd_trajectory_sub_ = nh.subscribe(kDefaultCommandTrajectoryTopic, 10,&FTControllerNode::CommandTrajectoryCallback, this);
  odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 10,&FTControllerNode::OdometryCallback, this);
  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::CommandMotorSpeed>(kDefaultMotorSpeedTopic, 10);
  startTime = ros::Time::now();
}

FTControllerNode::~FTControllerNode() { }

void FTControllerNode::InitializeParams() 
{
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());
  GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
}
void FTControllerNode::Publish() {
}

void FTControllerNode::CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_reference_msg) 
{
  mav_msgs::EigenCommandTrajectory trajectory;
  mav_msgs::eigenCommandTrajectoryFromMsg(*trajectory_reference_msg, &trajectory);
  lee_position_controller_.SetCommandTrajectory(trajectory);
}


void FTControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
  ROS_INFO_ONCE("FTControllerNode got first odometry message.");
  
  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::CommandMotorSpeedPtr turning_velocities_msg(new mav_msgs::CommandMotorSpeed);

  turning_velocities_msg->motor_speed.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
  {
    std::cout<<"Motor: "<<i<<" velocity:"<<ref_rotor_velocities[i]<<"\n";
    if (i==0 && (ros::Time::now() - startTime) > ros::Duration(10.0))
    {
        ref_rotor_velocities[i] = 0;
        std::cout<<"I am killing Motor: "<<i<<"\n";
    }
    
    turning_velocities_msg->motor_speed.push_back(ref_rotor_velocities[i]);
  }
  turning_velocities_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(turning_velocities_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ftControllerNode");
  rotors_control::FTControllerNode ftControllerNode;

  ros::spin();

  return 0;
}
