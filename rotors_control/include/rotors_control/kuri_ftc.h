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

#ifndef ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
#define ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(6, 6, 6);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(4.7, 4.7, 4.7);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class FTControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  LeePositionControllerParameters()
      : position_gain_(kDefaultPositionGain),
        velocity_gain_(kDefaultVelocityGain),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class FTController {
 public:
  FTController();
  ~FTController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) const;

  void SetOdometry(const EigenOdometry& odometry);
  void SetCommandTrajectory(const mav_msgs::EigenCommandTrajectory& command_trajectory);

  LeePositionControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  mav_msgs::EigenCommandTrajectory command_trajectory_;
  EigenOdometry odometry_;

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration, Eigen::Vector3d* angular_acceleration) const;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) const;
};
}

#endif // ROTORS_CONTROL_LEE_POSITION_CONTROLLER_H
