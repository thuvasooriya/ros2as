/**
 * Copyright (c) 2024 ENTC, University of Moratuwa
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of UoM nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 * Original AUTHOR(s)   :   Peshala J
 * Modified By          :
 * Date                 :   March 10, 2024
 * Comments             :
 *
 */

#ifndef ZUMO_IMU_KF_H
#define ZUMO_IMU_KF_H

#include <Eigen/Geometry>
#include <geometry_msgs/msg/transform_stamped.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <zumo_defs/zumo_definitions.hpp>
#include <zumo_imu_kf/definitions.hpp>
#include <zumo_msgs/msg/zumo_sensors.hpp>

using std::placeholders::_1;

class ZumoImuKF : public rclcpp::Node {
public:
  ZumoImuKF();
  ~ZumoImuKF();

  bool Init();

private:
  void ZumoSensorCb(const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg);
  void KalmanPredict();
  void KalmanUpdate();
  void Normalize();
  void
  TransformAccRawData(const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg);
  void
  TransformGyroRawData(const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg);
  void
  TransformMagRawData(const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg);
  void UpdateMeasRotMatrix();
  void PublishTF();

  // subscription
  rclcpp::Subscription<zumo_msgs::msg::ZumoSensors>::ConstSharedPtr
      p_ZumoSubscription;

  Eigen::Matrix<double, 3, 1> m_GyroBody_vec, m_GyroEarth_vec;
  Eigen::Matrix<double, 3, 1> m_Acc_vec;
  Eigen::Matrix<double, 3, 1> m_Mag_vec;
  Eigen::Matrix<double, 3, 1>
      m_MagMax_vec; // maximum magnetometer values, used for calibration
  Eigen::Matrix<double, 3, 1>
      m_MagMin_vec; // minimum magnetometer values, used for calibration

  // Kalman Filter
  double d_ProcX_std, d_ProcY_std, d_ProcZ_std;
  double d_MeasX_std, d_MeasY_std, d_MeasZ_std;
  Eigen::Matrix<double, 3, 1> m_StateX_vec, m_StateY_vec, m_StateZ_vec;
  Eigen::Matrix<double, 3, 3> m_StateXCov_matrix, m_StateYCov_matrix,
      m_StateZCov_matrix;
  Eigen::Matrix<double, 3, 3> m_StateTransit_matrix;
  Eigen::Matrix<double, 3, 3> m_ProcessNoiseCov_matrix;
  Eigen::Matrix<double, 3, 3> m_MeasNoiseCov_matrix;
  Eigen::Matrix<double, 3, 3> m_DCM_matrix;
  Eigen::Matrix<double, 3, 3> m_Obs_matrix;
  Eigen::Matrix<double, 3, 3> m_MeasRot_matrix;
  Eigen::Matrix<double, 3, 3> m_Identity_matrix;

  std::unique_ptr<tf2_ros::TransformBroadcaster> p_TfBroadcaster;

  // calibration
  Eigen::Matrix<double, 4, 3> m_AccCalib_matrix; // accelerometer

  rclcpp::Time m_PrevSensorStamp;

  // bool b_DriftCorrection;
  bool b_TaylorNormalization;
  bool b_SensorInit;
  bool b_IMUDemo;
  bool b_PrintValues;
  bool b_DisplayParams;

  double dt;
  double d_Roll, d_Pitch, d_Yaw;
};

#endif // ZUMO_IMU_KF_H