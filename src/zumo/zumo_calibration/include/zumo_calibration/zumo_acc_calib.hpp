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
 * Date                 :   March 04, 2024
 * Comments             :
 *
 */

#ifndef ZUMO_ACC_CALIB_H
#define ZUMO_ACC_CALIB_H

#include <Eigen/Geometry>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <chrono>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include <zumo_calibration/definitions.hpp>
#include <zumo_msgs/msg/zumo_sensors.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZumoAccCalib : public rclcpp::Node {
public:
  ZumoAccCalib();
  ~ZumoAccCalib();

  bool Init();
  void KeyboardLoop();

private:
  void ZumoSensorCb(const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg);
  void DataCollection(int iPose);
  void SetMatrices(int iRow, double dAx, double dAy, double dAz, double dAmx,
                   double dAmy, double dAmz);

  void Calibrate();

  rclcpp::Subscription<zumo_msgs::msg::ZumoSensors>::ConstSharedPtr
      p_ZumoSubscription;

  int i_Pose;
  int i_MeasCount;

  Eigen::Matrix<double, MEAS_COUNT * 6, 4> m_MeasurementMatrix;
  Eigen::Matrix<double, MEAS_COUNT * 6, 3> m_AccelerationMatrix;
  Eigen::Matrix<double, 4, 3> m_CalibrationMatrix;

  boost::mutex m_Pose_mutex;

  bool b_DataCollected;

  // serial thread pointer
  boost::thread *p_KeyboardThread;
};

#endif // ZUMO_ACC_CALIB_H
