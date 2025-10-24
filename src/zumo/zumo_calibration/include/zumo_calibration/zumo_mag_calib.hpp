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

#ifndef ZUMO_MAG_CALIB_H
#define ZUMO_MAG_CALIB_H

#include <rclcpp/rclcpp.hpp>

#include <zumo_calibration/definitions.hpp>
#include <zumo_msgs/msg/zumo_sensors.hpp>

using namespace std;
using std::placeholders::_1;

class ZumoMagCalib : public rclcpp::Node {
public:
  ZumoMagCalib();
  ~ZumoMagCalib();

  bool Init();

private:
  void ZumoSensorCb(const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg);

  // subscription
  rclcpp::Subscription<zumo_msgs::msg::ZumoSensors>::ConstSharedPtr
      p_ZumoSubscription;

  std::vector<int64_t> running_min_vec;
  std::vector<int64_t> running_max_vec;
};

#endif // ZUMO_ACC_CALIB_H