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

#ifndef ZUMO_IMU_KF_DEFS_H
#define ZUMO_IMU_KF_DEFS_H

/// accelerometer
#define ACC_CONVERSION_FACTOR                                                  \
  0.613125 // 1g -> 1000mg => 0011 1110 1000 12bits => 0011 1110 1000 0000
           // 16bits => 16000 acceleration data registers actually contain a
           // left-aligned 12-bit number, so the lowest 4 bits are always 0
           // conversion factor: 9.81/16 = 0.613125

// gyroscope
#define GYRO_CONVERSION_FACTOR 8.75

/// other
#define GRAVITATIONAL_ACC 9.81
#define MEAS_COUNT 10

#define INF 999999

#define TAYLOR_NORM_PARAMETER "taylor_normalization"
#define IMU_DEMO_PARAMETER "imu_demo"
#define PRINT_IMU_VALUES_PARAMETER "print_imu_values"
#define DISPLAY_PARAMETERS "display_parameters"
#define TRUE "True"
#define FALSE "False"
#define MAG_MIN_X_PARAMETER "mag_min_x"
#define MAG_MIN_Y_PARAMETER "mag_min_y"
#define MAG_MIN_Z_PARAMETER "mag_min_z"
#define MAG_MAX_X_PARAMETER "mag_max_x"
#define MAG_MAX_Y_PARAMETER "mag_max_y"
#define MAG_MAX_Z_PARAMETER "mag_max_z"
#define PROCESS_X_STD "process_x_std"
#define PROCESS_Y_STD "process_y_std"
#define PROCESS_Z_STD "process_z_std"
#define MEAS_X_STD "meas_x_std"
#define MEAS_Y_STD "meas_y_std"
#define MEAS_Z_STD "meas_z_std"
#define ACC_CALI_11 "acc_cali_11"
#define ACC_CALI_12 "acc_cali_12"
#define ACC_CALI_13 "acc_cali_13"
#define ACC_CALI_21 "acc_cali_21"
#define ACC_CALI_22 "acc_cali_22"
#define ACC_CALI_23 "acc_cali_23"
#define ACC_CALI_31 "acc_cali_31"
#define ACC_CALI_32 "acc_cali_32"
#define ACC_CALI_33 "acc_cali_33"
#define ACC_CALI_41 "acc_cali_41"
#define ACC_CALI_42 "acc_cali_42"
#define ACC_CALI_43 "acc_cali_43"

#endif // ZUMO_IMU_KF_DEFS_H