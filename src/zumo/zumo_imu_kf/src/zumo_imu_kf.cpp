#include <zumo_imu_kf/zumo_imu_kf.hpp>

///*********************************************************************************
/// Constructor
///*********************************************************************************
ZumoImuKF::ZumoImuKF() : rclcpp::Node("zumo_imu_kf") {}

///*********************************************************************************
/// Deconstructor
///*********************************************************************************
ZumoImuKF::~ZumoImuKF() {}

///*********************************************************************************
/// Initialization
///*********************************************************************************
bool ZumoImuKF::Init() {
  RCLCPP_INFO(this->get_logger(), "ZumoImuKF initializing ...");

  p_ZumoSubscription = this->create_subscription<zumo_msgs::msg::ZumoSensors>(
      "/zumo/zumo_sensors", 100, std::bind(&ZumoImuKF::ZumoSensorCb, this, _1));
  p_TfBroadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Get parameters
  int iMagMinX, iMagMinY, iMagMinZ;
  int iMagMaxX, iMagMaxY, iMagMaxZ;

  // Initialize the parameters
  this->declare_parameter(TAYLOR_NORM_PARAMETER, false);
  this->declare_parameter(IMU_DEMO_PARAMETER, true);
  this->declare_parameter(PRINT_IMU_VALUES_PARAMETER, true);
  this->declare_parameter(DISPLAY_PARAMETERS, false);

  this->declare_parameter(MAG_MIN_X_PARAMETER, 0);
  this->declare_parameter(MAG_MIN_Y_PARAMETER, 0);
  this->declare_parameter(MAG_MIN_Z_PARAMETER, 0);
  this->declare_parameter(MAG_MAX_X_PARAMETER, 0);
  this->declare_parameter(MAG_MAX_Y_PARAMETER, 0);
  this->declare_parameter(MAG_MAX_Z_PARAMETER, 0);

  this->declare_parameter(PROCESS_X_STD, 0.01);
  this->declare_parameter(PROCESS_Y_STD, 0.01);
  this->declare_parameter(PROCESS_Z_STD, 0.01);

  this->declare_parameter(MEAS_X_STD, 0.01);
  this->declare_parameter(MEAS_Y_STD, 0.01);
  this->declare_parameter(MEAS_Z_STD, 0.01);

  this->declare_parameter(ACC_CALI_11, 0.0);
  this->declare_parameter(ACC_CALI_12, 0.0);
  this->declare_parameter(ACC_CALI_13, 0.0);
  this->declare_parameter(ACC_CALI_21, 0.0);
  this->declare_parameter(ACC_CALI_22, 0.0);
  this->declare_parameter(ACC_CALI_23, 0.0);
  this->declare_parameter(ACC_CALI_31, 0.0);
  this->declare_parameter(ACC_CALI_32, 0.0);
  this->declare_parameter(ACC_CALI_33, 0.0);
  this->declare_parameter(ACC_CALI_41, 0.0);
  this->declare_parameter(ACC_CALI_42, 0.0);
  this->declare_parameter(ACC_CALI_43, 0.0);

  // Get the parameter value
  this->get_parameter(TAYLOR_NORM_PARAMETER, b_TaylorNormalization);
  this->get_parameter(IMU_DEMO_PARAMETER, b_IMUDemo);
  this->get_parameter(PRINT_IMU_VALUES_PARAMETER, b_PrintValues);
  this->get_parameter(DISPLAY_PARAMETERS, b_DisplayParams);

  this->get_parameter(MAG_MIN_X_PARAMETER, iMagMinX);
  this->get_parameter(MAG_MIN_Y_PARAMETER, iMagMinY);
  this->get_parameter(MAG_MIN_Z_PARAMETER, iMagMinZ);
  this->get_parameter(MAG_MAX_X_PARAMETER, iMagMaxX);
  this->get_parameter(MAG_MAX_Y_PARAMETER, iMagMaxY);
  this->get_parameter(MAG_MAX_Z_PARAMETER, iMagMaxZ);

  this->get_parameter(PROCESS_X_STD, d_ProcX_std);
  this->get_parameter(PROCESS_Y_STD, d_ProcY_std);
  this->get_parameter(PROCESS_Z_STD, d_ProcZ_std);

  this->get_parameter(MEAS_X_STD, d_MeasX_std);
  this->get_parameter(MEAS_Y_STD, d_MeasY_std);
  this->get_parameter(MEAS_Z_STD, d_MeasZ_std);

  // accelerometer calibration
  double cali11, cali12, cali13;
  double cali21, cali22, cali23;
  double cali31, cali32, cali33;
  double cali41, cali42, cali43;

  this->get_parameter(ACC_CALI_11, cali11);
  this->get_parameter(ACC_CALI_12, cali12);
  this->get_parameter(ACC_CALI_13, cali13);
  this->get_parameter(ACC_CALI_21, cali21);
  this->get_parameter(ACC_CALI_22, cali22);
  this->get_parameter(ACC_CALI_23, cali23);
  this->get_parameter(ACC_CALI_31, cali31);
  this->get_parameter(ACC_CALI_32, cali32);
  this->get_parameter(ACC_CALI_33, cali33);
  this->get_parameter(ACC_CALI_41, cali41);
  this->get_parameter(ACC_CALI_42, cali42);
  this->get_parameter(ACC_CALI_43, cali43);

  // Display parameters
  if (true == b_DisplayParams) {
    RCLCPP_INFO(this->get_logger(), "%s: %s", TAYLOR_NORM_PARAMETER,
                b_TaylorNormalization ? TRUE : FALSE);
    RCLCPP_INFO(this->get_logger(), "%s: %s", IMU_DEMO_PARAMETER,
                b_IMUDemo ? TRUE : FALSE);
    RCLCPP_INFO(this->get_logger(), "%s: %s", PRINT_IMU_VALUES_PARAMETER,
                b_PrintValues ? TRUE : FALSE);
  }

  m_MagMin_vec[0] = iMagMinX;
  m_MagMin_vec[1] = iMagMinY;
  m_MagMin_vec[2] = iMagMinZ;
  m_MagMax_vec[0] = iMagMaxX;
  m_MagMax_vec[1] = iMagMaxY;
  m_MagMax_vec[2] = iMagMaxZ;

  // accelerometer calibration matrix
  m_AccCalib_matrix(0, 0) = cali11;
  m_AccCalib_matrix(0, 1) = cali12;
  m_AccCalib_matrix(0, 2) = cali13;
  m_AccCalib_matrix(1, 0) = cali21;
  m_AccCalib_matrix(1, 1) = cali22;
  m_AccCalib_matrix(1, 2) = cali23;
  m_AccCalib_matrix(2, 0) = cali31;
  m_AccCalib_matrix(2, 1) = cali32;
  m_AccCalib_matrix(2, 2) = cali33;
  m_AccCalib_matrix(3, 0) = cali41;
  m_AccCalib_matrix(3, 1) = cali42;
  m_AccCalib_matrix(3, 2) = cali43;

  m_GyroBody_vec.setZero();
  m_GyroEarth_vec.setZero();

  // Kalman Filter
  m_StateX_vec.setZero();
  m_StateX_vec[0] = 1.0;
  m_StateY_vec.setZero();
  m_StateY_vec[1] = 1.0;
  m_StateZ_vec.setZero();
  m_StateZ_vec[2] = 1.0;

  m_StateXCov_matrix.setZero();
  m_StateXCov_matrix(0, 0) = m_StateXCov_matrix(1, 1) =
      m_StateXCov_matrix(2, 2) = INF;
  m_StateYCov_matrix.setZero();
  m_StateYCov_matrix(0, 0) = m_StateYCov_matrix(1, 1) =
      m_StateYCov_matrix(2, 2) = INF;
  m_StateZCov_matrix.setZero();
  m_StateZCov_matrix(0, 0) = m_StateZCov_matrix(1, 1) =
      m_StateZCov_matrix(2, 2) = INF;

  m_StateTransit_matrix.setZero();

  m_ProcessNoiseCov_matrix.setZero();
  m_ProcessNoiseCov_matrix(0, 0) = pow(d_ProcX_std, 2);
  m_ProcessNoiseCov_matrix(1, 1) = pow(d_ProcY_std, 2);
  m_ProcessNoiseCov_matrix(2, 2) = pow(d_ProcZ_std, 2);

  m_MeasNoiseCov_matrix.setZero();
  m_MeasNoiseCov_matrix(0, 0) = pow(d_MeasX_std, 2);
  m_MeasNoiseCov_matrix(1, 1) = pow(d_MeasY_std, 2);
  m_MeasNoiseCov_matrix(2, 2) = pow(d_MeasZ_std, 2);

  m_Obs_matrix.setIdentity();
  m_Identity_matrix.setIdentity();

  m_MeasRot_matrix.setZero();

  dt = 0;
  b_SensorInit = false;

  m_DCM_matrix.setIdentity();

  d_Roll = d_Pitch = d_Yaw = 0;

  RCLCPP_INFO(this->get_logger(), "ZumoImuKF initialized");

  return true;
}

///*********************************************************************************
/// Zumo sensor callback
///*********************************************************************************
void ZumoImuKF::ZumoSensorCb(
    const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg) {
  // RCLCPP_INFO(this->get_logger(), "ZumoImuKF::ZumoSensorCb");

  if (!b_SensorInit) {
    // very first data
    b_SensorInit = true;
    m_PrevSensorStamp = pMsg->header.stamp;
    return;
  }

  rclcpp::Duration mTimeDiff = m_PrevSensorStamp - pMsg->header.stamp;
  dt = mTimeDiff.seconds();

  if (pMsg->ax == pMsg->az) {
    RCLCPP_ERROR(this->get_logger(), "Accelerometer data not updating!");
    return;
  }

  // rectify accelerometer data
  TransformAccRawData(pMsg);

  // rectify gyro data
  TransformGyroRawData(pMsg);

  // rectify magnetometer data
  TransformMagRawData(pMsg);

  // rotation matrix based on current raw measurements
  UpdateMeasRotMatrix();

  // Kalman Prediction
  KalmanPredict();

  // normalize
  Normalize();

  // Kalman Update
  KalmanUpdate();

  // publish tf and an IMU msg
  PublishTF();

  m_PrevSensorStamp = pMsg->header.stamp;
}

///*********************************************************************************
/// accelerometer raw data to rectified data
///*********************************************************************************
void ZumoImuKF::TransformAccRawData(
    const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg) {
  /// conversion of ACC data
  double dAccRawX, dAccRawY, dAccRawZ;
  dAccRawX = ACC_CONVERSION_FACTOR * (pMsg->ax / (1000.0));
  dAccRawY = ACC_CONVERSION_FACTOR * (pMsg->ay / (1000.0));
  dAccRawZ = ACC_CONVERSION_FACTOR * (pMsg->az / (1000.0));

  if (true == b_PrintValues) {
    printf("Accelerometer raw X: %f\n", dAccRawX);
    printf("Accelerometer raw Y: %f\n", dAccRawY);
    printf("Accelerometer raw Z: %f\n\n", dAccRawZ);
  }

  // rectified acc
  Eigen::Matrix<double, 1, 3> m_RectifiedAcc_vec;
  Eigen::Matrix<double, 1, 4> m_RawSensor_vec(dAccRawX, dAccRawY, dAccRawZ, 1);
  m_RectifiedAcc_vec = m_RawSensor_vec * m_AccCalib_matrix;

  if (true == b_PrintValues) {
    printf("Accelerometer rectified X: %f\n", m_RectifiedAcc_vec[0]);
    printf("Accelerometer rectified Y: %f\n", m_RectifiedAcc_vec[1]);
    printf("Accelerometer rectified Z: %f\n\n", m_RectifiedAcc_vec[2]);
  }

  m_Acc_vec[0] = m_RectifiedAcc_vec[0];
  m_Acc_vec[1] = m_RectifiedAcc_vec[1];
  m_Acc_vec[2] = m_RectifiedAcc_vec[2];

  /// eq. (23) and (24)
  // Note:
  // use atan2(y,x) function for arc tangent to obtain angles plus its quadrant
  // d_Roll  =
  // d_Pitch =
}

///*********************************************************************************
/// gyro raw data to rectified data
///*********************************************************************************
void ZumoImuKF::TransformGyroRawData(
    const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg) {
  /// conversion of GYRO data
  m_GyroBody_vec[0] =
      GYRO_CONVERSION_FACTOR * M_PI * (pMsg->gx / (180.0 * 1000.0));
  m_GyroBody_vec[1] =
      GYRO_CONVERSION_FACTOR * M_PI * (pMsg->gy / (180.0 * 1000.0));
  m_GyroBody_vec[2] =
      GYRO_CONVERSION_FACTOR * M_PI * (pMsg->gz / (180.0 * 1000.0));

  if (true == b_PrintValues) {
    printf("Gyro Body X: %f\n", m_GyroBody_vec[0]);
    printf("Gyro Body Y: %f\n", m_GyroBody_vec[1]);
    printf("Gyro Body Z: %f\n\n", m_GyroBody_vec[2]);
  }
}

///*********************************************************************************
/// magnetometer raw data to rectified data
///*********************************************************************************
void ZumoImuKF::TransformMagRawData(
    const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg) {
  /// conversion of Mag data
  // [https://github.com/praneshkmr/node-lsm303/wiki/Understanding-the-calibration-of-the-LSM303-magnetometer-(compass)]
  Eigen::Matrix<double, 3, 1> mTempMag_vec;
  // subtract offset (average of min and max) from magnetometer readings
  mTempMag_vec[0] = pMsg->mx - (m_MagMin_vec[0] + m_MagMax_vec[0]) / 2;
  mTempMag_vec[1] = pMsg->my - (m_MagMin_vec[1] + m_MagMax_vec[1]) / 2;
  mTempMag_vec[2] = pMsg->mz - (m_MagMin_vec[2] + m_MagMax_vec[2]) / 2;

  mTempMag_vec.normalize();

  /// eq. (32)
  // use atan2(y,x) function for arc tangent to obtain angles plus its quadrant
  // d_Yaw =

  if (true == b_PrintValues) {
    printf("Heading: %f\n\n", d_Yaw * 180.0 / M_PI);
  }
}

///*********************************************************************************
/// Update Measurement Rotation Matrix
///*********************************************************************************
void ZumoImuKF::UpdateMeasRotMatrix() {
  m_MeasRot_matrix(0, 0) = cos(d_Yaw) * cos(d_Pitch);
  m_MeasRot_matrix(0, 1) =
      -sin(d_Yaw) * cos(d_Roll) + cos(d_Yaw) * sin(d_Pitch) * sin(d_Roll);
  m_MeasRot_matrix(0, 2) =
      sin(d_Yaw) * sin(d_Roll) + cos(d_Yaw) * sin(d_Pitch) * cos(d_Roll);
  m_MeasRot_matrix(1, 0) = sin(d_Yaw) * cos(d_Pitch);
  m_MeasRot_matrix(1, 1) =
      cos(d_Yaw) * cos(d_Roll) + sin(d_Yaw) * sin(d_Pitch) * sin(d_Roll);
  m_MeasRot_matrix(1, 2) =
      -cos(d_Yaw) * sin(d_Roll) + sin(d_Yaw) * sin(d_Pitch) * cos(d_Roll);
  m_MeasRot_matrix(2, 0) = -sin(d_Pitch);
  m_MeasRot_matrix(2, 1) = cos(d_Pitch) * sin(d_Roll);
  m_MeasRot_matrix(2, 2) = cos(d_Pitch) * cos(d_Roll);
}

///*********************************************************************************
/// Kalman Prediction
///*********************************************************************************
void ZumoImuKF::KalmanPredict() {
  // RCLCPP_INFO(this->get_logger(), "ZumoImuKF::KalmanPredict");

  // Predicted state estimates for rotation matrix column vectors
  // Help:
  // Matrices can be multiplied using * operator

  // 1) Transform gyro readings to Earth frame
  m_GyroEarth_vec = m_DCM_matrix * m_GyroBody_vec;

  // 2) Populate state transition matrix A
  // Help:
  // e.g. dTheta_y = m_GyroEarth_vec[1]*dt
  // time step is available at dt variable
  // m_StateTransit_matrix(0,0) =
  // m_StateTransit_matrix(0,1) =
  // m_StateTransit_matrix(0,2) =
  // m_StateTransit_matrix(1,0) =
  // m_StateTransit_matrix(1,1) =
  // m_StateTransit_matrix(1,2) =
  // m_StateTransit_matrix(2,0) =
  // m_StateTransit_matrix(2,1) =
  // m_StateTransit_matrix(2,2) =

  // 3) Predicted rotation matrix column vectors
  // eq. (12), (13) and (14)
  // m_StateX_vec, m_StateY_vec, m_StateZ_vec will be updated at each operation

  // m_StateX_vec =
  // m_StateY_vec =
  // m_StateZ_vec =

  // 4) Predicted state estimate covariances
  // eq. (5)
  // m_StateXCov_matrix, m_StateYCov_matrix, m_StateZCov_matrix will be updated
  // at each operation Rt is the process noise covariance matrix available as
  // m_ProcessNoiseCov_matrix Transpose of a matrix can be achieved using
  // .transpose() function
  // m_StateXCov_matrix =
  // m_StateYCov_matrix =
  // m_StateZCov_matrix =

  // 5) populate rotation matrix
  m_DCM_matrix.block<3, 1>(0, 0) = m_StateX_vec;
  m_DCM_matrix.block<3, 1>(0, 1) = m_StateY_vec;
  m_DCM_matrix.block<3, 1>(0, 2) = m_StateZ_vec;
}

///*********************************************************************************
/// normalization of the DCM matrix
///*********************************************************************************
void ZumoImuKF::Normalize() {
  double dError = 0;
  Eigen::Matrix<double, 3, 3> mTemp_vec;

  Eigen::Matrix<double, 3, 1> mRotX_vec;
  Eigen::Matrix<double, 3, 1> mRotY_vec;
  mRotX_vec = m_DCM_matrix.block<3, 1>(0, 0);
  mRotY_vec = m_DCM_matrix.block<3, 1>(0, 1);

  /// eq. (33)
  // Help:
  // Transpose of a vector: x.transpose()
  // dot product: x.dot(y)
  // dError =

  Eigen::Matrix<double, 3, 1> mRotXOrtho_vec;
  Eigen::Matrix<double, 3, 1> mRotYOrtho_vec;
  Eigen::Matrix<double, 3, 1> mRotZOrtho_vec;

  /// eq. (34) and (35)
  // mRotXOrtho_vec =
  // mRotYOrtho_vec =

  /// eq. (36)
  // cross product: x.cross(y)
  // mRotZOrtho_vec =

  Eigen::Matrix<double, 3, 1> mRotXNorm_vec;
  Eigen::Matrix<double, 3, 1> mRotYNorm_vec;
  Eigen::Matrix<double, 3, 1> mRotZNorm_vec;

  if (b_TaylorNormalization)
  /// eq. (38), (39) and (40)
  {
    // mRotXNorm_vec =
    // mRotYNorm_vec =
    // mRotZNorm_vec =
  } else
  /// eq. (37)
  // Help:
  // to normalize a vector: x.normalize()
  {
    // mRotXNorm_vec =
    // mRotYNorm_vec =
    // mRotZNorm_vec =
  }

  // set the values
  m_StateX_vec = m_DCM_matrix.block<3, 1>(0, 0) = mRotXNorm_vec;
  m_StateY_vec = m_DCM_matrix.block<3, 1>(0, 1) = mRotYNorm_vec;
  m_StateZ_vec = m_DCM_matrix.block<3, 1>(0, 2) = mRotZNorm_vec;
}

///*********************************************************************************
/// Kalman Update
///*********************************************************************************
void ZumoImuKF::KalmanUpdate() {
  // RCLCPP_INFO(this->get_logger(), "ZumoImuKF::KalmanUpdate");

  // 1) Innovations
  Eigen::Matrix<double, 3, 1> mInnovationX_vec, mInnovationY_vec,
      mInnovationZ_vec;
  Eigen::Matrix<double, 3, 1> mObservationX_vec, mObservationY_vec,
      mObservationZ_vec;

  mObservationX_vec = m_MeasRot_matrix.block<3, 1>(0, 0);
  mObservationY_vec = m_MeasRot_matrix.block<3, 1>(0, 1);
  mObservationZ_vec = m_MeasRot_matrix.block<3, 1>(0, 2);

  // eq. (6) for all three: mInnovationX_vec, mInnovationY_vec and
  // mInnovationZ_vec C matrix is available as m_Obs_matrix
  // mInnovationX_vec =
  // mInnovationY_vec =
  // mInnovationZ_vec =

  // 2) Innovation Covariances
  Eigen::Matrix<double, 3, 3> mInnovCovX, mInnovCovY, mInnovCovZ;
  // eq. (7) for all three: mInnovCovX, mInnovCovY and mInnovCovZ
  // Qt matrix is the measurement noise covariance available as
  // m_MeasNoiseCov_matrix
  // mInnovCovX =
  // mInnovCovY =
  // mInnovCovZ =

  // 3) Optimal Kalman Gains
  Eigen::Matrix<double, 3, 3> mKalmanGainX, mKalmanGainY, mKalmanGainZ;
  // eq. (8) for all three: mKalmanGainX, mKalmanGainY and mKalmanGainZ
  // Inverse of a matrix can be achieved using .inverse() function
  // mKalmanGainX =
  // mKalmanGainY =
  // mKalmanGainZ =

  // 4) Updated State Estimate
  // eq. (9) for all three states: m_StateX_vec, m_StateY_vec and m_StateZ_vec
  // m_StateX_vec =
  // m_StateY_vec =
  // m_StateZ_vec =

  // 5) Updated State Covariance
  // eq. (10) for all three: m_StateXCov_matrix, m_StateYCov_matrix and
  // m_StateZCov_matrix Help: a 3x3 identity matrix is available as
  // m_Identity_matrix
  // m_StateXCov_matrix =
  // m_StateYCov_matrix =
  // m_StateZCov_matrix =

  // 6) populate rotation matrix
  m_DCM_matrix.block<3, 1>(0, 0) = m_StateX_vec;
  m_DCM_matrix.block<3, 1>(0, 1) = m_StateY_vec;
  m_DCM_matrix.block<3, 1>(0, 2) = m_StateZ_vec;
}

///*********************************************************************************
/// publishing Transform
///*********************************************************************************
void ZumoImuKF::PublishTF() {
  // 1)KF transform
  geometry_msgs::msg::TransformStamped mTf;

  // Set the header information
  mTf.header.stamp = this->now();
  mTf.header.frame_id = "world";
  mTf.child_frame_id = "base_footprint";

  mTf.transform.translation.x = 0;
  mTf.transform.translation.y = 0;
  mTf.transform.translation.z = ZUMO_WHEEL_RADIUS;

  tf2::Matrix3x3 mMat;
  mMat.setValue(m_DCM_matrix(0, 0), m_DCM_matrix(0, 1), m_DCM_matrix(0, 2),
                m_DCM_matrix(1, 0), m_DCM_matrix(1, 1), m_DCM_matrix(1, 2),
                m_DCM_matrix(2, 0), m_DCM_matrix(2, 1), m_DCM_matrix(2, 2));
  tf2::Quaternion q;
  mMat.getRotation(q);
  mTf.transform.rotation.w = q.getW();
  mTf.transform.rotation.x = q.getX();
  mTf.transform.rotation.y = q.getY();
  mTf.transform.rotation.z = q.getZ();

  double dRoll, dPitch, dYaw;
  tf2::Matrix3x3(q).getRPY(dRoll, dPitch, dYaw);

  if (true == b_PrintValues) {
    printf("roll=%f pitch=%f yaw=%f\n\n", dRoll * 180.0 / M_PI,
           dPitch * 180.0 / M_PI, dYaw * 180.0 / M_PI);
  }

  if (b_IMUDemo) {
    p_TfBroadcaster->sendTransform(mTf);
  } else {
    // TODO
  }

  // 2)raw measurement transform
  geometry_msgs::msg::TransformStamped mTf2;

  // Set the header information
  mTf2.header.stamp = this->now();
  mTf2.header.frame_id = "world";
  mTf2.child_frame_id = "zumo_raw/base_footprint";

  mTf2.transform.translation.x = 0;
  mTf2.transform.translation.y = 0.2;
  mTf2.transform.translation.z = ZUMO_WHEEL_RADIUS;

  tf2::Matrix3x3 mMat2;
  mMat2.setValue(
      m_MeasRot_matrix(0, 0), m_MeasRot_matrix(0, 1), m_MeasRot_matrix(0, 2),
      m_MeasRot_matrix(1, 0), m_MeasRot_matrix(1, 1), m_MeasRot_matrix(1, 2),
      m_MeasRot_matrix(2, 0), m_MeasRot_matrix(2, 1), m_MeasRot_matrix(2, 2));
  tf2::Quaternion q2;
  mMat2.getRotation(q2);
  mTf2.transform.rotation.w = q2.getW();
  mTf2.transform.rotation.x = q2.getX();
  mTf2.transform.rotation.y = q2.getY();
  mTf2.transform.rotation.z = q2.getZ();

  p_TfBroadcaster->sendTransform(mTf2);
}