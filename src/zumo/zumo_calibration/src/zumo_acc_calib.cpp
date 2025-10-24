#include <zumo_calibration/zumo_acc_calib.hpp>

///*********************************************************************************
/// Constructor
///*********************************************************************************
ZumoAccCalib::ZumoAccCalib() : rclcpp::Node("zumo_acc_calib") {}

///*********************************************************************************
/// Deconstructor
///*********************************************************************************
ZumoAccCalib::~ZumoAccCalib() {
  p_KeyboardThread->join();
  delete p_KeyboardThread;
}

///*********************************************************************************
/// Initialization
///*********************************************************************************
bool ZumoAccCalib::Init() {
  RCLCPP_INFO(this->get_logger(), "ZumoAccCalib initializing ...");

  // m_ZumoSensors_sub = mNH.subscribe("zumo/sensors_stamped", 100,
  // &ZumoAccCalib::ZumoSensorCb, this);
  p_ZumoSubscription = this->create_subscription<zumo_msgs::msg::ZumoSensors>(
      "/zumo/zumo_sensors", 100,
      std::bind(&ZumoAccCalib::ZumoSensorCb, this, _1));

  i_Pose = POSE_UNKNOWN;
  i_MeasCount = 0;

  b_DataCollected = false;

  // serial read thread
  p_KeyboardThread =
      new boost::thread(boost::bind(&ZumoAccCalib::KeyboardLoop, this));

  RCLCPP_INFO(this->get_logger(), "ZumoAccCalib initialized");

  return true;
}

///*********************************************************************************
/// Zumo sensor callback
///*********************************************************************************
void ZumoAccCalib::ZumoSensorCb(
    const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg) {
  // RCLCPP_INFO(this->get_logger(), "ZumoAccCalib::ZumoSensorCb");

  boost::mutex::scoped_lock lock(m_Pose_mutex);

  if (POSE_UNKNOWN == i_Pose) {
    return;
  }

  if (i_MeasCount > MEAS_COUNT - 1) {
    return;
  }

  if (POSE_Z_UPWARDS == i_Pose) {
    // puts("POSE_Z_UPWARDS == i_Pose");
    SetMatrices(i_MeasCount, 0, 0, GRAVITATIONAL_ACC, pMsg->ax, pMsg->ay,
                pMsg->az);
  } else if (POSE_Z_DOWNWARDS == i_Pose) {
    // puts("POSE_Z_DOWNWARDS == i_Pose");
    SetMatrices(i_MeasCount + MEAS_COUNT, 0, 0, -GRAVITATIONAL_ACC, pMsg->ax,
                pMsg->ay, pMsg->az);
  } else if (POSE_Y_UPWARDS == i_Pose) {
    // puts("POSE_Y_UPWARDS == i_Pose");
    SetMatrices(i_MeasCount + 2 * MEAS_COUNT, 0, GRAVITATIONAL_ACC, 0, pMsg->ax,
                pMsg->ay, pMsg->az);
  } else if (POSE_Y_DOWNWARDS == i_Pose) {
    // puts("POSE_Y_DOWNWARDS == i_Pose");
    SetMatrices(i_MeasCount + 3 * MEAS_COUNT, 0, -GRAVITATIONAL_ACC, 0,
                pMsg->ax, pMsg->ay, pMsg->az);
  } else if (POSE_X_UPWARDS == i_Pose) {
    // puts("POSE_X_UPWARDS == i_Pose");
    SetMatrices(i_MeasCount + 4 * MEAS_COUNT, GRAVITATIONAL_ACC, 0, 0, pMsg->ax,
                pMsg->ay, pMsg->az);
  } else if (POSE_X_DOWNWARDS == i_Pose) {
    // puts("POSE_X_DOWNWARDS == i_Pose");
    SetMatrices(i_MeasCount + 5 * MEAS_COUNT, -GRAVITATIONAL_ACC, 0, 0,
                pMsg->ax, pMsg->ay, pMsg->az);
  }

  i_MeasCount++;
}

///*********************************************************************************
/// Setting the matrices
///*********************************************************************************
void ZumoAccCalib::SetMatrices(int iRow, double dAx, double dAy, double dAz,
                               double dAmx, double dAmy, double dAmz) {
  m_AccelerationMatrix(iRow, 0) = dAx;
  m_AccelerationMatrix(iRow, 1) = dAy;
  m_AccelerationMatrix(iRow, 2) = dAz;

  m_MeasurementMatrix(iRow, 0) = ACC_CONVERSION_FACTOR * (dAmx / (1000.0));
  m_MeasurementMatrix(iRow, 1) = ACC_CONVERSION_FACTOR * (dAmy / (1000.0));
  m_MeasurementMatrix(iRow, 2) = ACC_CONVERSION_FACTOR * (dAmz / (1000.0));
  m_MeasurementMatrix(iRow, 3) = 1;
}

///*********************************************************************************
/// Keyboard input
///*********************************************************************************
void ZumoAccCalib::KeyboardLoop() {
  puts("\n\n===============================================");
  puts("Place the Zumo robot in \"local_z upwards\" pose");
  DataCollection(POSE_Z_UPWARDS);

  puts("Place the Zumo robot in \"local_z downwards\" pose");
  DataCollection(POSE_Z_DOWNWARDS);

  puts("Place the Zumo robot in \"local_y upwards\" pose");
  DataCollection(POSE_Y_UPWARDS);

  puts("Place the Zumo robot in \"local_y downwards\" pose");
  DataCollection(POSE_Y_DOWNWARDS);

  puts("Place the Zumo robot in \"local_x upwards\" pose");
  DataCollection(POSE_X_UPWARDS);

  puts("Place the Zumo robot in \"local_x downwards\" pose");
  DataCollection(POSE_X_DOWNWARDS);

  b_DataCollected =
      true; // we assume that the sensor data flow is uninterrupted.

  Calibrate();

  // display
  puts("measurements\n");
  std::cout << m_MeasurementMatrix << endl;

  puts("\nknown accelerations\n");
  std::cout << m_AccelerationMatrix << endl;

  puts("\nCalibration matrix\n");
  std::cout << m_CalibrationMatrix << endl;

  puts("Press CTRL+C to end!");
}

///*********************************************************************************
/// Keyboard input
///*********************************************************************************
void ZumoAccCalib::DataCollection(int iPose) {
  char c = 0;

  while (c != 'y') {
    puts("Press 'y' and ENTER when you are done!");
    cin >> c;
  }

  {
    boost::mutex::scoped_lock lock(m_Pose_mutex);
    i_Pose = iPose;
    i_MeasCount = 0;
  }

  puts("Collecting data ...");
  // allow for data collected for 2 seconds
  rclcpp::sleep_for(2s);
}

///*********************************************************************************
/// Do the least square calibration
///*********************************************************************************
void ZumoAccCalib::Calibrate() {
  // least square calibration
  Eigen::Matrix<double, 4, MEAS_COUNT * 6> mTranspose =
      m_MeasurementMatrix.transpose();
  Eigen::Matrix<double, 4, 4> mTempMatrix = mTranspose * m_MeasurementMatrix;
  Eigen::Matrix<double, 4, MEAS_COUNT * 6> mCalibrationMatrix_ =
      mTempMatrix.inverse() * mTranspose;
  m_CalibrationMatrix = mCalibrationMatrix_ * m_AccelerationMatrix;
}