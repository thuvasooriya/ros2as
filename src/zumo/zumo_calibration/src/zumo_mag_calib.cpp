#include <zumo_calibration/zumo_mag_calib.hpp>

ZumoMagCalib::ZumoMagCalib()
    : rclcpp::Node("zumo_mag_calib"), running_min_vec(3), running_max_vec(3) {}

ZumoMagCalib::~ZumoMagCalib() { SaveCalibration(); }

bool ZumoMagCalib::Init() {
  RCLCPP_INFO(this->get_logger(), "ZumoMagCalib initializing ...");

  p_ZumoSubscription = this->create_subscription<zumo_msgs::msg::ZumoSensors>(
      "/zumo/zumo_sensors", 100,
      std::bind(&ZumoMagCalib::ZumoSensorCb, this, _1));

  running_min_vec[0] = running_min_vec[1] = running_min_vec[2] = 32767;
  running_max_vec[0] = running_max_vec[1] = running_max_vec[2] = -32768;

  RCLCPP_INFO(this->get_logger(), "ZumoMagCalib initialized");

  return true;
}

///*********************************************************************************
/// Zumo sensor callback
///*********************************************************************************
void ZumoMagCalib::ZumoSensorCb(
    const zumo_msgs::msg::ZumoSensors::ConstSharedPtr pMsg) {
  // ROS_INFO("ZumoMagCalib::ZumoSensorCb");

  running_min_vec[0] = std::min(running_min_vec[0], pMsg->mx);
  running_min_vec[1] = std::min(running_min_vec[1], pMsg->my);
  running_min_vec[2] = std::min(running_min_vec[2], pMsg->mz);

  running_max_vec[0] = std::max(running_max_vec[0], pMsg->mx);
  running_max_vec[1] = std::max(running_max_vec[1], pMsg->my);
  running_max_vec[2] = std::max(running_max_vec[2], pMsg->mz);

  char report[80];
  snprintf(report, sizeof(report),
           "min: {%+6ld, %+6ld, %+6ld}    max: {%+6ld, %+6ld, %+6ld}",
           running_min_vec[0], running_min_vec[1], running_min_vec[2],
           running_max_vec[0], running_max_vec[1], running_max_vec[2]);

  printf("%s\n", report);
  fflush(stdout);
}

void ZumoMagCalib::SaveCalibration() {
  std::ofstream calib_file("results/mag_calib.txt");
  if (calib_file.is_open()) {
    calib_file << "min " << running_min_vec[0] << " " << running_min_vec[1]
               << " " << running_min_vec[2] << "\n";
    calib_file << "max " << running_max_vec[0] << " " << running_max_vec[1]
               << " " << running_max_vec[2] << "\n";
    calib_file.close();
    puts("\nMagnetometer calibration saved to results/mag_calib.txt");
  }
}
