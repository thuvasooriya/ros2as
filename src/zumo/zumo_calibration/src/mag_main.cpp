#include <zumo_calibration/zumo_mag_calib.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto p_ZumoMagCalibNode = std::make_shared<ZumoMagCalib>();

  if (false == p_ZumoMagCalibNode->Init()) {
    RCLCPP_ERROR(p_ZumoMagCalibNode->get_logger(),
                 "ZumoMagCalib could not be initialized properly");
    return -1;
  }

  rclcpp::spin(p_ZumoMagCalibNode);
  rclcpp::shutdown();
  return 0;
}
