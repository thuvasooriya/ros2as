#include <zumo_calibration/zumo_acc_calib.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto p_ZumoAccCalibNode = std::make_shared<ZumoAccCalib>();

  if (false == p_ZumoAccCalibNode->Init()) {
    RCLCPP_ERROR(p_ZumoAccCalibNode->get_logger(),
                 "ZumoAccCalib could not be initialized properly");
    return -1;
  }

  rclcpp::spin(p_ZumoAccCalibNode);
  rclcpp::shutdown();
  return 0;
}