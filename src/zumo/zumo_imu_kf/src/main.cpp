#include <zumo_imu_kf/zumo_imu_kf.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto p_ZumoImuKF = std::make_shared<ZumoImuKF>();

  if (false == p_ZumoImuKF->Init()) {
    RCLCPP_ERROR(p_ZumoImuKF->get_logger(),
                 "ZumoImuKF could not be initialized properly");
    return -1;
  }

  rclcpp::spin(p_ZumoImuKF);
  rclcpp::shutdown();
  return 0;
}
