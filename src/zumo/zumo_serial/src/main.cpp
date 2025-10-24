#include <zumo_serial/zumo_serial_node.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto p_ZumoSerialNode = std::make_shared<ZumoSerialNode>();

  if (false == p_ZumoSerialNode->Init()) {
    RCLCPP_ERROR(p_ZumoSerialNode->get_logger(),
                 "ZumoSerialNode could not be initialized properly");
    return -1;
  }

  rclcpp::spin(p_ZumoSerialNode);
  rclcpp::shutdown();
  return 0;
}
