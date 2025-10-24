#include <cpp_pubsub/sine_publisher.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SinePublisher>());
  rclcpp::shutdown();
  return 0;
}
