#include <cpp_pubsub/sine_subscriber.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SineSubscriber>());
  rclcpp::shutdown();
  return 0;
}
