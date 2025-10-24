#include <cpp_pubsub/my_publisher.hpp>

///*********************************************************************************
/// constructor
///*********************************************************************************
MyPublisher::MyPublisher() : rclcpp::Node("my_publisher"), sz_Count(0) {
  p_Publisher = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
  p_Timer = this->create_wall_timer(
      500ms, std::bind(&MyPublisher::TimerCallback, this));
}

///*********************************************************************************
///
///*********************************************************************************
void MyPublisher::TimerCallback() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(sz_Count++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  p_Publisher->publish(message);
}
