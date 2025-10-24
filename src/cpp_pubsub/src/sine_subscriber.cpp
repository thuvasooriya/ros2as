#include <cpp_pubsub/sine_subscriber.hpp>

SineSubscriber::SineSubscriber() : rclcpp::Node("sine_subscriber") {
  p_Subscription = this->create_subscription<std_msgs::msg::Float64>(
      "sine_wave", 10, std::bind(&SineSubscriber::TopicCallback, this, _1));
}

void SineSubscriber::TopicCallback(const std_msgs::msg::Float64 &msg) const {
  RCLCPP_INFO(this->get_logger(), "Received: %.4f", msg.data);
}
