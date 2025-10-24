#include <cpp_pubsub/my_subscriber.hpp>

///*********************************************************************************
/// constructor
///*********************************************************************************
MySubscriber::MySubscriber() : rclcpp::Node("my_subscriber") {
  p_Subscription = this->create_subscription<std_msgs::msg::String>(
      "my_topic", 10, std::bind(&MySubscriber::TopicCallback, this, _1));
}

///*********************************************************************************
/// Topic subscription callback
///*********************************************************************************
void MySubscriber::TopicCallback(const std_msgs::msg::String &msg) const {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
