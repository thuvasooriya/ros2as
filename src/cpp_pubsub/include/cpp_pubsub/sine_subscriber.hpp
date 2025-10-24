#ifndef SINE_SUBSCRIBER_HPP
#define SINE_SUBSCRIBER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using std::placeholders::_1;

class SineSubscriber : public rclcpp::Node {
public:
  SineSubscriber();

private:
  void TopicCallback(const std_msgs::msg::Float64 &msg) const;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr p_Subscription;
};

#endif
