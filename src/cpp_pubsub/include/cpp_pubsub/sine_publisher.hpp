#ifndef SINE_PUBLISHER_HPP
#define SINE_PUBLISHER_HPP

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

using namespace std::chrono_literals;

class SinePublisher : public rclcpp::Node {
public:
  SinePublisher();

private:
  void TimerCallback();

  rclcpp::TimerBase::SharedPtr p_Timer;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr p_Publisher;
  std::chrono::steady_clock::time_point start_time_;

  static constexpr double AMPLITUDE = 1.0;
  static constexpr double OMEGA = 1.0;
};

#endif
