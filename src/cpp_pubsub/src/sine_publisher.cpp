#include <cpp_pubsub/sine_publisher.hpp>

SinePublisher::SinePublisher() : rclcpp::Node("sine_publisher") {
  p_Publisher = this->create_publisher<std_msgs::msg::Float64>("sine_wave", 10);
  p_Timer = this->create_wall_timer(
      100ms, std::bind(&SinePublisher::TimerCallback, this));
  start_time_ = std::chrono::steady_clock::now();
}

void SinePublisher::TimerCallback() {
  auto current_time = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = current_time - start_time_;
  double t = elapsed.count();

  auto message = std_msgs::msg::Float64();
  message.data = AMPLITUDE * std::sin(OMEGA * t);

  RCLCPP_INFO(this->get_logger(), "Publishing: A*sin(ω*t) = %.4f (t=%.2fs)",
              message.data, t);
  p_Publisher->publish(message);
}
