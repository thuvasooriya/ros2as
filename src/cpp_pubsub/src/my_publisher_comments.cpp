#include <cpp_pubsub/my_publisher.hpp> // Include directive for the header file "my_publisher.hpp" located in the "cpp_pubsub" directory
// This header file contains declarations related to the MyPublisher class

///*********************************************************************************
/// constructor
///*********************************************************************************
MyPublisher::MyPublisher()
    : rclcpp::Node("my_publisher"),
      sz_Count(0) // Initializes a MyPublisher object
                  // It initializes the base class rclcpp::Node with the name
                  // "my_publisher" and the member variable sz_Count to 0

{
  p_Publisher = this->create_publisher<std_msgs::msg::String>(
      "my_topic", 10); // creates a publisher with topic name: "my_topic"
                       // 10: queue size - buffers up to 10 messages before
                       // discarding older ones

  p_Timer = this->create_wall_timer(
      500ms,
      std::bind(
          &MyPublisher::TimerCallback,
          this)); // creates a timer that triggers every 500ms
                  // The 2nd argument is a function pointer to the TimerCallback
                  // member function of the MyPublisher class whenever the timer
                  // triggers, TimerCallback function gets called.
}

///*********************************************************************************
/// Callback for the timer events
///*********************************************************************************
void MyPublisher::TimerCallback() {
  auto message =
      std_msgs::msg::String(); // This creates an empty string message object,
                               // ready to be populated with data auto keyword
                               // automatically deduces the type based on the
                               // initialization expression

  message.data =
      "Hello, world! " +
      std::to_string(
          sz_Count++); // Concatenates the string "Hello, world! " with the
                       // current value of sz_Count converted to a string fills
                       // the data segment of the message to be published

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
              message.data.c_str()); // ROS2 logging
  p_Publisher->publish(
      message); // publishes the generated message through the publisher
}
