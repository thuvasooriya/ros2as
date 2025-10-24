#include <cpp_pubsub/my_subscriber.hpp> // Include directive for the header file "my_subscriber.hpp" located in the "cpp_pubsub" directory
// This header file contains declarations related to the MyPublisher class

///*********************************************************************************
/// constructor
///*********************************************************************************
MySubscriber::MySubscriber()
    : rclcpp::Node(
          "my_subscriber") // Initializes a MySubscriber object
                           // It initializes the base class rclcpp::Node with
                           // the name "my_subscriber"
{
  p_Subscription = this->create_subscription<std_msgs::msg::String>(
      "my_topic", // Creates a subscription to "my_topic" with a message type of
                  // std_msgs::msg::String
      10,         // The subscription will have a queue size of 10
      std::bind(&MySubscriber::TopicCallback, // callback function for handling
                                              // received messages
                this, // Pointer to the current instance of MySubscriber
                _1) // Pass placeholder for the message argument (1st argument)
  );
}

///*********************************************************************************
/// Topic subscription callback
///*********************************************************************************
void MySubscriber::TopicCallback(const std_msgs::msg::String &msg) const {
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'",
              msg.data.c_str()); // ROS2 logging
}
