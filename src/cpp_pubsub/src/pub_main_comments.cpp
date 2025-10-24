#include <cpp_pubsub/my_publisher.hpp> // Include directive for the header file "my_publisher.hpp" located in the "cpp_pubsub" directory
// This header file contains declarations related to the MyPublisher class

int main(int argc,
         char *argv[]) // the entry point of the program
                       // argc: number of command-line arguments; argv: an array
                       // of strings representing the command-line arguments
{
  rclcpp::init(
      argc, argv); // This function initializes the ROS 2 runtime environment,
                   // including memory allocation and system resources setup

  rclcpp::spin(
      std::make_shared<
          MyPublisher>()); // rclcpp::spin() allows the corresponding ros node
                           // to start the ROS2 event loop This function blocks
                           // the execution of the main thread and processes ROS
                           // 2 events such as callbacks, timers, subscriptions,
                           // etc. This creates a new instance of the
                           // MyPublisher class and wraps it in a shared
                           // pointer, ensuring proper memory management

  rclcpp::shutdown(); // Shuts down and cleans up resources

  return 0;
}
