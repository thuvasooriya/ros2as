/**
 * Copyright (c) 2024 ENTC, University of Moratuwa
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of UoM nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.

 * Original AUTHOR(s)   :   Peshala J
 * Modified By          :
 * Date                 :   February 27, 2024
 * Comments             :
 *
 */

#ifndef MY_PUBLISHER_HPP // Header guard to prevent multiple inclusion of the
                         // same header file
#define MY_PUBLISHER_HPP

#include <chrono> // <chrono> is the standard C++ library header providing facilities for working with time durations and time points
#include <functional> // This header provides components for working with function objects and callable entities in C++
#include <memory> // This header provides components for dynamic memory management in C++
#include <string> // This header provides components for working with string objects and string manipulation in C++

#include <rclcpp/rclcpp.hpp> // This header includes the main functionalities of the ROS 2 C++ client library (rclcpp)
// It provides classes and utilities for creating ROS 2 nodes, publishers,
// subscribers, timers, services, etc.

#include <std_msgs/msg/string.hpp> // This header provides the message definition for the standard string message (std_msgs/String) in ROS 2

using namespace std::
    chrono_literals; // This allows the usage of chrono literals for
                     // representing durations in code For example, 1s
                     // represents a duration of 1 second, 100ms represents 100
                     // milliseconds, etc. This namespace provides literals for
                     // chrono durations such as seconds, milliseconds,
                     // microseconds, and nanoseconds.

class MyPublisher
    : public rclcpp::Node // The class inherits publicly from the rclcpp::Node
                          // class This indicates that MyPublisher is a ROS 2
                          // node, allowing it to utilize the functionality
                          // provided by the rclcpp library By inheriting from
                          // rclcpp::Node, MyPublisher can create publishers,
                          // subscribers, timers, services, etc., and
                          // participate in the ROS 2 communication graph

{
  // public members that are accessible from outside the class
public:
  MyPublisher(); // constructor

  // private members that are accessible only within the class
private:
  void TimerCallback(); // callback function for a timer
                        //  Callback functions are functions that are called
                        //  automatically when a certain event occurs, in this
                        //  case, when a timer expires

  rclcpp::TimerBase::SharedPtr
      p_Timer; // The shared pointer points to an object of type
               // rclcpp::TimerBase Shared pointers are smart pointers that
               // automatically manage the memory of the pointed object They
               // ensure that the object is deleted when no more shared pointers
               // are pointing to it, preventing memory leaks

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      p_Publisher; // The shared pointer points to an object of type
                   // rclcpp::Publisher<std_msgs::msg::String> In this case,
                   // p_Publisher represents a shared pointer to a ROS 2
                   // publisher object that publishes messages of type
                   // std_msgs::msg::String

  size_t sz_Count; // size_t is an unsigned integral type capable of
                   // representing the size of objects in bytes
};

#endif // MY_PUBLISHER_HPP
       //  end of header guard
