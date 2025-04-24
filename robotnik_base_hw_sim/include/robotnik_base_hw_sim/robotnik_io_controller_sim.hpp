// Copyright (c) 2025, Robotnik Automation
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
// 
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#pragma once
#ifndef ROBOTNIK_IO_CONTROLLER_SIM__ROBOTNIK_IO_CONTROLLER_SIM_HPP
#define ROBOTNIK_IO_CONTROLLER_SIM__ROBOTNIK_IO_CONTROLLER_SIM_HPP


#include <rclcpp/rclcpp.hpp>

#include <robotnik_io_msgs/msg/inputs_outputs.hpp>
#include <robotnik_io_msgs/msg/digital_io.hpp>
#include <robotnik_io_msgs/msg/analog_io.hpp>
#include <robotnik_io_msgs/srv/set_digital_output.hpp>


using SetDigitalOutput = robotnik_io_msgs::srv::SetDigitalOutput;
using DigitalIO = robotnik_io_msgs::msg::DigitalIO;
using AnalogIO = robotnik_io_msgs::msg::AnalogIO;

namespace robotnik_io_controller_sim
{

struct RobotnikIOControllerSim : public rclcpp::Node
{
  explicit RobotnikIOControllerSim();

  ~RobotnikIOControllerSim();

private:

  void read_parameters();
  void advertise_topics();
  void advertise_services();
  
  void timer_callback();

  void configure_inputs_outputs();
  DigitalIO create_digital_io(const std::string& name, int id, bool value);
  AnalogIO create_analog_io(const std::string& name, int id, double value);
  void set_digital_output_callback(SetDigitalOutput::Request::ConstSharedPtr req,
    SetDigitalOutput::Response::SharedPtr res);

  rclcpp::TimerBase::SharedPtr publish_timer_;


  // Publishers
  rclcpp::Publisher<robotnik_io_msgs::msg::InputsOutputs>::SharedPtr pub_io_{ nullptr };

  // Services
  rclcpp::Service<SetDigitalOutput>::SharedPtr srv_set_digital_output_{ nullptr };

  std::vector<std::string> wheel_names_;
  std::vector<int64_t> digital_inputs_n_, digital_outputs_, analog_inputs_, analog_outputs_;

  robotnik_io_msgs::msg::InputsOutputs io_msg_;
  
};

} // namespace robotnik_io_controller_sim

#endif  // ROBOTNIK_IO_CONTROLLER_SIM__ROBOTNIK_IO_CONTROLLER_SIM_HPP
