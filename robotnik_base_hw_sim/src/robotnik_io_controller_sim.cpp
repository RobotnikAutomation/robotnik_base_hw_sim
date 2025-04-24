#include <robotnik_base_hw_sim/robotnik_io_controller_sim.hpp>

#include <chrono>

namespace robotnik_io_controller_sim
{

RobotnikIOControllerSim::RobotnikIOControllerSim()
: Node("robotnik_io_controller_sim")
{
  RCLCPP_INFO(get_logger(), "Initializing robotnik_io_controller_sim");

  // Read parameters from the parameter server
  RCLCPP_DEBUG(get_logger(), "Reading parameters from server");
  read_parameters();

  RCLCPP_DEBUG(get_logger(), "Advertising topics");
  advertise_topics();

  RCLCPP_DEBUG(get_logger(), "Advertising services");
  advertise_services();

  configure_inputs_outputs();
}

RobotnikIOControllerSim::~RobotnikIOControllerSim()
{
  // Node has been terminated
}

void RobotnikIOControllerSim::read_parameters()
{
  // Declare default values for parameters
  std::vector<std::string> default_wheel_names;
  declare_parameter<std::vector<std::string>>("wheel_names", default_wheel_names);
  
  std::vector<int> default_analog_inputs;
  declare_parameter<std::vector<int>>("analog_inputs", default_analog_inputs);
  
  std::vector<int> default_analog_outputs;
  declare_parameter<std::vector<int>>("analog_outputs", default_analog_outputs);
  
  std::vector<int> default_digital_inputs;
  declare_parameter<std::vector<int>>("digital_inputs", default_digital_inputs);
  
  std::vector<int> default_digital_outputs;
  declare_parameter<std::vector<int>>("digital_outputs", default_digital_outputs);

  // Get parameters from the parameter server
  wheel_names_ = get_parameter("wheel_names").as_string_array();
  digital_inputs_n_ = get_parameter("digital_inputs").as_integer_array();
  digital_outputs_ = get_parameter("digital_outputs").as_integer_array();
  analog_inputs_ = get_parameter("analog_inputs").as_integer_array();
  analog_outputs_ = get_parameter("analog_outputs").as_integer_array();

  // Check that all vectors have the same size
  if (!(wheel_names_.size() == digital_inputs_n_.size() &&
        wheel_names_.size() == digital_outputs_.size() &&
        wheel_names_.size() == analog_inputs_.size() &&
        wheel_names_.size() == analog_outputs_.size())) {
    RCLCPP_FATAL(get_logger(), "Parameter size mismatch: all parameter vectors must have the same size.");
    rclcpp::shutdown();
    exit(EXIT_FAILURE);
  }
}

void RobotnikIOControllerSim::advertise_topics()
{
  pub_io_ = create_publisher<robotnik_io_msgs::msg::InputsOutputs>("~/io", rclcpp::SensorDataQoS());

  publish_timer_ = create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotnikIOControllerSim::timer_callback, this));
}

void RobotnikIOControllerSim::advertise_services()
{
  srv_set_digital_output_ = create_service<robotnik_io_msgs::srv::SetDigitalOutput>(
    "~/set_digital_output", std::bind(&RobotnikIOControllerSim::set_digital_output_callback, this, std::placeholders::_1, std::placeholders::_2));
}

void RobotnikIOControllerSim::timer_callback()
{
  pub_io_->publish(io_msg_);
  RCLCPP_DEBUG(get_logger(), "Published IO message");
} 

void RobotnikIOControllerSim::configure_inputs_outputs()
{  
  uint8_t digital_input_i = 1;
  uint8_t digital_output_i = 1;
  uint8_t analog_input_i = 1;
  uint8_t analog_output_i = 1;
  uint8_t i = 0;
  for(auto const& wheel_name : wheel_names_) {
    RCLCPP_DEBUG(get_logger(), "Wheel name: %s", wheel_name.c_str());
    
    // Configure the inputs and outputs for each wheel
    auto const& digital_inputs_n = digital_inputs_n_[i];
    for (uint8_t j = 0; j < digital_inputs_n; ++j) {
      std::string name = (wheel_name + "/digital_input_" + std::to_string(j)); 
      io_msg_.digital_inputs.push_back(create_digital_io(name, digital_input_i, false));
      digital_input_i++;
    }

    auto const& digital_outputs = digital_outputs_[i];
    for (uint8_t j = 0; j < digital_outputs; ++j) {
      std::string name = (wheel_name + "/digital_output_" + std::to_string(j)); 
      io_msg_.digital_outputs.push_back(create_digital_io(name, digital_output_i, false));
      digital_output_i++;
    }

    auto const& analog_inputs = analog_inputs_[i];
    for (uint8_t j = 0; j < analog_inputs; ++j) {
      std::string name = (wheel_name + "/analog_input_" + std::to_string(j));
      io_msg_.analog_inputs.push_back(create_analog_io(name, analog_input_i, 0.0));
      analog_input_i++;
    }

    auto const& analog_outputs = analog_outputs_[i];
    for (uint8_t j = 0; j < analog_outputs; ++j) {
      std::string name = (wheel_name + "/analog_output_" + std::to_string(j));
      io_msg_.analog_outputs.push_back(create_analog_io(name, analog_output_i, 0.0));
      analog_output_i++;
    }

    i++;
  }
}

DigitalIO RobotnikIOControllerSim::create_digital_io(const std::string& name, int id, bool value)
{
  robotnik_io_msgs::msg::DigitalIO digital_io;
  digital_io.name = name;
  digital_io.id = id;
  digital_io.value = value;

  return digital_io;
}

AnalogIO RobotnikIOControllerSim::create_analog_io(const std::string& name, int id, double value)
{
  robotnik_io_msgs::msg::AnalogIO analog_io;
  analog_io.name = name;
  analog_io.id = id;
  analog_io.value = value;

  return analog_io;
}

void RobotnikIOControllerSim::set_digital_output_callback(SetDigitalOutput::Request::ConstSharedPtr req,
  SetDigitalOutput::Response::SharedPtr res)
{
  RCLCPP_INFO(get_logger(), "RobotnikIOControllerSim::set_output_callback()");
  std::string msg;
  if (static_cast<std::size_t>(req->output.id) <= 0ull)
  {
    msg = "Output must be greater than 0";
    RCLCPP_ERROR(get_logger(), "RobotnikIOControllerSim::set_output_callback(): %s", msg.c_str());
    res->response.success = false;
    res->response.message = msg;
    return;
  }
  if (static_cast<std::size_t>(req->output.id) > io_msg_.digital_outputs.size())
  {
    msg = "Output must be less than " + std::to_string(io_msg_.digital_outputs.size());
    RCLCPP_ERROR(get_logger(), "RobotnikIOControllerSim::set_output_callback(): %s", msg.c_str());
    res->response.success = false;
    res->response.message = msg;
    return;
  }
  io_msg_.digital_outputs[req->output.id - 1].value = req->output.value;
  msg = "Output " + std::to_string(req->output.id) + " set to " + std::to_string(req->output.value);
  RCLCPP_INFO(get_logger(), "RobotnikIOControllerSim::set_output_callback(): %s", msg.c_str());
  res->response.message = msg;
  res->response.success = true;
}

} // namespace robotnik_io_controller_sim
