#include <rclcpp/rclcpp.hpp>
#include <robotnik_base_hw_sim/robotnik_io_controller_sim.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robotnik_io_controller_sim::RobotnikIOControllerSim>();
  rclcpp::spin(node);
  return 0;
}