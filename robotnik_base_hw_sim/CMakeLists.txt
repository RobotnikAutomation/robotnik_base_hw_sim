cmake_minimum_required(VERSION 3.8)
project(robotnik_base_hw_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robotnik_io_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(robotnik_io_controller_sim_node src/robotnik_io_controller_sim_node.cpp src/robotnik_io_controller_sim.cpp)
target_include_directories(robotnik_io_controller_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(robotnik_io_controller_sim_node
  rclcpp
  robotnik_io_msgs
)

install(TARGETS robotnik_io_controller_sim_node
  DESTINATION lib/${PROJECT_NAME})

## Mark executables and/or libraries for installation
install(TARGETS
robotnik_io_controller_sim_node
DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
config launch
DESTINATION share/${PROJECT_NAME}/
)

ament_package()
