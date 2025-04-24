# robotnik_base_hw_sim

This package aims to simulate the different interfaces provided by robotnik_base_hw on a real robot, enabling testing and development without requiring physical hardware.

## Dependencies

- [robotnik_interfaces](https://www.github.com/RobotnikAutomation/robotnik_interfaces)

### robotnik_io_controller_sim

Simulates IO controller interfaces. Publishes dummy inputs and outputs data and allows to set the digital outputs values.

#### Docker

You can run this component with docker compose. Setting the ROBOT environment variable to the desired robot name will load the corresponding configuration file located in robotnik_base_hw_sim/config/robotnik_io_controller_sim

