# robotnik_base_hw_sim

This package is intended to simulate the Robotnik base hw common in most of the platforms

## robotnik_base_hw_sim_node

### Bringup

Run the following launch inside the desired namespace.

**Arguments**

* prefix: prefix for the default joints (for multirobots envs)
* config_yaml: default config yaml. Every robot should have one.

For RB2:
```
roslaunch robotnik_base_hw_sim rb2_hw_sim.launch prefix:=rb2_a config_yaml:=rb2 
```

For SummitXLS
```
roslaunch robotnik_base_hw_sim summit_xl_hw_sim.launch prefix:=summit_xl_a config_yaml:=summit_xls
```

## robotnik_elevator_controller plugin

Plugin to simulate the Elevator interface available in some robots.

It requires a transmission of **<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>**.

Example of control configuration:

```
  robotnik_elevator_controller:
    type        : "robotnik_elevator_controller/RobotnikElevatorController"
    joint:    
      elevator_joint:
        name: rb2_a_elevator_platform_joint
    elevator_position_up: 0.05
    elevator_position_down: 0.0
```
