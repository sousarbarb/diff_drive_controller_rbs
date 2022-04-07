# diff_drive_controller_rbs

**Version 0.0.2** (2022/04/07)

This project is based on the
[`diff_drive_controller`](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/diff_drive_controller)
from the library `ros_controllers`
([git](https://github.com/ros-controls/ros_controllers/tree/melodic-devel),
[wiki](http://wiki.ros.org/ros_controllers)). The main goal is to publish the
tick count of each wheel instead of publishing directly the odometry.

Given that there is no standard for publishing the tick count of each wheel
(this data is usually processed by the driver for the robot's motors), the type
of message used is the same used by
[INESC TEC](https://www.inesctec.pt/en) /
[CRIIS](https://www.inesctec.pt/en/centres/criis) for its robotics navigation 
stack, i.e.,
[`motors_output_array_data`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/master/itrci_hardware/msg/motor_output.msg).

**With this version, it is possible to do:**

- Parameter to define if it is published the exact ticks (based on the 
  current position of the joints) or the more realistic one (integer 
  accumulator susceptible to round errors)
- Publish tick count of each wheel (topic: `motors_ticks`; ROS message:
  [`motors_output_array_data`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/master/itrci_hardware/msg/motor_output.msg))
- Modify the plugin to improve the code readability 
  (`diff_drive_controller_rbs/SpeedLimiter`, 
  `diff_drive_controller_rbs/Odometry`)
- Improve documentation of the plugin
- Base driver for simulating differential robots in Gazebo based on
  [`diff_drive_controller`](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/diff_drive_controller)
- Declaration of the plugin in the [`package.xml`](package.xml) file
- Initialization of the Git repository
- Initialization of the ROS package

**The next version will add these features:**

-  Controller compatible with different wheels radius

## ROS

**Current version:**

- Ubuntu 18.04 LTS
- ROS Melodic

### Dependencies

- controller_interface
- control_msgs
- dynamic_reconfigure
- geometry_msgs
- hardware_interface
- [itrci_hardware](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/tree/master/itrci_hardware)
- nav_msgs
- pluginlib
- realtime_tools
- tf
- urdf

### Parameters

- Same as [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller)
- `motors_encoders_resolution`: resolution of the simulated encoders
- `motors_gear_reduction`: gear reduction of the simulated motors
- `enable_exact_ticks`:
  - `true`: accumulated ticks computed based on the joints position (Gazebo 
    accumulates the position of continuous joint, does not restrain their 
    range to 360deg)
  - `false`: ticks accumulate the difference between the current and 
    previous position of the wheel (susceptible to accumulated rounding errors)

### Publishes

- Same as [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller)
- [`motors_ticks`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/master/itrci_hardware/msg/motors_array_output.msg)

### Subscribes

- Same as [`diff_drive_controller`](http://wiki.ros.org/diff_drive_controller)

### Services

None.

### Actions

None.

## Usage

## Contacts

- Ricardo B. Sousa ([gitlab](https://gitlab.com/sousarbarb/),
  [github](https://github.com/sousarbarb/),
  [personal](mailto:sousa.ricardob@outlook.com),
  [feup:professor](mailto:rbs@fe.up.pt),
  [feup:student](mailto:up201503004@edu.fe.up.pt),
  [inesctec](mailto:ricardo.b.sousa@inesctec.pt))
