# diff_drive_controller_rbs

**Version 0.0.1** (2022/04/06)

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

- Base driver for simulating differential robots in Gazebo based on
  [`diff_drive_controller`](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/diff_drive_controller)
- Declaration of the plugin in the [`package.xml`](package.xml) file
- Initialization of the Git repository
- Initialization of the ROS package

**The next version will add these features:**

- Modify the plugin to improve the code readability
- Improve documentation of the plugin

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

- TBD

### Publishes

- TBD

### Subscribes

- TBD

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
