# History

## Version 0

### Version 0.0

**Version 0.0.2** (2022/04/07)

- Parameter to define if it is published the exact ticks (based on the
  current position of the joints) or the more realistic one (integer
  accumulator susceptible to round errors)
- Publish tick count of each wheel (topic: `motors_ticks`; ROS message:
  [`motors_output_array_data`](https://gitlab.inesctec.pt/CRIIS/inesctec_robotics_custom_interfaces_stack/-/blob/master/itrci_hardware/msg/motor_output.msg))
- Modify the plugin to improve the code readability
  (`diff_drive_controller_rbs/SpeedLimiter`,
  `diff_drive_controller_rbs/Odometry`)
- Improve documentation of the plugin

**Version 0.0.1** (2022/04/06)

- Base driver for simulating differential robots in Gazebo based on
  [`diff_drive_controller`](https://github.com/ros-controls/ros_controllers/tree/melodic-devel/diff_drive_controller)
- Declaration of the plugin in the [`package.xml`](package.xml) file

**Version 0.0.0** (2022/04/06)

- Initialization of the Git repository
- Initialization of the ROS package
