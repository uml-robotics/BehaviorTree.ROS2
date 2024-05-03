# TreeExecutionServer

This base class is used to implement a Behavior Tree executor wrapped inside a `rclcpp_action::Server`.

Users are expected to create a derived class to improve its functionalities, but often it can be used
out of the box directly.

Further, the terms "load" will be equivalent to "register into the `BT::BehaviorTreeFactory`".

The `TreeExecutionServer`offers the following features:

- Configurable using ROS parameters (see below).
- Load Behavior Trees definitions (XML files) from a list of folders.
- Load BT plugins from a list of folders. These plugins may depend or not on ROS.
- Invoke the execution of a Tree from an external ROS Node, using `rclcpp_action::Client`.

Furthermore, the user can customize it to:

- Register custom BT Nodes directly (static linking).
- Attach additional loggers. The **Groot2** publisher will be attached by default.
- Use the "global blackboard", a new idiom/pattern explained in [this tutorial](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/examples/t19_global_blackboard.cpp).
- Customize the feedback of the `rclcpp_action::Server`.


## ROS Parameters

Default Config

```yaml
bt_action_server:
  ros__parameters:
    action_name: bt_action_server
    behavior_tick_frequency: 100.0
    behavior_trees: '{}'
    groot2_port: 1667.0
    plugins: '{}'
    ros_plugins: '{}'

```

## action_name

The name the Action Server takes requests from

* Type: `string`
* Default Value: "bt_action_server"
* Read only: True

## behavior_tick_frequency

Frequency in Hz to tick() the Behavior tree at

* Type: `int`
* Default Value: 100
* Read only: True

*Constraints:*
 - parameter must be within bounds 1

## groot2_port

Server port value to publish Groot2 messages on

* Type: `int`
* Default Value: 1667
* Read only: True

*Constraints:*
 - parameter must be within bounds 1

## plugins

List of 'package_name/subfolder' containing BehaviorTree plugins to load into the factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates

## ros_plugins

List of 'package_name/subfolder' containing BehaviorTree ROS plugins to load into the factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates

## behavior_trees

List of 'package_name/subfolder' containing SubTrees to load into the BehaviorTree factory

* Type: `string_array`
* Default Value: {}

*Constraints:*
 - contains no duplicates
