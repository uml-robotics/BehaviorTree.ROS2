# Sample Behaviors

For documentation on sample behaviors included in this package please see the BehaviorTree.CPP [ROS 2 Integration documentation](https://www.behaviortree.dev/docs/ros2_integration) .

# TreeExecutionServer Documentation and Example

This package also includes an example Behavior Tree Executor that is designed to make building, combining and executing [BehaviorTree.CPP](https://www.behaviortree.dev/docs/intro) based Behaviors easy and reusable.
This Executor includes an Action Server that is able to register plugins or directly linked Behaviors and Trees/Subtrees so that a user can execute any known BehaviorTree by simply sending the name of it to the server.

The `TreeExecutionServer` class offers several overridable methods that that can be used to meet your specific needs. Please see [tree_execution_server.hpp](../behaviortree_ros2/include/behaviortree_ros2/tree_execution_server.hpp) for descriptions and requirements of each virtual method. You can also refer to [sample_bt_executor.cpp](src/sample_bt_executor.cpp) for a working example of the `TreeExecutionServer`.

A launch file is included that starts the Execution Server and loads a list of plugins and BehaviorTrees from `yaml` file:
``` bash
ros2 launch btcpp_ros2_samples sample_bt_executor.launch.xml
```

> *NOTE:* For documentation on the `yaml` parameters please see [bt_executor_parameters.md](../behaviortree_ros2/bt_executor_parameters.md).

As the Server starts up it will print out the name of the ROS Action followed by the plugins and BehaviorTrees it was able to load.
```
[bt_action_server]: Starting Action Server: bt_action_server
[bt_action_server]: Loaded Plugin: libdummy_nodes_dyn.so
[bt_action_server]: Loaded Plugin: libmovebase_node_dyn.so
[bt_action_server]: Loaded Plugin: libcrossdoor_nodes_dyn.so
[bt_action_server]: Loaded ROS Plugin: libsleep_plugin.so
[bt_action_server]: Loaded BehaviorTree: door_closed.xml
[bt_action_server]: Loaded Beha viorTree: cross_door.xml
```

To call the Action Server from the command line:
``` bash
ros2 action send_goal /bt_action_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: CrossDoor}"
```

You can also try a Behavior that is a ROS Action or Service client itself.
```bash
ros2 action send_goal /bt_action_server btcpp_ros2_interfaces/action/ExecuteTree "{target_tree: SleepActionSample}"
```
