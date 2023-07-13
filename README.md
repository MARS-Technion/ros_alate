# ros_alate

A ROS2 adapter for the [AntAlate](https://www.frontiersin.org/articles/10.3389/frobt.2021.719496/full) framework

## Subscribes to Topics

1. "alate_input_operator_command"
1. "alate_input_velocity"

## Publishes to Topics

1. "alate_output_mission_control_state"
1. "alate_output_high_level_control_state"
1. "alate_output_high_level_control_telemetry"
1. "alate_output_high_level_control_platform_errors"

## Usage

```console
ros2 run ros_alate adapter
```

## About

The ros_alate adapter is a ROS2 node which has a AntAlate dispatcher, as can be seen in the class diagram below.

![Class Diagram](doc/images/class_adapter.svg)

The adapter was designed to run alongside an AntAlate application, adapting the AntAlate messages to ROS2 messages and vice versa.
The adapter is a ROS2 node, which has a [NeMALA::Dispatcher](https://gitlab.com/nemala/core/-/blob/master/doc/components.md).
The adapter spins, and when a subscribed ROS topic invokes one of its callback functions, the adapter publishes the message to the appropriate AntAlate topic.
When one of the adapter's NeMALA::Handlers becomes active, it uses the appropriate ROS2 publisher to publish its message.

In the class diagram above, only the HandlerMissionState is described, but all other handlers are constructed similarly.

The component diagram below shows the function of the adapter to translate ROS2 topics to AntAlate topics and vice versa.

![Component Diagram](doc/images/component_adapter.svg)
