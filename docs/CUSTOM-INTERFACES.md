# Creating custom ROS 2 interfaces (messages and services)

The template publishes `std_msgs/Float64MultiArray` and subscribes to
`geometry_msgs/Twist`. Both are standard interfaces. To use your own message or
service type, the type must be compiled into the micro-ROS static library,
because micro-ROS does not generate type support on the device at runtime.

The template does not ship a custom interface of its own. The only extra package
it pulls is the standard `control_msgs`, listed in
`uros_example/micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages/extra_packages.repos`.
You create your own interface by following the steps below.

## Overview

The flow has two halves.

1. Define the interface as a normal ROS 2 package on the host, so the agent and
   other nodes know the type.
2. Bake that package into the firmware's micro-ROS library by dropping it into
   `extra_packages` and forcing a library rebuild.

Both sides must use the same interface definition, or the agent and the board
will disagree on the wire format.

## 1. Define the interface package (host side)

Follow the official tutorial,
[Creating custom msg and srv files](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Custom-ROS2-Interfaces.html).
In short, create a package with a `.msg` or `.srv` file.

```bash
cd ~/microros_ws/src           # or your ROS 2 workspace
ros2 pkg create --build-type ament_cmake my_robot_msgs
mkdir my_robot_msgs/msg
```

Write the message fields in `my_robot_msgs/msg/MotorState.msg`.
```
float64[6] data
uint32     stamp_ms
```

Register it in `my_robot_msgs/CMakeLists.txt`.
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
)
```
Add the matching lines to `package.xml`.
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Build and source it so the agent can use the type.
```bash
cd ~/microros_ws && colcon build --packages-select my_robot_msgs
source install/local_setup.bash
```

## 2. Bake the package into the firmware library

Copy the entire package directory into the micro-ROS library generator's
extra-packages folder inside your STM32 project.

```
uros_example/micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages/my_robot_msgs/
```

Force a clean regeneration by deleting the previously built library, so the
Docker pre-build step rebuilds it with your type included.

```bash
rm -rf uros_example/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/
```

Now press the clean button first, then build. In STM32CubeIDE choose
`Project -> Clean...`, then build the project. The pre-build step regenerates
`libmicroros` with `my_robot_msgs` compiled in.

## 3. Use it on the device

```c
#include <my_robot_msgs/msg/motor_state.h>

my_robot_msgs__msg__MotorState pub_msg;

rclc_publisher_init_default(&publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(my_robot_msgs, msg, MotorState),
    "motor_state");
```

The C names are the snake_case form of the ROS type. The type
`my_robot_msgs/msg/MotorState` becomes the header
`my_robot_msgs/msg/motor_state.h` and the C type
`my_robot_msgs__msg__MotorState`.

## Troubleshooting

The type is not found at build time. Confirm the package folder is under
`extra_packages/` and that you deleted `libmicroros/`, then clean and rebuild.

The agent shows the topic but messages are garbage or dropped. The host package
and the baked-in definition differ. Rebuild both from the same `.msg`.

The build is stuck on a stale library. Delete `libmicroros/` and `Debug/`, then
clean and build again. The pre-build step only regenerates when the library is
missing.
