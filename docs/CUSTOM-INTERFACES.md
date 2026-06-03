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
In short, create one package that holds both your messages and your services.

```bash
cd ~/microros_ws/src           # or your ROS 2 workspace
ros2 pkg create --build-type ament_cmake my_robot_msgs
mkdir my_robot_msgs/msg
mkdir my_robot_msgs/srv
```

A message has a single section. Write the fields in
`my_robot_msgs/msg/MotorState.msg`.
```
float64[6] data
uint32     stamp_ms
```

A service has a request section and a response section split by `---`. Write
`my_robot_msgs/srv/SetMode.srv`.
```
uint8 mode
---
bool success
```

Register both in `my_robot_msgs/CMakeLists.txt`.
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "srv/SetMode.srv"
)
```
Add the matching lines to `package.xml`.
```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Build and source it so the agent can use the types.
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
Docker pre-build step rebuilds it with your types included.

```bash
rm -rf uros_example/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/
```

Now clean the project first, then build. In STM32CubeIDE choose **Project** then
**Clean**, then build the project. The pre-build step regenerates `libmicroros`
with `my_robot_msgs` compiled in.

## 3. Use a custom message on the device

The C names are the snake_case form of the ROS type. The message
`my_robot_msgs/msg/MotorState` becomes the header
`my_robot_msgs/msg/motor_state.h` and the C type
`my_robot_msgs__msg__MotorState`.

```c
#include <my_robot_msgs/msg/motor_state.h>

my_robot_msgs__msg__MotorState pub_msg;

rclc_publisher_init_default(&publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(my_robot_msgs, msg, MotorState),
    "motor_state");
```

## 4. Use a custom service on the device

A service uses `ROSIDL_GET_SRV_TYPE_SUPPORT` and two generated types, one for the
request and one for the response. The service
`my_robot_msgs/srv/SetMode` becomes the header `my_robot_msgs/srv/set_mode.h`
with the types `my_robot_msgs__srv__SetMode_Request` and
`my_robot_msgs__srv__SetMode_Response`.

```c
#include <my_robot_msgs/srv/set_mode.h>

rcl_service_t service;
my_robot_msgs__srv__SetMode_Request  set_mode_req;
my_robot_msgs__srv__SetMode_Response set_mode_res;

void set_mode_callback(const void *req_in, void *res_out) {
    const my_robot_msgs__srv__SetMode_Request *request =
            (const my_robot_msgs__srv__SetMode_Request *) req_in;
    my_robot_msgs__srv__SetMode_Response *response =
            (my_robot_msgs__srv__SetMode_Response *) res_out;

    // read request->mode and fill response->success
    response->success = true;
}
```

Create the service after the node, then add it to the executor with its request
and response buffers.

```c
rclc_service_init_default(&service, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(my_robot_msgs, srv, SetMode), "set_mode");

rclc_executor_add_service(&executor, &service,
    &set_mode_req, &set_mode_res, set_mode_callback);
```

A service is one more executor handle. Raise the handle count in
`rclc_executor_init` to cover it. The template uses `2` for one timer and one
subscription, so adding this service makes it `3`.

```c
rclc_executor_init(&executor, &support.context, 3, &allocator);
```

## Troubleshooting

The type is not found at build time. Confirm the package folder is under
`extra_packages/` and that you deleted `libmicroros/`, then clean and rebuild.

The agent shows the topic or service but the data is garbage or dropped. The host
package and the baked-in definition differ. Rebuild both from the same `.msg` or
`.srv`.

The build is stuck on a stale library. Delete `libmicroros/` and `Debug/`, then
clean and build again. The pre-build step only regenerates when the library is
missing.
