# Custom interfaces

The board only knows the types built into its micro-ROS library. To add your own, make an interface package, copy it into the project, and rebuild the library.

The examples use the project `~/ros2_ws/firmware/my_robot`.

## 1. Make the interface package

```bash
cd ~/ros2_ws/src && ros2 pkg create --build-type ament_cmake my_interfaces && mkdir my_interfaces/msg my_interfaces/srv
```

Add `my_interfaces/msg/Num.msg`:

```text
int64 num
```

Add `my_interfaces/srv/AddThreeInts.srv`:

```text
int64 a
int64 b
int64 c
---
int64 sum
```

Add to `CMakeLists.txt`, before `ament_package()`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Num.msg"
  "srv/AddThreeInts.srv"
)
```

Add to `package.xml`:

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Build it on the host too, so `ros2` commands know the type.

```bash
cd ~/ros2_ws && colcon build --packages-select my_interfaces && source install/setup.bash
```

## 2. Copy it into the project

Copy it to `micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages` folder, so the library generator sees it.

```bash
cp -r ~/ros2_ws/src/my_interfaces ~/ros2_ws/firmware/my_robot/micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/extra_packages/
```

## 3. Rebuild the library

The library only rebuilds when its folder is gone, so delete it.

```bash
rm -rf ~/ros2_ws/firmware/my_robot/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
```

Then **Project > Clean** and **Build** in STM32CubeIDE.

## 4. Use it

`my_interfaces/msg/Num` becomes the header `<my_interfaces/msg/num.h>` and the C type `my_interfaces__msg__Num`.

```c
#include <my_interfaces/msg/num.h>

my_interfaces__msg__Num num_msg;

rclc_publisher_init_default(&publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(my_interfaces, msg, Num), "num");
```

For the service, follow [SERVICES.md](SERVICES.md)
