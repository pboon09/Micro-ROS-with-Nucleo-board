<div align="center">

# Micro-ROS-with-Nucleo-board

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy_%7C_Humble-22314E?logo=ros&logoColor=white)
![Board](https://img.shields.io/badge/board-NUCLEO--G474RE-03234B?logo=stmicroelectronics&logoColor=white)
![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-8BC34A)
![IDE](https://img.shields.io/badge/IDE-STM32CubeIDE-03234B?logo=stmicroelectronics&logoColor=white)
![Build](https://img.shields.io/badge/build-Docker-2496ED?logo=docker&logoColor=white)

</div>

## 🤖 About

A script that creates a micro-ROS project for STM32CubeIDE, ready to build and flash.

The template runs the node `uros_motor_node` on FreeRTOS. It publishes `/robot_pos`, subscribes to `/cmd_vel`, and talks to the agent through the ST-Link USB port at 2000000 baud.

The example uses ROS domain ID `127`, set in [`app_freertos.c`](uros_example/Core/Src/app_freertos.c#L194). The host must use the same domain ID, so check it before you run anything.

## 🚀 Getting Started

### 1. Install the tools

With ROS 2 installed, get the micro-ROS agent, Docker and STM32CubeIDE from [INSTALL.md](docs/INSTALL.md).

### 2. Clone

```bash
git clone https://github.com/pboon09/Micro-ROS-with-Nucleo-board.git
```

### 3. Create a project

`create_project.sh` only works for the NUCLEO-G474RE. For any other board, follow [SETUP_FROM_SCRATCH.md](docs/SETUP_FROM_SCRATCH.md).

```bash
. Micro-ROS-with-Nucleo-board/create_project.sh <workspace> <project_name> <humble|jazzy>
```

For example, this creates `~/ros2_ws/firmware/my_robot`:

```bash
~/Micro-ROS-with-Nucleo-board/create_project.sh ~/ros2_ws my_robot jazzy
```

### 4. Build and flash

Open it in STM32CubeIDE with **File > Open Projects from File System...**, then **Build** and **Run**. The first build takes a few minutes because it compiles the micro-ROS library in Docker.

### 5. Run the agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 --dev /dev/ttyACM0
```
Expect `/robot_pos` and `/cmd_vel`.

```bash
ros2 topic list
```

Your code goes in [`Core/Src/app_freertos.c`](uros_example/Core/Src/app_freertos.c).

## 📚 Guides

| Guide | For |
|---|---|
| [INSTALL.md](docs/INSTALL.md) | Installing the agent, Docker and STM32CubeIDE |
| [SERVICES.md](docs/SERVICES.md) | A service server or client on the board |
| [CUSTOM_INTERFACES.md](docs/CUSTOM_INTERFACES.md) | Your own `.msg` and `.srv` types |
| [CMSIS_DSP.md](docs/CMSIS_DSP.md) | ARM's math library |
| [TIPS_AND_TROUBLESHOOTING.md](docs/TIPS_AND_TROUBLESHOOTING.md) | Git setup and common problems |
| [SETUP_FROM_SCRATCH.md](docs/SETUP_FROM_SCRATCH.md) | Building the same project from a blank CubeMX project |

## 🙏 Credits

Built on [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils).
