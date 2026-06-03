# Micro-ROS STM32 Nucleo Template

A ready-to-build **micro-ROS** firmware template for the **STM32 Nucleo-G474RE**
board, using FreeRTOS (CMSIS-V2) and an LPUART with DMA serial transport. Clone
it, rename it to your own project, build, and flash, and you have a working
micro-ROS node without any blank-project setup.

Out of the box the firmware runs a node called `uros_motor_node` with these
behaviors.
- It **publishes** `std_msgs/Float64MultiArray` on `robot_pos`.
- It **subscribes** to `geometry_msgs/Twist` on `cmd_vel`.
- It uses `ROS_DOMAIN_ID = 127` and an LPUART serial transport at **2000000 baud**.
- It blinks the user LED (LD2) and kicks an independent watchdog (IWDG).

To build from a blank STM32CubeMX project instead, or to understand how every
piece is wired, see [docs/SETUP_FROM_SCRATCH.md](docs/SETUP_FROM_SCRATCH.md).

---

## Prerequisites

- Ubuntu 22.04 with **ROS 2 Humble** and the **micro-ROS agent** installed
- **STM32CubeIDE**
- **Docker**, runnable without `sudo`, because the build's pre-build step calls
  Docker. Grant access once with the commands below.
  ```bash
  sudo usermod -aG docker $USER
  newgrp docker            # or log out and back in
  docker run hello-world   # must succeed without sudo
  ```
- A Nucleo-G474RE board and a USB cable

The ROS 2 and micro-ROS installation commands are in
[docs/SETUP_FROM_SCRATCH.md](docs/SETUP_FROM_SCRATCH.md), Steps 1 and 2.

---

## Use the template

### 1. Get the code and name your project
```bash
git clone https://github.com/pboon09/Micro-ROS-with-Nucleo-board.git my_robot_fw
cd my_robot_fw
./rename_project.sh my_robot      # pick any name: letters, digits, underscores
```
`rename_project.sh` renames the project folder, the `.ioc` and `.launch` files,
and every internal reference, then clears stale build output. It is safe to
re-run if you want to rename again later.

### 2. Open in STM32CubeIDE
Open **File** then **Open Projects from File System...** and select the
`my_robot/` folder.

### 3. Build
Right-click the project and choose **Build**. The first build runs a pre-build
step that pulls the `microros/micro_ros_static_library_builder:humble` Docker
image and compiles the micro-ROS static library into
`my_robot/micro_ros_stm32cubemx_utils/`. This takes a few minutes the first time.

### 4. Flash
Click **Run** or **Debug** to upload to the board.

### 5. Run the micro-ROS agent
On the host, start the agent against the board's serial port. Match the baud rate
to the firmware, which is 2000000 by default.
```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 --dev /dev/ttyACM0
```
Then confirm the node and topics are visible.
```bash
ros2 topic list      # expect /robot_pos and /cmd_vel
ros2 node list       # expect /uros_motor_node
```
If nothing appears, press the board's reset button.

---

## Customize it

| To change | Edit |
|---|---|
| Node name, topics, message types, domain ID | the micro-ROS setup in [`uros_example/Core/Src/app_freertos.c`](uros_example/Core/Src/app_freertos.c) (`StartDefaultTask`) |
| Pre-scheduler init (timers, robot config) | `USER CODE BEGIN Init` in [`app_freertos.c`](uros_example/Core/Src/app_freertos.c) (`MX_FREERTOS_Init`) |
| Serial baud rate | the LPUART config in the `.ioc` and the agent's `-b` flag |
| Add custom message or service types | [docs/CUSTOM-INTERFACES.md](docs/CUSTOM-INTERFACES.md) |
| Add CMSIS-DSP math routines | [docs/CMSIS-DSP.md](docs/CMSIS-DSP.md) |

---

## Troubleshooting

- **Agent connects but `ros2 topic list` shows nothing.** Check
  `ROS_LOCALHOST_ONLY`. If it is `1`, run `export ROS_LOCALHOST_ONLY=0`. Make
  sure ROS 2 and the firmware share the same domain ID, which is 127.
- **Pre-build fails with a Docker permission error.** Your user cannot reach the
  Docker daemon. Redo the `usermod -aG docker` step above, or run `sudo chmod 666
  /var/run/docker.sock` for a one-off session.
- **The build prints `warning: _gettimeofday is not implemented`.** Add the
  `_gettimeofday_r` stub from
  [docs/TIPS-AND-TROUBLESHOOTING.md](docs/TIPS-AND-TROUBLESHOOTING.md).

Full explanations and more fixes are in
[docs/TIPS-AND-TROUBLESHOOTING.md](docs/TIPS-AND-TROUBLESHOOTING.md).

---

## Documentation

| Guide | Read it for |
|---|---|
| [docs/SETUP_FROM_SCRATCH.md](docs/SETUP_FROM_SCRATCH.md) | Building the whole project from a blank STM32CubeMX project, step by step |
| [docs/CMSIS-DSP.md](docs/CMSIS-DSP.md) | Adding ARM's CMSIS-DSP math library (optional) |
| [docs/CUSTOM-INTERFACES.md](docs/CUSTOM-INTERFACES.md) | Creating custom messages and services and baking them into `libmicroros` |
| [docs/TIPS-AND-TROUBLESHOOTING.md](docs/TIPS-AND-TROUBLESHOOTING.md) | Git ignore rules, workspace layout, Docker, and common runtime fixes |

---

## Repository layout

```
.
├── README.md                         # this file, how to use the template
├── rename_project.sh                 # one-shot project renamer
├── docs/
│   ├── SETUP_FROM_SCRATCH.md         # full build-it-yourself walkthrough
│   ├── CMSIS-DSP.md                  # optional ARM DSP math library
│   ├── CUSTOM-INTERFACES.md          # custom messages and services
│   └── TIPS-AND-TROUBLESHOOTING.md   # git, workspace, common fixes
├── CMSIS/                            # CMSIS packs and install manual (optional DSP use)
├── picture/                          # screenshots referenced by the from-scratch guide
└── uros_example/                     # the STM32CubeIDE project (renamed by the script)
```

## Credits

Built on the official [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble).
Additional references and acknowledgements are listed at the end of
[docs/SETUP_FROM_SCRATCH.md](docs/SETUP_FROM_SCRATCH.md).

## Feedback
If you have any feedback, please open an issue on the
[GitHub repository](https://github.com/pboon09/Micro-ROS-with-Nucleo-board).
