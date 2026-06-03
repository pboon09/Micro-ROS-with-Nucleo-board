# Setting up Micro-ROS on a Nucleo board from scratch

This guide walks through creating a Micro-ROS STM32 project from a blank
STM32CubeMX project. It covers installing the tooling, configuring the `.ioc`,
wiring in the micro-ROS utilities, and writing the application code.

## Table of Contents
- [Requirement](#requirement)
- [About choosing UART](#about-choosing-uart)
- [Step 1 - Install ROS 2](#step-1---install-ros-2)
- [Step 2 - Install Micro-ROS](#step-2---install-micro-ros)
- [Step 3 - Download STM32CubeIDE](#step-3---download-stm32cubeide)
- [Step 4 - Download Docker](#step-4---download-docker)
- [Step 5 - IOC Setup](#step-5---ioc-setup)
- [Step 6 - Clone micro\_ros\_stm32cubemx\_utils](#step-6---clone-micro_ros_stm32cubemx_utils)
- [Step 7 - CMSIS-DSP](#step-7---cmsis-dsp)
- [Step 8 - About Git](#step-8---about-git)
- [Step 9 - Setting Project's Properties](#step-9---setting-projects-properties)
- [Step 10 - Add Micro-ROS Code](#step-10---add-micro-ros-code)
- [Step 11 - Running Micro-ROS](#step-11---running-micro-ros)
- [Related guides](#related-guides)
- [Documentation](#documentation)

Focused topics live in their own guides. See [CMSIS-DSP](CMSIS-DSP.md),
[Custom interfaces](CUSTOM-INTERFACES.md), and
[Tips and troubleshooting](TIPS-AND-TROUBLESHOOTING.md).

## Requirement
Before starting, make sure you have the following.
- A computer with Ubuntu 22.04.4 LTS
- ROS2 installed
- Micro-ROS installed
- STM32CubeIDE installed
- Docker installed
- A USB cable to connect the Nucleo board to your computer
- Nucleo Board

## About choosing UART
You need the UART that is wired to the ST-Link Virtual COM Port (VCP). Only one
UART on the board reaches the ST-Link USB connector, and that is the one the
agent talks to over the serial cable. Check your board user manual to find it.
For our example board, the Nucleo-G474RE
([UM2505](https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf)),
it is **LPUART1**.

## Step 1 - Install ROS 2
To get started, you'll need to install ROS 2 on your system. For this guide, we are using the ROS 2 Humble distribution. Follow the official ROS 2 installation guide for Ubuntu by clicking [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

## Step 2 - Install Micro-ROS
To set up Micro-ROS, visit the [official Micro-ROS tutorial](https://micro.ros.org/docs/tutorials/core/first_application_linux/) and follow the guide to get started.
```bash
mkdir microros_ws
cd microros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_firmware_ws.sh host
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
To verify, please run the following.
```bash
ros2 run micro_ros_demos_rclc int32_publisher
ros2 topic list
```

## Step 3 - Download STM32CubeIDE
Visit this youtube video [Install STM32CUBEIDE on Linux(UBUNTU) by Embedded Icon
](https://youtu.be/j3P2rsB_-BY?si=XfH9ioiwhtgmfos6) and follow the guide to install the STM32CubeIDE.

## Step 4 - Download Docker
Visit this youtube video [How to Install Docker on Ubuntu: A Step-By-Step Guide
 by vCloudBitsBytes](https://www.youtube.com/watch?v=cqbh-RneBlk) and follow the guide to install the Docker.

After installing Docker, allow your user to run Docker without `sudo` so the
STM32CubeIDE pre-build step works.
```bash
sudo usermod -aG docker $USER
newgrp docker   # or log out and back in
docker run hello-world   # should succeed without sudo
```

## Step 5 - IOC Setup
### 1. Create STM32 Project
`File` -> `New` -> `STM32 Project` -> `Board Selector`

Type your Commercial Part Number for example NUCLEO-F411RE and click `Next`

Complete the project name then click `Finish`

`Initialize all peripherals with their default Mode?` -> `Yes`

`Device Configuration Tool ...` -> `Yes`

### 2. Setting IOC
- System Core
    - RCC
        - `HSE: Cystal/Ceramic Resonator`
    - SYS
        - `Timebase Source: TIM1`
    - IWDG (Enable if you want auto-reconnect)
        - `Activated`
        - `down-counter reload 2499`

![RCC](../picture/rcc.png)

![SYS](../picture/sys.png)

![IWDG](../picture/iwdg.png)

- Timers (optional, only if you want a hardware timer like this template's TIM2)
    - TIM2 `Activated` with `Clock Source: Internal Clock`
        - In NVIC Settings tab, enable `global interrupt`
        - Set `Prescaler` and `Counter Period` for the rate you want. This
          template uses `Prescaler 169` and `Counter Period 999` on a 170 MHz
          clock, which gives a 1 kHz (1 ms) update interrupt. It is started in
          `app_freertos.c` (see
          [Step 10.5](#105-pre-scheduler-init-user-code-begin-init)).

- Connectivity
    - LPUART (Choose preferable baudrate) `Asynchronous`
        - In DMA Settings tab, click Add
            - Add RX - `Mode: Circular` `Priority: Very High`
            - Add TX - `Priority: Very High`
        - In NVIC Settings tab, click enable `global interrupt`

![set uart mode](../picture/uart1.png)
![set baud rate](../picture/uart2.png)
![rx dma](../picture/uart3.png)
![tx dma](../picture/uart4.png)

- Middleware
    - FREERTOS `CMSIS_V2`
        - Double click `defaultTask`
            - `Stack Size (Words): 3000`
        - Make sure the micro-ROS task has more than 10 kB of stack (1 Word = 4 Bytes)

![cmsis](../picture/freertos1.png)
![task and queue](../picture/freertos2.png)
![edit task](../picture/freertos3.png)

Click `Device Configuration Tool Code Generation` or `Gear Icon`

## Step 6 - Clone micro_ros_stm32cubemx_utils
Go to the your project folder in workspace, and then open terminal.
```bash
git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
cd micro_ros_stm32cubemx_utils
git checkout humble
git branch
```
Next, copy the content from the `extra_sources` folder and paste it into the `core -> src` directory.

In the `microros_transport` directory, delete all files except for `dma_transport.c`.

## Step 7 - CMSIS-DSP
If you want ARM's optimized math library (filters, matrices, transforms) on the
Cortex-M4F, see the dedicated [CMSIS-DSP guide](CMSIS-DSP.md). It covers the
installable packs in the [`CMSIS/`](../CMSIS) folder, the library already
vendored in the template, and how to use it in code. CMSIS-DSP is optional, so
skip this step if you do not need it.

## Step 8 - About Git
Keep regenerable build output (`Debug/`, `Release/`, the generated `libmicroros/`)
out of version control. A ready-made [.gitignore](../uros_example/.gitignore) is
already included. For the exact ignore rules, how to untrack or delete files that
were committed before, and how to vendor `micro_ros_stm32cubemx_utils` into your
own repo, see
[Tips and troubleshooting](TIPS-AND-TROUBLESHOOTING.md#git-ignore-rules).

## Step 9 - Setting Project's Properties

The template project already has all of the settings below configured. This step
is for reproducing them on a fresh project.

- Navigate to `Project -> Settings -> C/C++ Build -> Settings -> Build Steps Tab`
    - In `Pre-build steps` add the command below. It uses no `sudo` and works for
      any project name because it expands the `${ProjName}` build variable.
    ```bash
    docker pull microros/micro_ros_static_library_builder:humble && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble
    ```
    This requires that your user can run Docker without `sudo` (see Step 4).
    Never commit your sudo password into the build configuration.

- Navigate to `Project -> Settings -> C/C++ Build -> Settings -> Tool Settings Tab -> MCU/MPU GCC Compiler -> Include paths`
```bash
../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
```

- Navigate to `Project -> Settings -> C/C++ Build -> Settings -> MCU/MPU GCC Linker -> Libraries`
    - In Libraries (-l)
    ```bash
    microros
    ```
    - in Library search path (-L)
    ```bash
    ../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
    ```

Finally, right-click the project and select `Build`. At this point it will take a while, and the build should complete without any errors.

## Step 10 - Add Micro-ROS Code
With FreeRTOS file separation enabled, STM32CubeMX generates the default task and
`MX_FREERTOS_Init()` in
[`Core/Src/app_freertos.c`](../uros_example/Core/Src/app_freertos.c).
**Put all micro-ROS code there, not in `main.c`.** There are three reasons.
`main.c` holds only the generated hardware bring-up, so keeping the application
in `app_freertos.c` matches CubeMX's intended layout. It also keeps `main.c`
clean across `.ioc` regenerations. Most important, the executor spin has to live
inside the FreeRTOS task, which CubeMX generates in `app_freertos.c`. Each
snippet below names the exact `USER CODE` block in `app_freertos.c` it belongs
to.

### 10.1 Make the main.c timer callback weak
`app_freertos.c` defines its own `HAL_TIM_PeriodElapsedCallback` (step 10.7
below). The HAL also generates a default `HAL_TIM_PeriodElapsedCallback` in
`main.c`. Mark that one `__weak` so the linker uses your strong version instead
of failing on a duplicate symbol.

```c
/* Core/Src/main.c */
__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* ... generated body ... */
}
```

### 10.2 Include libraries (USER CODE BEGIN Includes)
The task uses the LPUART, IWDG, TIM, GPIO and DMA handles. In the single-file
layout those are visible from `main.c`. In `app_freertos.c` you must include
their headers explicitly, in addition to the micro-ROS headers.

```c
/* USER CODE BEGIN Includes */
#include "dma.h"
#include "iwdg.h"
#include "usart.h"
#include "tim.h"
#include "gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/multi_array_dimension.h>
#include <std_msgs/msg/multi_array_layout.h>
#include <std_msgs/msg/float64_multi_array.h>
#include <geometry_msgs/msg/twist.h>
/* USER CODE END Includes */
```

### 10.3 Macro and variables (USER CODE BEGIN PM and Variables)
In `app_freertos.c` the variable block is named `USER CODE BEGIN Variables` and
the prototype block is named `USER CODE BEGIN FunctionPrototypes`. These names
differ from `main.c`, which uses `PV` and `PFP`.

```c
/* USER CODE BEGIN PM */
#define RCLSOFTCHECK(fn) if (fn!= RCL_RET_OK){};
/* USER CODE END PM */
```
```c
/* USER CODE BEGIN Variables */
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_init_options_t init_options;
rclc_executor_t executor;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;

std_msgs__msg__Float64MultiArray pub_msg;
geometry_msgs__msg__Twist sub_msg;

rcl_timer_t timer;
const unsigned int timer_period = RCL_MS_TO_NS(10);
const int timeout_ms = 1000;

float linear_x, linear_y, linear_z, angular_x, angular_y, angular_z;
/* USER CODE END Variables */
```

### 10.4 Function prototypes (USER CODE BEGIN FunctionPrototypes)
```c
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport,
        const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf,
        size_t len, int timeout, uint8_t *err);

void* microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void* microros_reallocate(void *pointer, size_t size, void *state);
void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element,
        void *state);

void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void subscription_callback(const void *msgin);
/* USER CODE END FunctionPrototypes */
```

### 10.5 Pre-scheduler init (USER CODE BEGIN Init)
Start anything the application needs before the scheduler runs. This block lives
inside `MX_FREERTOS_Init`. The template starts the TIM2 timer with its update
interrupt. Enable TIM2 in the `.ioc` first (see [Step 5](#step-5---ioc-setup)).

```c
/* USER CODE BEGIN Init */
// Add init here eg. Robot Config
HAL_TIM_Base_Start_IT(&htim2);
/* USER CODE END Init */
```

### 10.6 micro-ROS bring-up and spin (USER CODE BEGIN StartDefaultTask)
The body of the generated `StartDefaultTask` sets up the transport and
allocators, creates the node, publisher, subscriber, timer and executor, then
spins. The call to `rclc_executor_spin` never returns.

```c
/* USER CODE BEGIN StartDefaultTask */
rmw_uros_set_custom_transport(true, (void*) &hlpuart1,
        cubemx_transport_open, cubemx_transport_close,
        cubemx_transport_write, cubemx_transport_read);

rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
freeRTOS_allocator.allocate = microros_allocate;
freeRTOS_allocator.deallocate = microros_deallocate;
freeRTOS_allocator.reallocate = microros_reallocate;
freeRTOS_allocator.zero_allocate = microros_zero_allocate;

if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
    printf("Error on default allocators (line %d)\n", __LINE__);
}
allocator = rcl_get_default_allocator();

// create init
init_options = rcl_get_zero_initialized_init_options();
RCLSOFTCHECK(rcl_init_options_init(&init_options, allocator));
RCLSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 99));

// create support
rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

// create node
rclc_node_init_default(&node, "uros_motor_node", "", &support);

pub_msg.layout.dim.capacity = 1;
pub_msg.layout.dim.size = 1;
pub_msg.layout.dim.data = malloc(sizeof(std_msgs__msg__MultiArrayDimension) * 1);

pub_msg.layout.dim.data[0].label.data = malloc(10);
pub_msg.layout.dim.data[0].label.capacity = 10;
pub_msg.layout.dim.data[0].label.size = strlen("motor_data");
strcpy(pub_msg.layout.dim.data[0].label.data, "motor_data");

pub_msg.layout.data_offset = 0;

pub_msg.data.capacity = 6;
pub_msg.data.size = 6;
pub_msg.data.data = malloc(6 * sizeof(double));

// Create publisher
rclc_publisher_init_default(&publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64MultiArray),
        "robot_pos");

// Create subscriber
rclc_subscription_init_best_effort(&subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

// create timer
rclc_timer_init_default(&timer, &support, timer_period, timer_callback);

// create executor (handles = subscriptions + timers + clients + services)
executor = rclc_executor_get_zero_initialized_executor();
rclc_executor_init(&executor, &support.context, 2, &allocator);
rclc_executor_add_timer(&executor, &timer);
rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
        &subscription_callback, ON_NEW_DATA);
rclc_executor_spin(&executor);

/* Infinite loop */
for (;;) {
    osDelay(1);
}
/* USER CODE END StartDefaultTask */
```

### 10.7 Callbacks (USER CODE BEGIN Application)
The timer and subscription callbacks plus the timer-interrupt callback go in the
application section. The `HAL_TIM_PeriodElapsedCallback` here is the strong
definition that overrides the weak one from
[step 10.1](#101-make-the-mainc-timer-callback-weak). It forwards TIM1 to
`HAL_IncTick()` for the HAL time base and gives you a TIM2 hook that runs at the
1 kHz rate set in Step 5.

```c
/* USER CODE BEGIN Application */
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    static uint8_t cnt = 0;

    if (timer != NULL) {
        // Sync micro-ROS session
        rmw_uros_sync_session(timeout_ms);

        // Toggle LED every 50 cycles (approximately every 0.5 seconds)
        if (cnt == 0)
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        cnt = (cnt + 1) % 50;

        // Prepare and publish multi-array message with motor data
        if (pub_msg.data.data != NULL) {
            pub_msg.data.data[0] = linear_x;
            pub_msg.data.data[1] = linear_y;
            pub_msg.data.data[2] = linear_z;
            pub_msg.data.data[3] = angular_x;
            pub_msg.data.data[4] = angular_y;
            pub_msg.data.data[5] = angular_z;

            // Publish the multi-array message
            RCLSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
        }

        // Reinitialize watchdog timer
        HAL_IWDG_Init(&hiwdg);
    }
}

void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *twist_msg =
            (const geometry_msgs__msg__Twist*) msgin;

    linear_x = twist_msg->linear.x;
    linear_y = twist_msg->linear.y;
    linear_z = twist_msg->linear.z;

    angular_x = twist_msg->angular.x;
    angular_y = twist_msg->angular.y;
    angular_z = twist_msg->angular.z;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }
    /* USER CODE BEGIN Callback 1 */
    if (htim->Instance == TIM2) {
        // Run at 1k Hz
    }
    /* USER CODE END Callback 1 */
}
/* USER CODE END Application */
```

Then build and click `Run` to upload to the board. If you get
`warning: _gettimeofday is not implemented and will always fail`, add the
`_gettimeofday_r` stub from
[Tips and troubleshooting](TIPS-AND-TROUBLESHOOTING.md#the-_gettimeofday-warning).

## Step 11 - Running Micro-ROS
If the build's Docker pre-build step hit a permission error, see
[Docker permissions](TIPS-AND-TROUBLESHOOTING.md#docker-permissions).

Run the Micro-ROS Agent and change the baud rate to match your configuration.
```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 --dev /dev/ttyACM0
```

Check if the agent is running successfully.
```bash
ros2 topic list
```

If you see `/uros_motor_node` in the list, congratulations. You have
successfully installed Micro-ROS on the Nucleo board.

If nothing appears, press the reset button.

## Related guides
These topics have their own focused guides.

| Guide | Covers |
|---|---|
| [CMSIS-DSP](CMSIS-DSP.md) | Installing the CMSIS packs, the vendored DSP library, using it in code |
| [Custom interfaces](CUSTOM-INTERFACES.md) | Creating custom messages and services and baking them into `libmicroros` |
| [Tips and troubleshooting](TIPS-AND-TROUBLESHOOTING.md) | Git ignore rules, workspace organization (`COLCON_IGNORE`), Docker permissions, `ROS_LOCALHOST_ONLY`, the `_gettimeofday` warning |

## Documentation
### GitHub Repositories
- [micro_ros_setup - Humble](https://github.com/micro-ROS/micro_ros_setup/tree/humble)
- [micro_ros_stm32cubemx_utils - Humble](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble)
- [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/humble)

### Installations
- [Install STM32CUBEIDE on Linux (Ubuntu) by Embedded Icon](https://youtu.be/j3P2rsB_-BY?si=XfH9ioiwhtgmfos6)
- [How to Install Docker on Ubuntu: A Step-By-Step Guide by vCloudBitsBytes](https://www.youtube.com/watch?v=cqbh-RneBlk)

### Special Thanks
Thanks to the following videos that have helped me reach this point.

- [How to Set Up Micro-ROS on Any STM32 Microcontroller by Robotics in a Nutshell](https://www.youtube.com/watch?v=xbWaHARjSmk)
- [Micro-ROS STM32 with STM32CubeIDE by Sokheng Din](https://www.youtube.com/watch?v=bn-P3fxtTF4)
