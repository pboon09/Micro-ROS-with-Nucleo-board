# Setup from scratch

How to build the template yourself, starting from a blank CubeMX project. The result matches [`uros_example/`](../uros_example). If you only want a working project, use [`create_project.sh`](../create_project.sh) from the [README](../README.md) instead.

## Contents

- [1. Install the tools](#1-install-the-tools)
- [2. Create the project](#2-create-the-project)
- [3. Configure the .ioc](#3-configure-the-ioc)
- [4. Add micro_ros_stm32cubemx_utils](#4-add-micro_ros_stm32cubemx_utils)
- [5. Project settings](#5-project-settings)
- [6. Add the micro-ROS code](#6-add-the-micro-ros-code)
- [7. Build and run](#7-build-and-run)
- [References](#references)

## 1. Install the tools

Install ROS 2, then follow [INSTALL.md](INSTALL.md) for the micro-ROS agent, Docker and STM32CubeIDE.

## 2. Create the project

In STM32CubeIDE, go to **File > New > STM32 Project**, open the **Board Selector** tab and pick your board, for example `NUCLEO-G474RE`. Save the project under `~/ros2_ws/firmware/`. Answer **Yes** to both prompts.

Keep colcon from treating the firmware as a ROS package:

```bash
touch ~/ros2_ws/firmware/COLCON_IGNORE
```

## 3. Configure the .ioc

The agent talks to the board over the UART wired to the ST-Link USB port. On the Nucleo-G474RE that is **LPUART1** ([UM2505](https://www.st.com/resource/en/user_manual/um2505-stm32g4-nucleo64-boards-mb1367-stmicroelectronics.pdf)). On another board, check its user manual.

| Peripheral | Setting |
|---|---|
| RCC | High Speed Clock: **Crystal/Ceramic Resonator** |
| SYS | Timebase Source: **TIM1** |
| IWDG | Activated, prescaler `4`, window `4095`, reload `4095` |
| TIM2 | Activated, Clock Source **Internal Clock**, Prescaler `169`, Counter Period `999`, NVIC global interrupt on |
| LPUART1 | **Asynchronous**, `2000000` baud, NVIC global interrupt on |
| LPUART1 DMA | RX: **Circular**, **Very High**. TX: **Very High** |
| FREERTOS | Interface **CMSIS_V2**, defaultTask stack `3000` words, `TOTAL_HEAP_SIZE` `3072` |

TIM2 with those values gives a 1 kHz interrupt on the 170 MHz clock. The IWDG screenshot shows reload `2499`, but use `4095`.

![RCC](../picture/rcc.png)
![SYS](../picture/sys.png)
![IWDG](../picture/iwdg.png)
![set uart mode](../picture/uart1.png)
![set baud rate](../picture/uart2.png)
![rx dma](../picture/uart3.png)
![tx dma](../picture/uart4.png)
![cmsis](../picture/freertos1.png)
![task and queue](../picture/freertos2.png)
![edit task](../picture/freertos3.png)

Click the gear icon to generate the code.

## 4. Add micro_ros_stm32cubemx_utils

Run these from the project folder. Use the branch that matches your ROS 2.

```bash
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
```

Copy the extra sources, then keep only the DMA transport:

```bash
cp -r micro_ros_stm32cubemx_utils/extra_sources/* Core/Src/
find Core/Src/microros_transports -type f ! -name dma_transport.c -delete
```

## 5. Project settings

Open **Project > Properties > C/C++ Build > Settings**.

**Build Steps > Pre-build steps**. Replace `humble` with `jazzy` twice if you use Jazzy.

```text
docker pull microros/micro_ros_static_library_builder:humble && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble
```

| Tab | Field | Value |
|---|---|---|
| MCU/MPU GCC Compiler > Include paths | Include paths | `../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include` |
| MCU/MPU GCC Linker > Libraries | Libraries (-l) | `microros` |
| MCU/MPU GCC Linker > Libraries | Library search path (-L) | `../micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros` |

Build once. The first build compiles the micro-ROS library in Docker and takes a few minutes.

## 6. Add the micro-ROS code

All micro-ROS code goes in [`Core/Src/app_freertos.c`](../uros_example/Core/Src/app_freertos.c), inside the `USER CODE` blocks so it survives code generation. The finished file is [`uros_example/Core/Src/app_freertos.c`](../uros_example/Core/Src/app_freertos.c).

### main.c

`app_freertos.c` defines its own `HAL_TIM_PeriodElapsedCallback`, so mark the one in [`main.c`](../uros_example/Core/Src/main.c) as `__weak`. This line is outside the `USER CODE` blocks, so check it again after every code generation.

```c
__weak void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
```

### Includes

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

### Macro and variables

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

### Function prototypes

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

### Init

Start TIM2 before the scheduler runs.

```c
/* USER CODE BEGIN Init */
HAL_TIM_Base_Start_IT(&htim2);
/* USER CODE END Init */
```

### Default task

Sets up the transport and allocators, creates the node, publisher, subscriber and timer, then spins forever.

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
RCLSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 127));

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

### Callbacks

The timer publishes and feeds the watchdog. The subscription stores `/cmd_vel`. TIM1 drives the HAL tick, and TIM2 gives you a 1 kHz hook.

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

## 7. Build and run

Build, then **Run** to flash. If you get a `_gettimeofday` warning, see [TIPS_AND_TROUBLESHOOTING.md](TIPS_AND_TROUBLESHOOTING.md#troubleshooting).

Start the agent, then press the reset button on the board.

```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 2000000 --dev /dev/ttyACM0
```

Check from another terminal. Expect `/uros_motor_node`, then `/robot_pos` and `/cmd_vel`.

```bash
ros2 node list && ros2 topic list
```

Next: [CMSIS_DSP.md](CMSIS_DSP.md) for math, [TIPS_AND_TROUBLESHOOTING.md](TIPS_AND_TROUBLESHOOTING.md) for git setup.

## References

- [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup)
- [micro_ros_stm32cubemx_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
- [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino)

Thanks to these videos:

- [How to Set Up Micro-ROS on Any STM32 Microcontroller by Robotics in a Nutshell](https://www.youtube.com/watch?v=xbWaHARjSmk)
- [Micro-ROS STM32 with STM32CubeIDE by Sokheng Din](https://www.youtube.com/watch?v=bn-P3fxtTF4)
