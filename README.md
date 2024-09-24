# Micro-ROS-with-Nucleo-board
This project provides a complete guide to setting up Micro-ROS on a Nucleo board. It walks through the steps required to get started, from installing necessary tools to running your first Micro-ROS application.
## Requirement
Before starting, make sure you have the following:
- A computer with Ubuntu 22.04.4 LTS
- ROS2 installed
- Micro-ROS installed
- STM32CubeIDE installed
- Docker installed
- A USB cable to connect the Nucleo board to your computer
- Nucleo Board
## About choosing UART
To choose the correct UART port, you need to identify which port the microcontroller uses to connect to your computer. You can search for your specific board's datasheet by entering "Your Board Datasheet" on Google. For more detailed information, visit [STMicroelectronics' website](www.st.com) and refer to the user manual document.
## Step 1 - Install ROS 2
To get started, you'll need to install ROS 2 on your system. For this guide, we are using the ROS 2 Humble distribution.

### 1. Select the ROS 2 Version:
In this case, we are using the ROS 2 Humble distribution.

### 2. Follow the Installation Guide:
Follow the official ROS 2 installation guide for Ubuntu by clicking [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).

### 3. Verify the Installation:
- After installation, verify that ROS 2 is working correctly by running the following commands in separate terminal windows:
```bash
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_py listener
```
If both the talker and listener nodes are running successfully, you should see messages being published and received.

### 4. Update Your Bash Configuration:
- To ensure ROS 2 is sourced every time you open a new terminal, add the following line to your .bashrc file:
```bash
gedit ~/.bashrc
```
- Then add the line:
```bash
source /opt/ros/humble/setup.bash
```
## Step 2 - Install Micro-ROS

To set up Micro-ROS, follow these steps:

### 1. Follow the Micro-ROS Installation Guide
Visit the [official Micro-ROS tutorial](https://micro.ros.org/docs/tutorials/core/first_application_linux/) and follow the guide to get started.

### 2. Set Up the Workspace
- Create a new workspace
```bash
mkdir microros_ws
cd microros_ws
```
- Clone the Micro-ROS setup repository
```bash
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```
### 3. Install Dependencies
- Update dependencies using rosdep
```bash
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
```
### 4. Build Micro-ROS Tools
- Build Micro-ROS tools
```bash
colcon build
source install/local_setup.bash
```
### 5. Create and Build the Firmware
- Create the firmware workspace
```bash
ros2 run micro_ros_setup create_firmware_ws.sh host
```
- Build the firmware
```bash
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```
### 6. Set Up the Micro-ROS Agent
- Download Micro-ROS Agent packages
```bash
ros2 run micro_ros_setup create_agent_ws.sh
```
- Build the Micro-ROS Agent
```bash
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
### 7. Verify the Installation
- Run the Micro-ROS demo
```bash
ros2 run micro_ros_demos_rclc int32_publisher
```
- Check available ROS 2 topics
```bash
ros2 topic list
```
### 8. Update Your Bash Configuration:
- To ensure Micro-ROS is sourced every time you open a new terminal, add the following line to your .bashrc file:
```bash
gedit ~/.bashrc
```
- Then add the line:
```bash
source ../install/local_setup.bash
```
If the commands run successfully and you see the expected output, Micro-ROS has been installed correctly on your system.
## Step 3 - Download STM32CubeIDE
Visit this youtube video [Install STM32CUBEIDE on Linux(UBUNTU) by Embedded Icon
](https://youtu.be/j3P2rsB_-BY?si=XfH9ioiwhtgmfos6) and follow the guide to install the STM32CubeIDE.
## Step 4 - Download Docker
Visit this youtube video [How to Install Docker on Ubuntu: A Step-By-Step Guide
 by vCloudBitsBytes](https://www.youtube.com/watch?v=cqbh-RneBlk) and follow the guide to install the Docker.
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
- Connectivity
    - USART2
        - In DMA Settings tab, click Add
            - Add RX - `Mode: Circular` `Priority: Very High`
            - Add TX - `Priority: Very High`
        - In NVIC Settings tab, click enable `UART2 global interrupt`
- Middleware
    - FREERTOS
        - Double click `defaultTask`
            - `Stack Size (Words): 3000`
        - Make sure micro-ROS task has more than 10 kB of stack (1 Word = 4 Bytes)!

Click `Device Configuration Tool Code Generation` or `Gear Icon`
## Step 6 - Clone micro_ros_stm32cubemx_utils
Go to the your project folder in workspace, and then open terminal
```bash
git clone https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git
cd micro_ros_stm32cubemx_utils
git checkout humble
git branch
```
Next, copy the content from the `extra_sources` folder and paste it into the `core -> src` directory.

In the `microros_transport` directory, delete all files except for `dma_transport.c`.

## Step 7 - Setting Project's Properties

- Navigate to `Project -> Settings -> C/C++ Build -> Settings -> Build Steps Tab `
    - In `Pre-build steps` add:
    ```bash
    docker pull microros/micro_ros_static_library_builder:humble && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble
    ```

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

Finally, right-click the project and select `Build`. At this point, It will take a while, and the build should complete without any errors.

## Step 8 - Add Micro-ROS Code to main.c
### 1. Include Libraries
```c
/* USER CODE BEGIN Includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
/* USER CODE END Includes */
```

### 2. Create Functions
```c
/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */
```

### 3. Edit DefaultTask
Replace `huart3` with the UART port that you are using:
```c
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  // micro-ROS configuration

  rmw_uros_set_custom_transport(
    true,
    (void *) &huart2,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);

  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
      printf("Error on default allocators (line %d)\n", __LINE__);
  }

  // micro-ROS app

  rcl_publisher_t publisher;
  std_msgs__msg__Int32 msg;
  rclc_support_t support;
  rcl_allocator_t allocator;
  rcl_node_t node;
  rclc_executor_t executor;

  allocator = rcl_get_default_allocator();

  //create init_options
  rclc_support_init(&support, 0, NULL, &allocator);

  // create node
  rclc_node_init_default(&node, "cubemx_node", "", &support);

  // create publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "cubemx_publisher");

  rclc_executor_init(&executor, &support.context, 1, &allocator);

  msg.data = 0;

  for(;;)
  {
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    if (ret != RCL_RET_OK)
    {
      printf("Error publishing (line %d)\n", __LINE__);
    }

    msg.data++;
    osDelay(10);
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  }
  /* USER CODE END 5 */
}
```

### Additional
If you want to reconnect to your board, you can add:
```c
if (ret != RCL_RET_OK)
{
    printf("Error publishing (line %d)\n", __LINE__);
    NVIC_SystemReset();
}
```
Then, click `Run` to upload the code to the Nucleo board.

## Step 9 - Running Micro-ROS
Grant permission to Docker:
```bash
sudo chmod 666 /var/run/docker.sock
```

Run the Micro-ROS Agent (Change the baud rate to match your configuration):
```bash
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyACM0 
```

Check if the agent is running successfully:
```bash
ros2 topic list
```

If you see `/cubemx_publisher` in the list, congratulations! You have successfully installed Micro-ROS on the Nucleo board.

If nothing appear, press the reset button.
## Documentation

### GitHub Repositories:
- [micro_ros_setup - Humble](https://github.com/micro-ROS/micro_ros_setup/tree/humble)
- [micro_ros_stm32cubemx_utils - Humble](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble)
- [micro_ros_arduino](https://github.com/micro-ROS/micro_ros_arduino/tree/humble)

### Installations:
- [Install STM32CUBEIDE on Linux (Ubuntu) by Embedded Icon](https://youtu.be/j3P2rsB_-BY?si=XfH9ioiwhtgmfos6)
- [How to Install Docker on Ubuntu: A Step-By-Step Guide by vCloudBitsBytes](https://www.youtube.com/watch?v=cqbh-RneBlk)

### Special Thanks:
Thanks to the following videos that have helped me reach this point:

- [How to Set Up Micro-ROS on Any STM32 Microcontroller by Robotics in a Nutshell](https://www.youtube.com/watch?v=xbWaHARjSmk)
- [Micro-ROS STM32 with STM32CubeIDE by Sokheng Din](https://www.youtube.com/watch?v=bn-P3fxtTF4)

## Feedback
If you have any feedback, please create an issue and I will answer your questions there.

## Special Thanks

**P'Charwe, P'Kai, P'Pro FRAB8** - Providing material and support

**P'Beam FRAB9** - Providing Nucleo Board

**Liew Wuttipat** - Fixing Micro-ROS issues
