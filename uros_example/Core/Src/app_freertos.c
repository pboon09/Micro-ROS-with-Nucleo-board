/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : app_freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
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

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define RCLSOFTCHECK(fn) if (fn!= RCL_RET_OK){};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 3000 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
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

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	// Add init here eg. Robot Config
	HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	rmw_uros_set_custom_transport(true, (void*) &hlpuart1,
			cubemx_transport_open, cubemx_transport_close,
			cubemx_transport_write, cubemx_transport_read);

	rcl_allocator_t freeRTOS_allocator =
			rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate = microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		printf("Error on default allocators (line %d)\n", __LINE__);
	}
	allocator = rcl_get_default_allocator();

	//create init
	init_options = rcl_get_zero_initialized_init_options();
	RCLSOFTCHECK(rcl_init_options_init(&init_options, allocator));
	RCLSOFTCHECK(rcl_init_options_set_domain_id(&init_options, 127));

	//create support
	rclc_support_init_with_options(&support, 0, NULL, &init_options,
			&allocator);

	// create node
	rclc_node_init_default(&node, "uros_motor_node", "", &support);

	pub_msg.layout.dim.capacity = 1;
	pub_msg.layout.dim.size = 1;
	pub_msg.layout.dim.data = malloc(
			sizeof(std_msgs__msg__MultiArrayDimension) * 1);

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

	//create timer
	rclc_timer_init_default(&timer, &support, timer_period, timer_callback);

	//create executor
	executor = rclc_executor_get_zero_initialized_executor();
	rclc_executor_init(&executor, &support.context, 2, &allocator); // total number of handles = #subscriptions + #timers + #clients + #service (Should not handle too much)
	rclc_executor_add_timer(&executor, &timer);
	rclc_executor_add_subscription(&executor, &subscriber, &sub_msg,
			&subscription_callback, ON_NEW_DATA);
	rclc_executor_spin(&executor);
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
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
		//Run at 1k Hz
	}
	/* USER CODE END Callback 1 */
}
/* USER CODE END Application */

