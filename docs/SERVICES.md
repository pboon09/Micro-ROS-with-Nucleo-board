# Services

Both examples use `example_interfaces/srv/AddTwoInts`, which is already in the micro-ROS library. All code goes in [`Core/Src/app_freertos.c`](../uros_example/Core/Src/app_freertos.c).

The host must use Fast DDS, the ROS 2 default. With CycloneDDS, service requests reach the board as garbage. `echo $RMW_IMPLEMENTATION` should print nothing or `rmw_fastrtps_cpp`.

## Server on the board

The host asks, the board answers.

```c
/* USER CODE BEGIN Includes */
#include <example_interfaces/srv/add_two_ints.h>

/* USER CODE BEGIN Variables */
rcl_service_t add_service;
example_interfaces__srv__AddTwoInts_Request  add_req;
example_interfaces__srv__AddTwoInts_Response add_res;

/* USER CODE BEGIN FunctionPrototypes */
void add_service_callback(const void *req_in, void *res_out);

/* USER CODE BEGIN StartDefaultTask, after the node is created */
rclc_service_init_default(&add_service, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "board_add_two_ints");

rclc_executor_init(&executor, &support.context, 3, &allocator);  // was 2
rclc_executor_add_service(&executor, &add_service, &add_req, &add_res, add_service_callback);

/* USER CODE BEGIN Application */
void add_service_callback(const void *req_in, void *res_out) {
    const example_interfaces__srv__AddTwoInts_Request *req = req_in;
    example_interfaces__srv__AddTwoInts_Response *res = res_out;
    res->sum = req->a + req->b;
}
```

Build, flash, start the agent, then call it. Expect `sum=5`.

```bash
ros2 service call /board_add_two_ints example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

## Client on the board

The board asks, the host answers. The request is sent from the timer, once a second.

```c
/* USER CODE BEGIN Includes */
#include <example_interfaces/srv/add_two_ints.h>

/* USER CODE BEGIN Variables */
rcl_client_t add_client;
example_interfaces__srv__AddTwoInts_Request  add_client_req;
example_interfaces__srv__AddTwoInts_Response add_client_res;
int64_t last_sum = 0;

/* USER CODE BEGIN FunctionPrototypes */
void add_client_callback(const void *res_in);

/* USER CODE BEGIN StartDefaultTask, after the node is created */
rclc_client_init_default(&add_client, &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(example_interfaces, srv, AddTwoInts), "add_two_ints");

rclc_executor_init(&executor, &support.context, 3, &allocator);  // was 2
rclc_executor_add_client(&executor, &add_client, &add_client_res, add_client_callback);

/* USER CODE BEGIN Application */
void add_client_callback(const void *res_in) {
    const example_interfaces__srv__AddTwoInts_Response *res = res_in;
    last_sum = res->sum;
}

// inside timer_callback, which runs every 10 ms
static uint32_t ticks = 0;
if (++ticks >= 100) {
    ticks = 0;
    add_client_req.a = 2;
    add_client_req.b = 3;
    int64_t seq;
    rcl_send_request(&add_client, &add_client_req, &seq);
}
```

Build, flash, start the agent, then run the demo server. Expect `Incoming request` and `a: 2 b: 3` every second. `last_sum` reads `5` in the debugger's **Live Expressions**.

```bash
ros2 run demo_nodes_cpp add_two_ints_server
```

## Limits

The library allows one service and one client. For more, raise `RMW_UXRCE_MAX_SERVICES` or `RMW_UXRCE_MAX_CLIENTS` in [`colcon.meta`](../uros_example/micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/colcon.meta), then [rebuild the library](CUSTOM_INTERFACES.md#3-rebuild-the-library).
