#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"


#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "hardware/structs/iobank0.h"
#include "hardware/irq.h"

const uint LED_PIN = 25;
const uint motorPinA = 14;
const uint motorPinB = 15;

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
// nav_msgs__msg__Odometry odom;
int32_t counterRight = 0;
unsigned long timeSinceLastCall;
// rcl_publisher_t odomPublisher;
rcl_subscription_t cmdVelSubscriber;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data = counterRight;
}


void handleCallback(int pin){
    if((to_ms_since_boot(get_absolute_time()) - timeSinceLastCall) > 5){
    counterRight++;
    timeSinceLastCall = to_ms_since_boot(get_absolute_time());
    }
    
    

}

void drive(const void * msgin){
      const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
 
}

int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    timeSinceLastCall = 0;
    gpio_set_irq_enabled_with_callback(0, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &handleCallback);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");
   
//    rclc_publisher_init_default(
//         &odomPublisher,
//         &node,
//         ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
//         "pico_publisher_Odom");

    rclc_subscription_init_default(
        &cmdVelSubscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );


    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);
       rcl_ret_t rc = rclc_executor_add_subscription(
        &cmdVelSubscriber, &node,
        &drive, ON_NEW_DATA
    );
    

    gpio_put(LED_PIN, 1);

    msg.data = 0;
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
