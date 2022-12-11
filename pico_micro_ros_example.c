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
#include "hardware/pwm.h"
#include "math.h"
const uint LED_PIN = 25;
const uint motorPin1A = 14;// speed
const uint motorPin1B = 15;// direction

const uint motorPin2A = 1; // controls speed
const uint motorPin2B = 0; // controls direction

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg1;
geometry_msgs__msg__Twist msg3;
// nav_msgs__msg__Odometry odom;
int32_t counterRight = 0;
int32_t counterLeft = 0;
unsigned long timeSinceLastCall;
// rcl_publisher_t odomPublisher;
rcl_subscription_t cmdVelSubscriber;
float leftSpeed, rightSpeed = 0;
const float radius = 32.5/1000;
int32_t countsPerRot = 20;
const float pi = 3.1415926;

// gpio_init(motorPin1A);
// gpio_init(motorPin1B);






void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    rcl_ret_t ret = rcl_publish(&publisher, &msg1, NULL);

    leftSpeed = (((counterLeft % 20)/countsPerRot) * (pi/10)/1) * radius;
    rightSpeed = (((counterRight % 20)/countsPerRot) * (pi/10)/1) * radius;


    msg1.data = (int32_t)(rightSpeed * 1000000);
    
    //in meters per second
}

// void (* rclc_subscription_callback_t)(const void *);

void handleCallback(uint pin, uint32_t event_mask){
    if((to_ms_since_boot(get_absolute_time()) - timeSinceLastCall) > 5){
    if(pin == 8){
        counterRight++;
    }
    else if(pin == 9){
        counterLeft++;
    }
    
    timeSinceLastCall = to_ms_since_boot(get_absolute_time());
    }
    
    

}

int PIDController(float KP,float error){
    return KP * error;



}


void drive(const void * msgin){
      const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

        if(msg->linear.x < 0){
        gpio_put(motorPin1B, false);
        gpio_put(motorPin2B, false);
        }


        else if(msg->linear.x > 0){
        gpio_put(motorPin1B, true);
        gpio_put(motorPin2B, true);
        }
        else stop();
        
        pwm_set_gpio_level(motorPin1A, (int)65535 *.8);


        
        pwm_set_gpio_level(motorPin2A, (int)65535 * .8);

        
}

void stop(){

        gpio_put(motorPin1A, false);
        gpio_put(motorPin1B, false);
        
        gpio_put(motorPin1A, false);
        gpio_put(motorPin1B, false);

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
        
        gpio_init(motorPin1A);
        gpio_set_dir(motorPin1A, GPIO_OUT);
        gpio_set_function(motorPin1A, GPIO_FUNC_PWM);
        uint sliceNum = pwm_gpio_to_slice_num(motorPin1A);
        pwm_config config = pwm_get_default_config();
        pwm_init(sliceNum, &config, true);
        
        gpio_init(motorPin1B);
        gpio_set_dir(motorPin1B, GPIO_OUT);

        gpio_init(motorPin2A);
        gpio_set_dir(motorPin2A, GPIO_OUT);
        gpio_set_function(motorPin2A, GPIO_FUNC_PWM);
        uint sliceNum2 = pwm_gpio_to_slice_num(motorPin2A);
        pwm_init(sliceNum2, &config, true);


        gpio_init(motorPin2B);
        gpio_set_dir(motorPin2B, GPIO_OUT);





    timeSinceLastCall = 0;
    gpio_set_irq_enabled_with_callback(8, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &handleCallback);
    gpio_set_irq_enabled(9, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();

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

    gpio_set_dir(motorPin1A, true);
    gpio_set_dir(motorPin1B, true);

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
    unsigned int handles = 1;

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    
    rclc_executor_add_timer(&executor, &timer);
       
    rcl_ret_t rc = rclc_executor_add_subscription(
        &executor,
        &cmdVelSubscriber, &msg3,
        &drive, ON_NEW_DATA
    );
    

    gpio_put(LED_PIN, 1);
    
    
    msg1.data = 0;
    
    
    
    if (RCL_RET_ERROR == rc) {
        msg1.data = 420;
        
        }

    
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    stop();
    return 0;
}
