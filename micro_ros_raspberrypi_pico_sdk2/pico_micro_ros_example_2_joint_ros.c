#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//#include <std_msgs/msg/float64.h>
//#include <std_msgs/msg/float64_multi_array.h>
#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw_microros/rmw_microros.h>
//#include <geometry_msgs/msg/twist.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#define ARRAY_LEN 200
#define AKT_TYP sensor_msgs__msg__JointState
#define step1 14
#define step2 17
#define dir1 15
#define dir2 19


//############################################################################
//micro-ros Node für die Demo
//Der Code beinhaltet essential einen Subscriber für einen Jointstate.
//Der Jointstate wird von einem rviz-node standardmäßig ausgegeben(joint_state_publisher)
//eine jointstate-msg besteht standardmäßig aus position, geschwindigkeit, name und anstrengung(kraft)
//jedes dieser Attribute ist als Array vertreten, jedes element für einen joint. also position des ersten motors= msg->position[0]
//das microros auf C basiert und die jointstate-msg auf arrays basiert, sind ressourcen für diese zu reservieren
//#############################################################################
unsigned long coun=0;
int goal_val1=0;
int goal_val2=0;
int* derpoi;

struct stepper{
int steppin;
int dirpin;
int goal_pos;
int akt_pos;
unsigned int speed;
absolute_time_t dschritt;
bool stepstate
};

//mit absolute_time arbeiten
void stepper_motor_update(struct stepper* stepp){

    if(absolute_time_diff_us(stepp->dschritt,get_absolute_time())>0.5*1000000/stepp->speed){
    stepp->dschritt=get_absolute_time();
    if(stepp->goal_pos==stepp->akt_pos){
        return;
    }
    gpio_put(stepp->steppin,stepp->stepstate);
    stepp->stepstate=stepp->stepstate==0;
    if(stepp->goal_pos<stepp->akt_pos){
       
           gpio_put(stepp->dirpin,0);
           stepp->akt_pos--;
       }
    
    else if(stepp->goal_pos>stepp->akt_pos){
           gpio_put(stepp->dirpin,1);
           stepp->akt_pos++;
       }
  
    }
}


const uint LED_PIN = 25;
sensor_msgs__msg__JointState * join_states_msg;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
sensor_msgs__msg__JointState msg;
AKT_TYP recv_msg;
std_msgs__msg__Int32 s_msg;
unsigned long derint;
int32_t iz;
int32_t izz;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
//Publisher, implementiert, aber nicht notwendig
    //s_msg.data.data[1]=izz;
    s_msg.data=*derpoi;
    gpio_put(LED_PIN, 1);
    rcl_ret_t ret = rcl_publish(&publisher, &s_msg, NULL);
    //s_msg.data.data[0]++;
	//s_msg.data.data[1]=iz;
}
void subscription_callback(const void * msgin)
{
    //subscriber, es werden sich die notwendigen daten geholt.

    iz++;
    gpio_put(LED_PIN, 0);
    const AKT_TYP * msggg = (const AKT_TYP   *)msgin;
    s_msg.data=msggg->position.data[0]*10;
    goal_val1=msggg->position.data[1]*20;//muss so sein
    goal_val2=msggg->position.data[0]*40;
}
int main()
{
    
    struct stepper stepps[2]={
    {step1,dir1,0,0,1000,get_absolute_time(),false},
    {step2,dir2,0,0,1000,get_absolute_time(),false}
    };
    derpoi=&(stepps[0].akt_pos);
    gpio_init(step1);
    gpio_init(step2);
    gpio_init(dir1);
    gpio_init(dir2);
    gpio_set_dir(step1,GPIO_OUT);
    gpio_set_dir(step2,GPIO_OUT);
    gpio_set_dir(dir1,GPIO_OUT);
    gpio_set_dir(dir2,GPIO_OUT);
//reservieren der ressourcen für die message. Node stürzt ab, wenn dies nicht gemacht wird.
    static micro_ros_utilities_memory_conf_t conf = {0};

    bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
  &recv_msg,
  conf
);
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);//transport-funktionen des nodes, ist letzten endes putc und getc umgeschrieben, 
	//kann aber auch für andere schnittstellen wie I2c manipuliert werden
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();
    executor=rclc_executor_get_zero_initialized_executor();
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

    rclc_node_init_default(&node, "pico_nod_yeah", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),//nicht alle message typen werden von microros unterstützt, diese funktion checkt das
        "joint_states");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

	rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"topicc");
       // join_states_msg = create_joint_states_message();
	// 		s_msg.data.data = (int32_t * ) malloc(2 * sizeof(int));
	// s_msg.data.size = 2;
	// s_msg.data.capacity = 2;
    //s_msg.layout.dim1;
    // s_msg.dim[0].size=2;
    // s_msg.dim[0].stride = 1;
    // s_msg.dim[0].label="x";

	// recv_msg.data.data = (char * ) malloc(ARRAY_LEN * sizeof(char));
	// recv_msg.data.size = 0;
	// recv_msg.data.capacity = ARRAY_LEN;

    rclc_executor_init(&executor, &support.context, 2, &allocator);
    rclc_executor_add_timer(&executor, &timer);
	rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA);
    gpio_put(LED_PIN, 1);

absolute_time_t t1=get_absolute_time();
    while (true)
    {

        if(absolute_time_diff_us(t1,get_absolute_time())>100000){//unsauber. Soll halt dafür sorgen, dass der roskram nur alle 100 ms genutzt wird, damit er nicht unötig ressourcen blockiert
            t1=get_absolute_time();
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        //spin für rosnode, spin_some bedeutet, dass ros es weiterlaufen lässt, anstatt den rest des programms zu halten
        }
        derpoi=&(stepps[0].akt_pos);
        stepps[0].goal_pos=goal_val1;
        stepps[1].goal_pos=goal_val2;
        stepper_motor_update(&stepps[0]);
        stepper_motor_update(&stepps[1]);
    }
	// 	rcl_subscription_fini(&subscriber, &node);
	// rcl_publisher_fini(&publisher, &node);
	// rcl_node_fini(&node);
    return 0;
}
