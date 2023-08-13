#include <micro_ros_arduino.h>
#include <DynamixelWorkbench.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>
#include <stdio.h>
#include <DynamixelWorkbench.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw_microros/rmw_microros.h>
#define ANZAHL_DYNAMIXEL 2
#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif 
#define AKT_TYP sensor_msgs__msg__JointState
AKT_TYP recv_msg;
AKT_TYP * join_states_msg;
DynamixelWorkbench dxl_wb; 
int offsets[5]={1845,2961,1554,825,2850};
uint8_t get_id[16];
int32_t posis[5]={1845,2961,1554,825,2850};
const char *glog = NULL;
bool result = false;

float umrechnungsfaktor=(4096/(2*3.14159));
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
std_msgs__msg__Int32MultiArray mmsg;
rcl_node_t node;
rcl_timer_t timer;
int iz=7;
uint8_t ids[5]={11,12,13,14,15};
uint8_t scan_cnt;
const uint8_t range=20;
const char *plog = NULL;
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

int rest_4096(int eing){
if(eing<0){
  return eing%4096+4096;
}
return eing%4096;
}

void sr_handler(int index,char* akt){
        static uint8_t sync_write_handler_index = 0;
        //uint8_t id = cmd[1].toInt();

        result = dxl_wb.addSyncWriteHandler(index, akt, &glog);
        if (result == false)
        {
          error_loop();
          Serial1.println(glog);
          Serial1.print("Failed to add sync write handler\n");
          return;
        }
        else
        {
          Serial1.println(glog);
          Serial1.print("sync_write_handler_index = ");
          Serial1.println(sync_write_handler_index);
        }
}
void sync_writ(int index, int32_t* data){
//unnütz glaube
result = dxl_wb.syncWrite(index, ids, 5, (int32_t *)(&data[0]), 1, &glog);
}
void initt(){
  result = dxl_wb.init(DEVICE_NAME, 1000000);
      result = dxl_wb.scan(get_id, &scan_cnt, range,&glog);
      if (result == false)
      {
        error_loop();
        Serial1.println(glog);
        Serial1.println("Failed to scan");
      }
      else
      {
        for (int cnt = 0; cnt < scan_cnt; cnt++)
        {
          Serial1.print("id : ");
          Serial1.print(get_id[cnt]);
          Serial1.print(" model name : ");
          Serial1.println(dxl_wb.getModelName(get_id[cnt]));
        }
      }
for(int i=0;i<ANZAHL_DYNAMIXEL;i++){
result = dxl_wb.torqueOn(ids[i], &plog);
}
if(!result){
Serial1.println("fail4");while(1);
}
      sr_handler(11,"Goal_Position");
}
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  mmsg.data.size=5;
  //posis[3]=iz;
  for(int i=0;i<5;i++){
    mmsg.data.data[i]=posis[i];
  }
  //mmsg.data.data[3]=iz;
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &mmsg, NULL));
    //mmsg.data.data[3]++;
  }
}


void subscription_callback(const void * msgin)
{
	iz++;
   // gpio_put(LED_PIN, 0);
   digitalWrite(LED_PIN,!digitalRead(LED_PIN));
    const AKT_TYP * msggg = (const AKT_TYP   *)msgin;
    //posis[3]++;
  for(int i=0;i<5;i++){
    posis[i]=rest_4096(((int)(msggg->position.data[i]*umrechnungsfaktor))+offsets[i]);
  
  }
  result = dxl_wb.syncWrite(0, ids, ANZAHL_DYNAMIXEL, (int32_t *)posis, 1, &plog);
  //sync_writ(1,posis);
  
}
void setup() {
  set_microros_transports();
  initt();
  bool success=false;
  //folgender Code ist für die Allokation der jointmessage. Ros stellt funktionen für die Bereitstellung von Speicher bereit.
  //für den Typ "Multiarray" ging das bei mir nicht,darum ist das bei mir separat
     static micro_ros_utilities_memory_conf_t conf = {0};
     conf.max_string_capacity = 50;
    conf.max_ros2_type_sequence_capacity = 10;
    conf.max_basic_type_sequence_capacity = 10;
   success = micro_ros_utilities_create_message_memory(
   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
   &recv_msg,
   conf
   );
  //allokation des Speichers für einen Multiarray, das ist WICHTIG, da ROS-Node sonst abstürzt
  //für array-förmige Messages müssen rekursiv für jedes attribut die ressourcen bereitgestellt werden,
  //am besten hierfür im ROS-wiki nachgucken, bsp Int32Multiarray https://docs.ros.org/en/lunar/api/std_msgs/html/msg/Int32MultiArray.html
  mmsg.layout.dim.data = (std_msgs__msg__MultiArrayDimension*)malloc(2*sizeof(std_msgs__msg__MultiArrayDimension));
//msg_sensors->data.data = (int32_t*)malloc(sizeof(senors)); msg_sensors->data.capacity = 1; msg_sensors->data.size = sizeof(senors)/sizeof(int32_t); memcpy(msg_sensors->data.data , senors, sizeof(senors));
  mmsg.data.data = (int32_t * ) malloc(5 * sizeof(int32_t));
  mmsg.data.size = 0;
  mmsg.data.capacity = 5;
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);
  //folgende funktionen sind wichtig für die initialisierung des Nodes, fehlt eine, wird der Node nicht gestartet
  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  const unsigned int timer_timeout = 1000;
  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "micro_ros_arduino_node_publisher"));
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "topicc"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
