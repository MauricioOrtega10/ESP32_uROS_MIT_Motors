#include <Arduino.h>
#include <micro_ros_platformio.h>

#include "MitMotor.h"
#include "RmdMotor.h"
#include "array"
#include <algorithm>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <mit_motors_interfaces/msg/motor_info.h>
#include <mit_motors_interfaces/srv/motor_mode.h>
#include <mit_motors_interfaces/srv/control_loop.h>
#include <mit_motors_interfaces/srv/set_position.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only avaliable for Arduino framework with serial transport.
#endif

rcl_publisher_t motor_info_publisher;
mit_motors_interfaces__msg__MotorInfo motor_info_msg;
mit_motors_interfaces__srv__MotorMode_Request motor_mode_srv_req;
mit_motors_interfaces__srv__MotorMode_Response motor_mode_srv_res;
mit_motors_interfaces__srv__ControlLoop_Request control_loop_srv_req;
mit_motors_interfaces__srv__ControlLoop_Response control_loop_srv_res;
mit_motors_interfaces__srv__SetPosition_Request set_position_srv_req;
mit_motors_interfaces__srv__SetPosition_Response set_position_srv_res;

const rosidl_message_type_support_t * motor_info_support = ROSIDL_GET_MSG_TYPE_SUPPORT(
  mit_motors_interfaces, msg, MotorInfo);

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer_pub;
rcl_timer_t timer_control;
rcl_service_t motor_mode_service;
rcl_service_t control_loop_service;
rcl_service_t set_position_service;

/* 1 publisher, 3 services. 
Change "DRMW_UXRCE_MAX_SERVICES" and "DRMW_UXRCE_MAX_GUARD_CONDITION" value
 in micro_ros_platformio/metas/colcon.meta
It is better create a new meta file, <file_name.meta>, based in colcon.meta.
If use a meta file different to colcon.meta, Add in platformio.ini this line: 
  board_microros_user_meta = <file_name.meta>
num_handles is the total number of subscriptions, timers, services, clients 
  and guard conditions. Do not include the number of nodes and publishers.
References:
  https://github.com/micro-ROS/micro_ros_platformio - Other configuration
  https://github.com/micro-ROS/micro_ros_platformio/issues/87
  https://github.com/micro-ROS/micro_ros_platformio/issues/103
  https://github.com/micro-ROS/micro_ros_platformio/issues/100
*/
unsigned int num_handles = 6; 
float position = 0.0;
bool control_flag = false;
float Fc = 35;


#define CS_0 5
#define INT_0 4

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return;}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}
//#define RUNWAIT(fn) {fn; delay(10);}
//#define RUNWAIT_LONG(fn) {fn; delay(100);}

//Frame time in ms 
#define PUBLISHER_FRAME_TIME 500
#define CONTROL_FRAME_TIME 1
float T = CONTROL_FRAME_TIME/ 1000.0;

CanMotor * motors[] = 
{
    new MitMotor(MitMotor::GIM, CS_0, INT_0, "GIM_0")
};

constexpr size_t NUM_MOTORS = sizeof(motors) / sizeof(motors[0]);

void(*interrupt_handlers[NUM_MOTORS])() = {
    [](){motors[0]->handleInterrupt();}
};

// Conrol variables
float tau[NUM_MOTORS];
float kp[NUM_MOTORS];
float kd[NUM_MOTORS];
float qd[NUM_MOTORS];
float Dq[NUM_MOTORS];
float theta[NUM_MOTORS];

void controlMotors()
{
  for (uint8_t i = 0; i < NUM_MOTORS; i++)
  {
      Dq[i] = Fc * (theta[i] + motors[i]->position());
      theta[i] = theta[i] - T * Dq[i];
  }

  for(uint8_t i = 0; i < NUM_MOTORS; i++)
  {
    qd[i] = position;
    tau[i] = -kp[i] * (motors[i]->position() - qd[i])- (kd[i] * Dq[i]);

    if (!motors[i]->setTorque(tau[i],2000))
    {
      Serial.print("Message NOT Sent to "); Serial.println(motors[i]->name());
    } 
    if (!motors[i]->readMotorResponse(2000))
    {
      Serial.print("No response pending in "); Serial.print(motors[i]->name()); Serial.println("MCP2515 Buffer");
    }
  }
}

// Error handle loop
void error_loop() 
{
  while(1) 
  {
    delay(100);
  }
}


/* 
  Problemas a la hora de asignar directamente un valor al string.
    String en c cuenta con 3 parametros: data, size, position.
    Antes de asignar un valor a data de debe de castear el valor a un char *.
    Los demas datos del string no necesitan tener asignado un valor. 
    References:
      https://docs.ros2.org/foxy/api/rosidl_runtime_c/string_8h_source.html
      https://github.com/micro-ROS/micro-ROS-demos/blob/iron/rclc/complex_msg_publisher/main.c
      https://github.com/yalicar/esp32_microros/blob/master/src/ros.cpp
  */
void timer_pub_callback(rcl_timer_t * timer, int64_t last_call_time) 
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) 
  {
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
      motor_info_msg.name.data = const_cast<char*>(motors[i]->name());;
      //msg.name.size = 0;
      //msg.name.capacity = sizeof(msg.name.data);
      motor_info_msg.position = motors[i]->position();
      motor_info_msg.velocity = motors[i]->velocity();
      motor_info_msg.torque = motors[i]->torque();
      RCSOFTCHECK(rcl_publish(&motor_info_publisher, &motor_info_msg, NULL));
    }
    
  }
}

void timer_control_callback(rcl_timer_t * timer, int64_t last_call_time) 
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) 
  {
    if(control_flag == true)
    {
      controlMotors();
    }
  }
}

void motor_mode_callback(const void * req, void * res)
{
  mit_motors_interfaces__srv__MotorMode_Request * req_in = (mit_motors_interfaces__srv__MotorMode_Request *) req;
  mit_motors_interfaces__srv__MotorMode_Response * res_in = (mit_motors_interfaces__srv__MotorMode_Response *) res;

  if (req_in->state == true)
  {
    for (auto & motor : motors)
    {
      while(!motor->turnOn())
      {
        Serial.print("Retrying to turn on "); Serial.println(motor->name());
      }
    }
    res_in->state = true;
  }
  else
  {
  for (auto & motor : motors)
    {
      while(!motor->turnOff())
      {
          Serial.print("Retrying to turn off "); Serial.println(motor->name());
      }
    }
    res_in->state = false;
  }
}

void control_loop_callback(const void * req, void * res)
{
  mit_motors_interfaces__srv__ControlLoop_Request * req_in = (mit_motors_interfaces__srv__ControlLoop_Request *) req;
  mit_motors_interfaces__srv__ControlLoop_Response * res_in = (mit_motors_interfaces__srv__ControlLoop_Response *) res;

  if (req_in->state == true)
  {
    control_flag = true;
    res_in->success = true;
  }
  else
  {
    control_flag = false;
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
      while (!motors[i]->setTorque(0, 1000)){}
      while (!motors[i]->readMotorResponse(2000)){}
      while (!motors[i]->setTorque(0, 1000)){}
      while (!motors[i]->readMotorResponse(2000)){}
    }
    
    res_in->success = false;
  }
  
}

void set_position_callback(const void * req, void * res)
{
  mit_motors_interfaces__srv__SetPosition_Request * req_in = (mit_motors_interfaces__srv__SetPosition_Request *) req;
  mit_motors_interfaces__srv__SetPosition_Response * res_in = (mit_motors_interfaces__srv__SetPosition_Response *) res;
  position = req_in->position;
  res_in->success = true;
}

void setup() 
{
  // Configure serial transport
  Serial.begin(115200);

  for (uint8_t i = 0; i < NUM_MOTORS; i++)
    {
        tau[i] = 0;
        kp[i] = 0.25;
        kd[i] = 0.005;
        qd[i] = 0.0;
        Dq[i] = 0.0;
        theta[i] = 0.0;
    }

  for (auto & motor : motors)
  {
    while(!motor->initialize())
    {
        Serial.print("Retrying to initialize "); Serial.print(motor->name()); Serial.print(" MCP2515");
    }
    Serial.println("All motors initialized succesfully");
  }
  
  set_microros_serial_transports(Serial);

  /* Configure native_ethernet transport
  ToDo: Colocar comando para saber ip y mac
    byte local_mac[] = {0x60, 0xA4, 0x4C, 0x70, 0x60, 0x80};//84
    IPAddress local_ip(192, 168, 100, 177);
    IPAddress agent_ip(192, 168, 100, 228);
    size_t agent_port = 8888;
    set_microros_native_ethernet_transports(local_mac, local_ip, agent_ip, agent_port);
  */

  //Delay obligatorio para la correcta conexion con el agente uROS
  delay(2000);

 

  // Create init_options
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(
    &node, 
    "mit_motor_node", 
    "", 
    &support));

  //Create motor info publisher
  RCCHECK(rclc_publisher_init_default(
    &motor_info_publisher,
    &node,
    motor_info_support,
    "motor_info_publisher"));

  //Create motor mode service
  RCCHECK(rclc_service_init_default(
    &motor_mode_service,
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(mit_motors_interfaces, srv, MotorMode),
    "/motormode"));

  //Create control loop service
  RCCHECK(rclc_service_init_default(
    &control_loop_service,
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(mit_motors_interfaces, srv, ControlLoop),
    "/controlloop"));

  //Create set position service
  RCCHECK(rclc_service_init_default(
    &set_position_service,
    &node, 
    ROSIDL_GET_SRV_TYPE_SUPPORT(mit_motors_interfaces, srv, SetPosition),
    "/setposition"));

  // Create publisher timer
  RCCHECK(rclc_timer_init_default(
    &timer_pub,
    &support,
    RCL_MS_TO_NS(PUBLISHER_FRAME_TIME),
    timer_pub_callback));

    // Create control timer
    RCCHECK(rclc_timer_init_default(
    &timer_control,
    &support,
    RCL_MS_TO_NS(CONTROL_FRAME_TIME),
    timer_control_callback));

  // Create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_pub));
  RCCHECK(rclc_executor_add_timer(&executor, &timer_control));
  RCCHECK(rclc_executor_add_service(
    &executor, &set_position_service, &set_position_srv_res, &set_position_srv_req, set_position_callback));
  RCCHECK(rclc_executor_add_service(
    &executor, &motor_mode_service, &motor_mode_srv_res, &motor_mode_srv_req, motor_mode_callback));
  RCCHECK(rclc_executor_add_service(
    &executor, &control_loop_service, &control_loop_srv_res, &control_loop_srv_req, control_loop_callback));
  
}

void loop() 
{
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}