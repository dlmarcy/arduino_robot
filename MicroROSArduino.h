#ifndef MICROROSARDUINO_H
#define MICROROSARDUINO_H

#include "Arduino.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/fluid_pressure.h>
#include <sensor_msgs/msg/illuminance.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <sensor_msgs/msg/joy.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/relative_humidity.h>
#include <sensor_msgs/msg/temperature.h>
#include <rosidl_runtime_c/string_functions.h>

#define MAX_SENSORS 4
#define MAX_BATTERY 2
#define MAX_PRESSURE 2
#define MAX_LIGHT 2
#define MAX_IMU 2
#define MAX_JOINTSTATEBROAD 1
#define MAX_JOYSTICK 2
#define MAX_MAGNET 2
#define MAX_GPS 2
#define MAX_RANGE 4
#define MAX_HUMIDITY 2
#define MAX_TEMPERATURE 2

class MicroROSArduino
{
public:
  enum teSensorType { BATTERY = 0, PRESSURE, LIGHT, IMU, JOINTSTATEBROAD, JOYSTICK, MAGNET, GPS, RANGE, HUMIDITY, TEMPERATURE };
  typedef struct {
    teSensorType type; //type of sensor: battery, imu, etc.
    int msgIndex; //index into msg array for this sensor
    String topic; //unique to this sensor
    float rate;
    rcl_timer_t timer;
    rclc_executor_t executor;
    rcl_publisher_t broadcaster;
    void (*function)(rcl_timer_t*, int64_t);
  } tsSensor;
	
public:
  MicroROSArduino();
  void spin();   
  uint8_t beginBroadcaster(teSensorType type, String topic, float rate, void (*function)(rcl_timer_t*, int64_t), int jointNum=-1, String jointNames[]=nullptr);
  void endBroadcaster(teSensorType type, uint8_t index);
//  void publishBroadcaster(uint8_t sensor_id);
  void publishBattery(uint8_t index);
  void publishPressure(uint8_t index);
  void publishLight(uint8_t index);
  void publishImu(uint8_t index);
  void publishJointStateBroad(uint8_t index);
  void publishJoystick(uint8_t index);
  void publishMagnet(uint8_t index);
  void publishGps(uint8_t index);
  void publishRange(uint8_t index);
  void publishHumidity(uint8_t index);
  void publishTemperature(uint8_t index);

  sensor_msgs__msg__BatteryState battery_msg[MAX_BATTERY];
  sensor_msgs__msg__FluidPressure pressure_msg[MAX_PRESSURE];
  sensor_msgs__msg__Illuminance light_msg[MAX_LIGHT];
  sensor_msgs__msg__Imu imu_msg[MAX_IMU];    
  sensor_msgs__msg__JointState joint_state_msg; //there is only one of these
  sensor_msgs__msg__Joy joystick_msg[MAX_JOYSTICK];    
  sensor_msgs__msg__MagneticField magnet_msg[MAX_MAGNET];    
  sensor_msgs__msg__NavSatFix gps_msg[MAX_GPS];    
  sensor_msgs__msg__Range range_msg[MAX_RANGE];
  sensor_msgs__msg__RelativeHumidity humidity_msg[MAX_HUMIDITY];    
  sensor_msgs__msg__Temperature temperature_msg[MAX_TEMPERATURE];    
 
public:    
  // joint state commander
  void beginJointStateCommander(void (*command_function)(const void*), String topic, int NumJoints, String JointNames[]);
  void endJointStateCommander();
  void createJointStateCommander();
  void destroyJointStateCommander();

  sensor_msgs__msg__JointState command_msg;
    
private:    
  // Agent connection status
  enum states { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED } state;

  void errorLoop();
  void createSession();
  void destroySession();
  void destroyNode();
  void createBroadcasters();
  void destroyBroadcasters();

private:
  rcl_allocator_t allocator;
  rclc_support_t support;
  rcl_node_t node;
  unsigned long re_sync_time;
  int64_t stamp_time;

  int numSensors;
  int numBattery; 
  int numPressure; 
  int numLight; 
  int numImu; 
  int numJointStateBroad; 
  int numJoyStick; 
  int numMagnet; 
  int numGps; 
  int numRange; 
  int numHumidity; 
  int numTemperature; 
  tsSensor sensors[MAX_SENSORS];

  // joint state commander
  rclc_executor_t command_executor;
  rcl_subscription_t joint_state_commander;
  void (*command_function)(const void*);
  String command_topic;
  bool command;
};

#endif
