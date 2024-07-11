#ifndef MICROROSARDUINO_H
#define MICROROSARDUINO_H

#include "Arduino.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

#define MAX_SENSORS 4
#define MAX_BATTERY 2
#define MAX_IMU 2
#define MAX_RANGE 2

class MicroROSArduino
{
public:

	enum teSensorType { eeBattery = 0, eeRange, eeIMU, eeJointState, };
	typedef struct {
		teSensorType	type;		//type of sensor: battery, imu, etc.
		int				msgIndex;	//index into msg array for this sensor
    	String 			topic;		//unique to this sensor
    	float 			rate;
    	rcl_timer_t 	timer;
    	rclc_executor_t executor;
    	rcl_publisher_t broadcaster;
    	void            (*function)(rcl_timer_t*, int64_t);
	} tsSensor;
	
public:
    MicroROSArduino();
    void spin();   
    uint8_t beginBroadcaster(teSensorType type, String topic, float rate, void (*function)(rcl_timer_t*, int64_t), int jointNum=-1, String jointNames[]=nullptr);
    void endBroadcaster(uint8_t sensor_id);
    void publishBroadcaster(uint8_t sensor_id);
	uint8_t getMessageIndex(uint8_t sensor_id)	{return sensors[sensor_id].msgIndex;}

	void publishBattery(uint8_t index);
	void publishRange(uint8_t index);
	void publishImu(uint8_t index);
	void publishJointState();

    sensor_msgs__msg__BatteryState 	battery_msg[MAX_BATTERY];
    sensor_msgs__msg__Imu 			imu_msg[MAX_IMU];    
    sensor_msgs__msg__Range 		range_msg[MAX_RANGE];
    sensor_msgs__msg__JointState 	joint_state_msg;		//there is only one of these
    
public:    
    // joint state commander
    void beginJointStateCommander(void (*command_function)(const void*), String topic, int NumJoints, String JointNames[]);
    void endJointStateCommander();
    void createJointStateCommander();
    void destroyJointStateCommander();

    sensor_msgs__msg__JointState 	command_msg;
    
private:    
    void errorLoop();
    void createSession();
    void destroySession();
    void destroyNode();
    void createBroadcasters();
    void destroyBroadcasters();
    
private:
    rcl_allocator_t allocator;
    rclc_support_t 	support;
    rcl_node_t 		node;
    unsigned long   re_sync_time;
    int64_t         stamp_time;
    
    int numSensors;
    int numBatterys; 
    int numIMUs;
   	tsSensor sensors[MAX_SENSORS];

 
    // joint state commander
    rclc_executor_t command_executor;
    rcl_subscription_t joint_state_commander;
    void (*command_function)(const void*);
    String command_topic;
    bool command;
    
    // Agent connection status
    enum states {
      WAITING_AGENT,
      AGENT_AVAILABLE,
      AGENT_CONNECTED,
      AGENT_DISCONNECTED
    } state;
};

#endif
