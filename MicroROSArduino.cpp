#include "MicroROSArduino.h"

MicroROSArduino::MicroROSArduino()
{
  // create transport
  set_microros_transports();
  // set all counters to 0
  numSensors = numBattery = numPressure = numLight = numImu = 0;
  numJointStateBroad = numJoyStick = numMagnet = numGps = numRange = 0;
  numHumidity = numTemperature = 0;
  // identify that there are no sensors
  for ( int i = 0; i < MAX_SENSORS; i++ )
  {
    sensors[i].msgIndex = -1;
  }
  // identify that there are no commanders
  command = false;
  // initilize finite state machine to auto re-connect to micro_ros_agent
  state = WAITING_AGENT;
}

//---Error----------------------------------------------------------------------------------------------------

void MicroROSArduino::errorLoop()
{
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(800);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

//---Spin--------------------------------------------------------------------------------------------------------

void MicroROSArduino::spin()
{
  switch(state) 
  {
    case WAITING_AGENT:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      digitalWrite(14, HIGH); // g1
      digitalWrite(15, LOW); // r1
      digitalWrite(22, LOW); // g2
      digitalWrite(23, LOW); // r2
      delay(200);
      break;
    case AGENT_AVAILABLE:
      state = AGENT_CONNECTED;
      createSession();
      createBroadcasters();	  
      if (command) {createJointStateCommander();}
      digitalWrite(14, LOW); // g1
      digitalWrite(15, HIGH); // r1
      digitalWrite(22, LOW); // g2
      digitalWrite(23, LOW); // r2
      break;
    case AGENT_CONNECTED:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      if (state == AGENT_CONNECTED) 
      {
        // re sync if neccessary
        if ((millis()-re_sync_time) > 60000) 
        {
          rmw_uros_sync_session(10);
          re_sync_time = millis();
        }
        //go through list of broadcasters and spin
	for ( int id = 0; id < numSensors; id++ )
	{
          rclc_executor_spin_some(&sensors[id].executor, RCL_MS_TO_NS(5));
        }
        // go through list of commanders and spin
        if (command) {rclc_executor_spin_some(&command_executor, RCL_MS_TO_NS(5));}
        digitalWrite(14, LOW); // g1
        digitalWrite(15, LOW); // r1
        digitalWrite(22, HIGH); // g2
        digitalWrite(23, LOW); // r2
      }
      break;
    case AGENT_DISCONNECTED:
      state = WAITING_AGENT;
      destroySession();
      destroyBroadcasters();
      if (command) {destroyJointStateCommander();}
      destroyNode();
      digitalWrite(14, LOW); // g1
      digitalWrite(15, LOW); // r1
      digitalWrite(22, LOW); // g2
      digitalWrite(23, HIGH); // r2
      break;
    default:
      break;
  }
}

//---Session----------------------------------------------------------------------------------------------------

void MicroROSArduino::createSession()
{
  // create allocator
  allocator = rcl_get_default_allocator();
  // create init_options
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) { errorLoop(); }
  // create node
  if (rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support) != RCL_RET_OK) { errorLoop(); }
  // sync time
  rmw_uros_sync_session(1000);
  re_sync_time = millis();
}

void MicroROSArduino::destroySession()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
}

void MicroROSArduino::destroyNode()
{
  if (rcl_node_fini(&node) != RCL_RET_OK) {errorLoop();}
  rclc_support_fini(&support);
}

//---Broadcasters----------------------------------------------------------------------------------------------------

uint8_t MicroROSArduino::beginBroadcaster(teSensorType type, String topic, float rate, void (*function)(rcl_timer_t*, int64_t), int NumJoints, String JointNames[]) 
{
  // add the new sensor in the sensor list at 'id' and return the 'index' in the message list
  uint8_t id = numSensors;
  uint8_t index = -1;
  //initialize the sensor object
  sensors[id].type = type;
  sensors[id].topic = topic;
  sensors[id].rate = rate;
  sensors[id].function = function;  
  switch ( type )
  {
    case BATTERY:
      if ( numBattery < MAX_BATTERY ) {
        sensors[id].msgIndex = numBattery;
        index = numBattery;
        numBattery++;
      } 
      break;
    case PRESSURE:		
      if ( numPressure < MAX_PRESSURE ) {
        sensors[id].msgIndex = numPressure;
        index = numPressure;
        numPressure++;
      }
      break;
    case LIGHT:		
      if ( numLight < MAX_LIGHT ) {
        sensors[id].msgIndex = numLight;
        index = numLight;
        numLight++;
      }
      break;
    case IMU:			
      if ( numImu < MAX_IMU ) {
        sensors[id].msgIndex = numImu;
        index = numImu;
        numImu++;
      }
      break;
     case JOINTSTATEBROAD:		
      if ( numJointStateBroad < MAX_JOINTSTATEBROAD ) {
        sensors[id].msgIndex = numJointStateBroad;
        index = numJointStateBroad;
        numJointStateBroad++;
      }
      break;
    case JOYSTICK:		
      if ( numJoyStick < MAX_JOYSTICK ) {
        sensors[id].msgIndex = numJoyStick;
        index = numJoyStick;
        numJoyStick++;
      }
      break;
    case MAGNET:		
      if ( numMagnet < MAX_MAGNET ) {
        sensors[id].msgIndex = numMagnet;
        index = numMagnet;
        numMagnet++;
      } 
      break;
    case GPS:		
      if ( numGps < MAX_GPS ) {
        sensors[id].msgIndex = numGps;
        index = numGps;
        numGps++;
      }
      break;
    case RANGE:		
      if ( numRange < MAX_RANGE ) {
        sensors[id].msgIndex = numRange;
        index = numRange;
        numRange++;
      }
      break;
    case HUMIDITY:		
      if ( numHumidity < MAX_HUMIDITY ) {
        sensors[id].msgIndex = numHumidity;
        index = numHumidity;
        numHumidity++;
      } 
      break;
    case TEMPERATURE:		
      if ( numTemperature < MAX_TEMPERATURE ) {
        sensors[id].msgIndex = numTemperature;
        index = numTemperature;
        numTemperature++;
      } 
      break;
   default: 
      break;
  }
  // You must reserve memory for the joint_state_msg (MAX_JOINTSTATEBROAD = 1)
  if ( type == JOINTSTATEBROAD )
  {
    // create joint state msg data
    rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, NumJoints);
    for ( int i = 0; i < NumJoints; i++ ) {
      rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[i], JointNames[i].c_str(), JointNames[i].length());
    }
    joint_state_msg.position.data = (double *) malloc(NumJoints * sizeof(double));
    joint_state_msg.position.size= NumJoints;
    joint_state_msg.position.capacity = NumJoints;
    for ( int i = 0; i < NumJoints; i++ ) {
      joint_state_msg.position.data[i] = 0.0;
    }
    joint_state_msg.velocity.data = (double *) malloc(NumJoints * sizeof(double));
    joint_state_msg.velocity.size = NumJoints;
    joint_state_msg.velocity.capacity = NumJoints;
    for ( int i = 0; i < NumJoints; i++ ) {
      joint_state_msg.velocity.data[i] = 0.0;
    }
    joint_state_msg.effort.data = (double *) malloc(NumJoints * sizeof(double));
    joint_state_msg.effort.size = NumJoints;
    joint_state_msg.effort.capacity = NumJoints;
    for ( int i = 0; i < NumJoints; i++ ) {
      joint_state_msg.effort.data[i] = 0.0;
    }
  }  
  numSensors++;	
  return index;
}

void MicroROSArduino::endBroadcaster(teSensorType type, uint8_t index)
{
  // place holder for future, will allow user to remove a broadcaster
}

void MicroROSArduino::createBroadcasters()
{
  // gp through the list of sensors and create a publisher, timer, and executor for each
  for ( int id = 0; id < numSensors; id++ )
  {
    // create sensor publisher
    switch ( sensors[id].type )
    {
      case BATTERY:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case PRESSURE:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case LIGHT:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Illuminance),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case IMU:
         if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case JOINTSTATEBROAD:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
          break;
      case JOYSTICK:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Joy),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case MAGNET:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case GPS:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case RANGE:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case HUMIDITY:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, RelativeHumidity),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      case TEMPERATURE:
        if (rclc_publisher_init_default(
          &sensors[id].broadcaster,
          &node,
          ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
          String("arduino/" + sensors[id].topic).c_str()) != RCL_RET_OK) { errorLoop(); }
        break;
      default:
        break;
    }
    // create sensor timer,
    if (rclc_timer_init_default(
      &sensors[id].timer,
      &support,
      RCL_MS_TO_NS(1000/sensors[id].rate),
      sensors[id].function) != RCL_RET_OK) { errorLoop(); }
    // create sensor executor
    if (rclc_executor_init(&sensors[id].executor, &support.context, 1, &allocator) != RCL_RET_OK) { errorLoop(); }
    if (rclc_executor_add_timer(&sensors[id].executor, &sensors[id].timer) != RCL_RET_OK) { errorLoop(); }
  }
}

void MicroROSArduino::destroyBroadcasters()
{
  // go through the list of sensors destroying (fini) each
  for ( int id = 0; id < numSensors; id++ )
  {
    if (rcl_publisher_fini(&sensors[id].broadcaster, &node) != RCL_RET_OK) { errorLoop(); }
    if (rcl_timer_fini(&sensors[id].timer) != RCL_RET_OK) { errorLoop(); }
    rclc_executor_fini(&sensors[id].executor);
  }
}

//---Commanders----------------------------------------------------------------------------------------------------

void MicroROSArduino::beginJointStateCommander(void (*function)(const void*), String topic, int NumJoints, String JointNames[])
{
  command_function = function;
  command_topic = topic;
  // create joint state command data
  rosidl_runtime_c__String__Sequence__init(&command_msg.name, NumJoints);
  for ( int i = 0; i < NumJoints; i++ ) {
     rosidl_runtime_c__String__assignn(&command_msg.name.data[i], JointNames[i].c_str(), JointNames[i].length());
  }
  command_msg.position.data = (double *) malloc(NumJoints * sizeof(double));
  command_msg.position.size= NumJoints;
  command_msg.position.capacity = NumJoints;
  command_msg.velocity.data = (double *) malloc(NumJoints * sizeof(double));
  command_msg.velocity.size = NumJoints;
  command_msg.velocity.capacity = NumJoints;
  command_msg.effort.data = (double *) malloc(NumJoints * sizeof(double));
  command_msg.effort.size = NumJoints;
  command_msg.effort.capacity = NumJoints;
  //createJointStateCommander();
  command = true;
}

void MicroROSArduino::endJointStateCommander()
{
  // place holder for future, will allow user to remove a commander
}

void MicroROSArduino::createJointStateCommander()
{
  // create joint state commander
  if (rclc_subscription_init_default(
    &joint_state_commander,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    String("arduino/" + command_topic).c_str()) != RCL_RET_OK) { errorLoop(); }
  // create joint state command executor
  if (rclc_executor_init(&command_executor, &support.context, 1, &allocator) != RCL_RET_OK) { errorLoop(); }
  if (rclc_executor_add_subscription(&command_executor, &joint_state_commander, &command_msg, command_function, ON_NEW_DATA) != RCL_RET_OK) { errorLoop(); }
}

void MicroROSArduino::destroyJointStateCommander()
{
  if (rcl_subscription_fini(&joint_state_commander, &node) != RCL_RET_OK) { errorLoop(); }
  rclc_executor_fini(&command_executor);
}

//---Publishers----------------------------------------------------------------------------------------------------
/*
void MicroROSArduino::publishBroadcaster(uint8_t id)
{
	int index = sensors[id].msgIndex;
	switch ( sensors[id].type )
	{
	case BATTERY:
  		if ( rcl_publish(&sensors[id].broadcaster, &battery_msg[index], NULL) != RCL_RET_OK) {}
  		break;
  	case RANGE:
  		if ( rcl_publish(&sensors[id].broadcaster, &range_msg, NULL) != RCL_RET_OK) {}
  		break;
  	case IMU:
  		if ( rcl_publish(&sensors[id].broadcaster, &imu_msg[index], NULL) != RCL_RET_OK) {}
  		break;
  	case JOINTSTATEBROAD:
  		if ( rcl_publish(&sensors[id].broadcaster, &joint_state_msg, NULL) != RCL_RET_OK) {}
  		break;
  	default:
  		break;
  	}
}
*/
void MicroROSArduino::publishBattery(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  battery_msg[index].header.stamp.sec = stamp_time/1000000000;
  battery_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == BATTERY) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &battery_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishPressure(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  pressure_msg[index].header.stamp.sec = stamp_time/1000000000;
  pressure_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == PRESSURE) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &pressure_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishLight(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  light_msg[index].header.stamp.sec = stamp_time/1000000000;
  light_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == LIGHT) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &light_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishImu(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  imu_msg[index].header.stamp.sec = stamp_time/1000000000;
  imu_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == IMU) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &imu_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishJointStateBroad(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  joint_state_msg.header.stamp.sec = stamp_time/1000000000;
  joint_state_msg.header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if (sensors[id].type == JOINTSTATEBROAD) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &joint_state_msg, NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishJoystick(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  joystick_msg[index].header.stamp.sec = stamp_time/1000000000;
  joystick_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == JOYSTICK) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &joystick_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishMagnet(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  magnet_msg[index].header.stamp.sec = stamp_time/1000000000;
  magnet_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == MAGNET) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &magnet_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishGps(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  gps_msg[index].header.stamp.sec = stamp_time/1000000000;
  gps_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == GPS) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &gps_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishRange(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  range_msg[index].header.stamp.sec = stamp_time/1000000000;
  range_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == RANGE) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &range_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishHumidity(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  humidity_msg[index].header.stamp.sec = stamp_time/1000000000;
  humidity_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == HUMIDITY) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &humidity_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}

void MicroROSArduino::publishTemperature(uint8_t index)
{
  stamp_time = rmw_uros_epoch_nanos();
  temperature_msg[index].header.stamp.sec = stamp_time/1000000000;
  temperature_msg[index].header.stamp.nanosec = stamp_time%1000000000;
  // search for sensor id
  for ( int id = 0; id < numSensors; id++ )
  {
    if ((sensors[id].type == TEMPERATURE) && (sensors[id].msgIndex == index)) 
    {
      if ( rcl_publish(&sensors[id].broadcaster, &temperature_msg[index], NULL) != RCL_RET_OK) {}
      break;
    }
  }	
}
