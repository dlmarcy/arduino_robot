
#include "MicroROSArduino.h"

MicroROSArduino::MicroROSArduino()
{
  // create transport
  set_microros_transports();

  numSensors = 0;
  numBatterys = 0;
  numIMUs = 0;
  for ( int i = 0; i < MAX_SENSORS; i++ )
  {
  	sensors[i].msgIndex = -1;
  }	
  
  command = false;
  // init finite state machine to auto re-connect to micro_ros_agent
  state = WAITING_AGENT;
}

void MicroROSArduino::beginSession()
{
  // create allocator
  allocator = rcl_get_default_allocator();
  // create init_options
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  // create node
  if (rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support) != RCL_RET_OK) {
    errorLoop();
  }
}

void MicroROSArduino::endSession()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
}

void MicroROSArduino::endNode()
{
  if (rcl_node_fini(&node) != RCL_RET_OK) {errorLoop();}
  rclc_support_fini(&support);
}

void MicroROSArduino::errorLoop()
{
  while(1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(800);
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);
  }
}

void MicroROSArduino::spin()
{
  switch(state) {
    case WAITING_AGENT:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
      digitalWrite(14, HIGH); // g1
      digitalWrite(15, LOW); // r1
      digitalWrite(22, LOW); // g2
      digitalWrite(23, LOW); // r2
      delay(200);
      break;
    case AGENT_AVAILABLE:
      beginSession();
	  createBroadcasters();
	  
      if (command) {createJointStateCommander();}
      
      digitalWrite(14, LOW); // g1
      digitalWrite(15, HIGH); // r1
      digitalWrite(22, LOW); // g2
      digitalWrite(23, LOW); // r2
      state = AGENT_CONNECTED;
      break;
    case AGENT_CONNECTED:
      state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
      if (state == AGENT_CONNECTED) {
        spinBroadcasters();
        
        if (command) {rclc_executor_spin_some(&command_executor, RCL_MS_TO_NS(20));}
        digitalWrite(14, LOW); // g1
        digitalWrite(15, LOW); // r1
        digitalWrite(22, HIGH); // g2
        digitalWrite(23, LOW); // r2
      }
      break;
    case AGENT_DISCONNECTED:
      endSession();
      destroyBroadcasters();
      
      if (command) {endJointStateCommander();}
      endNode();
      digitalWrite(14, LOW); // g1
      digitalWrite(15, LOW); // r1
      digitalWrite(22, LOW); // g2
      digitalWrite(23, HIGH); // r2
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}
   
uint8_t MicroROSArduino::beginBroadcaster(teSensorType type, String topic, float rate, void (*function)(rcl_timer_t*, int64_t), int NumJoints, String JointNames[]) 
{
	uint8_t id = numSensors;
	//initialize the sensor object
	sensors[numSensors].type = type;
	sensors[numSensors].topic = topic;
	sensors[numSensors].rate = rate;
	sensors[numSensors].function = function;  
	switch ( type )
	{
	case eeBattery:		
		sensors[numSensors].msgIndex = numBatterys;
		numBatterys++;	
		break;
	case eeIMU:			
		sensors[numSensors].msgIndex = numIMUs;
		numIMUs++;		
		break;
	default: break;
	}
	
	//There is only one joint_state_msg
	if ( type == eeJointState )
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
	
	return id;
}
void MicroROSArduino::endBroadcaster(uint8_t sensor_id)
{
	//place holder for future, user (sketch) might call this
}

void MicroROSArduino::createBroadcasters()
{
	for ( int i = 0; i < MAX_SENSORS; i++ )
	{
		if ( sensors[i].topic.length() != 0 )
		{
 			// create sensor publisher
			switch ( sensors[i].type )
			{
			case eeBattery:
 				if (rclc_publisher_init_default(
			    		&sensors[i].broadcaster,
			    		&node,
			    		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
			    		String("arduino/" + sensors[i].topic).c_str()) != RCL_RET_OK) {
			    	errorLoop();
				}
				break;
			case eeRange:
 				if (rclc_publisher_init_default(
			    		&sensors[i].broadcaster,
			    		&node,
			    		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
			    		String("arduino/" + sensors[i].topic).c_str()) != RCL_RET_OK) {
			    	errorLoop();
				}
				break;
			case eeIMU:
 				if (rclc_publisher_init_default(
			    		&sensors[i].broadcaster,
			    		&node,
			    		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
			    		String("arduino/" + sensors[i].topic).c_str()) != RCL_RET_OK) {
			    	errorLoop();
				}
				break;
			case eeJointState:
 				if (rclc_publisher_init_default(
			    		&sensors[i].broadcaster,
			    		&node,
			    		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
			    		String("arduino/" + sensors[i].topic).c_str()) != RCL_RET_OK) {
			    	errorLoop();
				}
				break;
			default:
				break;
			}
			
  			// create sensor timer,
			if (rclc_timer_init_default(
					&sensors[i].timer,
					&support,
					RCL_MS_TO_NS(1000/sensors[i].rate),
					sensors[i].function) != RCL_RET_OK) {
				errorLoop();
			}
			// create sensor executor
			if (rclc_executor_init(&sensors[i].executor, &support.context, 1, &allocator) != RCL_RET_OK) {
				errorLoop();
			}
			if (rclc_executor_add_timer(&sensors[i].executor, &sensors[i].timer) != RCL_RET_OK) {
				errorLoop();
			}
		}
	}
}
void MicroROSArduino::publishBroadcaster(uint8_t id)
{
	int index = sensors[id].msgIndex;
	switch ( sensors[id].type )
	{
	case eeBattery:
  		if ( rcl_publish(&sensors[id].broadcaster, &battery_msg[index], NULL) != RCL_RET_OK) {}
  		break;
  	case eeRange:
  		if ( rcl_publish(&sensors[id].broadcaster, &range_msg, NULL) != RCL_RET_OK) {}
  		break;
  	case eeIMU:
  		if ( rcl_publish(&sensors[id].broadcaster, &imu_msg[index], NULL) != RCL_RET_OK) {}
  		break;
  	case eeJointState:
  		if ( rcl_publish(&sensors[id].broadcaster, &joint_state_msg, NULL) != RCL_RET_OK) {}
  		break;
  	default:
  		break;
  	}
}

void MicroROSArduino::spinBroadcasters()
{
	//go through list of broadcasters and spin
	for ( int i = 0; i < MAX_SENSORS; i++ )
	{
		if ( sensors[i].topic.length() > 0 ) 
    		rclc_executor_spin_some(&sensors[i].executor, RCL_MS_TO_NS(20));
	}	
}

void MicroROSArduino::destroyBroadcasters()
{
	//go through list finishing each sensor
	for ( int i = 0; i < MAX_SENSORS; i++ )
	{
		if ( sensors[i].topic.length() > 0 ) 
		{
			if (rcl_publisher_fini(&sensors[i].broadcaster, &node) != RCL_RET_OK) {errorLoop();}
	  		if (rcl_timer_fini(&sensors[i].timer) != RCL_RET_OK) {errorLoop();}
  			rclc_executor_fini(&sensors[i].executor);
  		}
	}
}

//-----------------------------------------------------------------------------------------------------------------


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

void MicroROSArduino::createJointStateCommander()
{
  // create joint state commander
  if (rclc_subscription_init_default(
    &joint_state_commander,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    String("arduino/" + command_topic).c_str()) != RCL_RET_OK) {
    errorLoop();
  }
  // create joint state command executor
  if (rclc_executor_init(&command_executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  if (rclc_executor_add_subscription(&command_executor, &joint_state_commander, &command_msg, command_function, ON_NEW_DATA) != RCL_RET_OK) {
    errorLoop();
  }
}

void MicroROSArduino::endJointStateCommander()
{
  if (rcl_subscription_fini(&joint_state_commander, &node) != RCL_RET_OK) {errorLoop();}
  rclc_executor_fini(&command_executor);
}



