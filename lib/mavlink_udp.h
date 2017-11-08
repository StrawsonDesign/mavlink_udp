/*******************************************************************************
* mavlink_udp.h
*******************************************************************************/


#ifndef RC_MAVLINK_UDP
#define RC_MAVLINK_UDP

#include <stdint.h>		// for specific integer types
#include "mavlink/mavlink.h"	// open-source mavlink definitions

// default port for UDP communications
#define RC_MAVLINK_DEFAULT_PORT	14551

/*
// List of the most common mavlink packets which have helper functions built
// into this api and allow for independent callback functions to be assigned
// when a packet of this type arrives. To use other mavlink packets use the
// 'generic' functions below
typedef enum rc_mav_packet_type_t{
	RC_MAV_MESG_OTHER,
	RC_MAV_MSG_HEARTBEAT,
	RC_MAV_MSG_ATTITUDE,
	RC_MAV_MSG_ATTITUDE_QUATERNION,
	RC_MAV_MSG_LOCAL_POSITION_NED,
	RC_MAV_MSG_LOCAL_POSITION_SETPOINT,
	RC_MAV_MSG_BATTERY_STATUS,
	RC_MAV_MSG_GLOBAL_GPS_RAW_INT,
	RC_MAV_MSG_GLOBAL_POSITION_SETPOINT,
	RC_MAV_MSG_GLOBAL_VISION_POSITION_ESTIMATE,
	RC_MAV_MSG_HIL_CONTROLS
} rc_mav_packet_type_t;
*/

// Sending Functions
int rc_mav_init_sender(int port, const char* dest_ip, uint8_t system_id);







int rc_mav_init_listener(int port);

// Cleanup functions
// these are blocking functions that return when the listening thread has
// returned or timed out.
int rc_mav_cleanup_listener();
int rc_mav_cleanup_sender();

// generic sending function
int rc_mav_send_msg(mavlink_message_t msg);

// select helper functions for most common packets
int rc_mav_send_heartbeat();
int rc_mav_send_attitude(rc_mavlink_socket_t* sock, float roll, float pitch, float yaw,
                         float rollspeed, float pitchspeed, float yawspeed);
int rc_mav_send_control(rc_mavlink_socket_t* sock, int16_t x, int16_t y, int16_t z,
                        int16_t r, uint16_t buttons);

// assign a callback function to be called when any packet is received.
int rc_mav_set_callback(int msg_type, void (*func)(void));

// returns the last received packet of type packet
void* rc_mav_get_packet(int msg_type);

int64_t rc_mav_ns_since_last_packet(int msg_type);


#endif /* RC_MAVLINK_UDP */

