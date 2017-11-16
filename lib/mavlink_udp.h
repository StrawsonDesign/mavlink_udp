/*******************************************************************************
* mavlink_udp.h
*******************************************************************************/


#ifndef RC_MAVLINK_UDP
#define RC_MAVLINK_UDP

#include <stdint.h>		// for specific integer types
#include "mavlink/mavlink.h"	// open-source mavlink definitions

// default port for UDP communications
#define RC_MAV_GROUND_CONTROL_PORT	14550
#define RC_MAV_UAV_PORT			14551


// Sending Initialization
int rc_mav_init_sender(int port, const char* dest_ip, uint8_t system_id);
int rc_mav_set_dest_ip(const char* dest_ip);
int rc_mav_set_system_id(uint8_t system_id);
int rc_mav_cleanup_sender();



// Sending functions
// generic packet sending function
int rc_mav_send_msg(mavlink_message_t msg);

// helper functions for most common packets, removes the need for packing your own
int rc_mav_send_heartbeat();

int rc_mav_send_heartbeat_full(
	uint32_t custom_mode,	// A bitfield for use for autopilot-specific flags.
	uint8_t type,		// Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
	uint8_t autopilot,	// Autopilot type / class. defined in MAV_AUTOPILOT ENUM
	uint8_t base_mode,	// System mode bitfield, see MAV_MODE_FLAGS ENUM in mavlink/include/mavlink_types.h
	uint8_t system_status);	// System status flag, see MAV_STATE ENUM




int rc_mav_send_attitude(
	float roll,		// Roll angle (rad, -pi..+pi)
	float pitch,		// Pitch angle (rad, -pi..+pi)
	float yaw,		// Yaw angle (rad, -pi..+pi)
	float rollspeed,	// Roll angular speed (rad/s)
	float pitchspeed,	// Pitch angular speed (rad/s)
	float yawspeed);	// Yaw angular speed (rad/s)




int rc_mav_send_attitude_quaternion(
	float q1,		// Quaternion component 1
	float q2,		// Quaternion component 2
	float q3,		// Quaternion component 3
	float q4,		// Quaternion component 4
	float rollspeed,	// Roll angular speed (rad/s)
	float pitchspeed,	// Pitch angular speed (rad/s)
	float yawspeed);	// Yaw angular speed (rad/s)




int rc_mav_send_local_position_ned(
	float x,		// X Position
	float y,		// Y Position
	float z,		// Z Position
	float vx,		// X Speed
	float vy,		// Y Speed
	float vz);		// Z Speed




int rc_mav_send_local_position_setpoint(
	float x,		// x position
	float y,		// y position
	float z,		// z position
	float yaw,		// Desired yaw angle
	uint8_t coordinate_frame);// Coordinate frame - valid values are only MAV_FRAME_LOCAL_NED or MAV_FRAME_LOCAL_ENU




int rc_mav_send_global_position_int(
	int32_t lat,		// Latitude, expressed as * 1E7
	int32_t lon,		// Longitude, expressed as * 1E7
	int32_t alt,		// Altitude in meters, expressed as * 1000 (millimeters), above MSL
	int32_t relative_alt,	// Altitude above ground in meters, expressed as * 1000 (millimeters)
	int16_t vx,		// Ground X Speed (Latitude), expressed as m/s * 100
	int16_t vy,		// Ground Y Speed (Longitude), expressed as m/s * 100
	int16_t vz,		// Ground Z Speed (Altitude), expressed as m/s * 100
	uint16_t hdg);		// Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX




int rc_mav_send_global_position_setpoint_int(
	int32_t latitude,		// Latitude (WGS84), in degrees * 1E7
	int32_t longitude,		// Longitude (WGS84), in degrees * 1E7
	int32_t altitude,		// Altitude (WGS84), in meters * 1000 (positive for up)
	int16_t yaw,			// Desired yaw angle in degrees * 100
	uint8_t coordinate_frame);	// Coordinate frame - valid values are only MAV_FRAME_GLOBAL or MAV_FRAME_GLOBAL_RELATIVE_ALT


int rc_mav_send_global_vision_position_estimate(
	uint64_t usec,		// Timestamp (microseconds, synced to UNIX time or since system boot)
	float x,		// Global X position
	float y,		// Global Y position
	float z,		// Global Z position
	float roll,		// Roll angle in rad
	float pitch,		// Pitch angle in rad
	float yaw);		// Yaw angle in rad



int rc_mav_send_battery(
	int32_t current_consumed,	// Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
	int32_t energy_consumed,	// Consumed energy, in 100*Joules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
	uint16_t voltage_cell_1,	// Battery voltage of cell 1, in millivolts (1 = 1 millivolt)
	uint16_t voltage_cell_2,	// Battery voltage of cell 2, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_3,	// Battery voltage of cell 3, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_4,	// Battery voltage of cell 4, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_5,	// Battery voltage of cell 5, in millivolts (1 = 1 millivolt), -1: no cell
	uint16_t voltage_cell_6,	// Battery voltage of cell 6, in millivolts (1 = 1 millivolt), -1: no cell
	int16_t current_battery,	// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	uint8_t accu_id,		// Accupack ID
	int8_t battery_remaining);	// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery


int rc_mav_send_gps_raw_int(
	int32_t lat,			// Latitude (WGS84), in degrees * 1E7
	int32_t lon,			// Longitude (WGS84), in degrees * 1E7
	int32_t alt,			// Altitude (WGS84), in meters * 1000 (positive for up)
	uint16_t eph,			// GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	uint16_t epv,			// GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX
	uint16_t vel,			// GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX
	uint16_t cog,			// Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX
	uint8_t fix_type,		// 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
	uint8_t satellites_visible);	// Number of satellites visible. If unknown, set to 255



int rc_mav_send_manual_control(
	int16_t x,		// X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	int16_t y,		// Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	int16_t z,		// Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
	int16_t r,		// R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	uint16_t buttons,	// A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	uint8_t target);	// The system to be controlled.




int rc_mav_send_rc_channels_scaled(
	int16_t chan1_scaled,	// RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	int16_t chan2_scaled,	// RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	int16_t chan3_scaled,	// RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	int16_t chan4_scaled,	// RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	int16_t chan5_scaled,	// RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	int16_t chan6_scaled,	// RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	int16_t chan7_scaled,	// RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	int16_t chan8_scaled,	// RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX.
	uint8_t port,		// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos.
	uint8_t rssi);		// Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown.








// receiving initialization
int rc_mav_init_listener(int port);
int rc_mav_cleanup_listener();


// assign a callback function to be called when a message is received.
int rc_mav_set_callback(int msg_id, void (*func)(void));

// returns the last received packet of type message
int rc_mav_is_new_message(int msg_id);
int rc_mav_get_message(int msg_id, mavlink_message_t* msg);
int64_t rc_mav_ns_since_last_message(int msg_id);

// extra helper functions for common packets so the user doesn't have to decode messages
int rc_mav_get_heartbeat_full(__mavlink_heartbeat_t* data);
int rc_mav_get_attitude(___mavlink_attitude_t* data);
int rc_mav_get_attitude_quaternion(___mavlink_attitude_quaternion_t* data);
int rc_mav_get__local_position_setpoint(___mavlink__local_position_setpoint_t* data);
int rc_mav_get__global_position_int(___mavlink__global_position_int_t* data);
int rc_mav_get__global_position_setpoint_int(___mavlink__global_position_setpoint_int_t* data);
int rc_mav_get__global_vision_position_estimate(___mavlink__global_vision_position_estimate_t* data);
int rc_mav_get__battery(___mavlink__battery_t* data);
int rc_mav_get__gps_raw_int(___mavlink__gps_raw_int_t* data);
int rc_mav_get__manual_control(___mavlink__manual_control_t* data);
int rc_mav_get__local_position_ned(___mavlink__local_position_ned_t* data);
int rc_mav_get__rc_channels_scaled(___mavlink__rc_channels_scaled_t* data);





#endif /* RC_MAVLINK_UDP */

