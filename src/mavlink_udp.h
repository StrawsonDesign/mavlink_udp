/*******************************************************************************
* mavlink_udp.h
*******************************************************************************/

#define _GNU_SORUCE // for pthread_timedjoin_np
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <sys/time.h>
#include <arpa/inet.h>	// Sockets & networking
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
//#include <perror.h>

#ifndef RC_MAVLINK_UDP
#define RC_MAVLINK_UDP

#include <stdint.h>		// for specific integer types
#include "../include/mavlink/common/mavlink.h" // MAVLINK
#include "../include/mavlink/mavlink_types.h" // MAVLINK

// default port for UDP communications
#define RC_MAV_GROUND_CONTROL_PORT	14550
#define RC_MAV_UAV_PORT			14551


// Sending Initialization
int rc_mav_init_sender(uint16_t port, const char* dest_ip, uint8_t system_id);
int rc_mav_set_dest_ip(const char* dest_ip);
int rc_mav_set_system_id(uint8_t system_id);
int rc_mav_cleanup_sender();



// Sending functions
// generic packet sending function
int rc_mav_send_msg(mavlink_message_t msg);

// helper functions for most common packets, removes the need for packing your own
int rc_mav_send_heartbeat_abbreviated();

int rc_mav_send_heartbeat(
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

int rc_mav_send_raw_pressure(
	int16_t press_abs,	// Absolute pressure (raw)
	int16_t press_diff1,	// Differential pressure 1 (raw)
	int16_t press_diff2,	// Differential pressure 2 (raw)
	int16_t temperature);	// Raw Temperature measurement (raw))

int rc_mav_send_servo_output_raw(
	uint16_t servo1_raw,	// Servo output 1 value, in microseconds
	uint16_t servo2_raw,	// Servo output 2 value, in microseconds
	uint16_t servo3_raw,	// Servo output 3 value, in microseconds
	uint16_t servo4_raw,	// Servo output 4 value, in microseconds
	uint16_t servo5_raw,	// Servo output 5 value, in microseconds
	uint16_t servo6_raw,	// Servo output 6 value, in microseconds
	uint16_t servo7_raw,	// Servo output 7 value, in microseconds
	uint16_t servo8_raw,	// Servo output 8 value, in microseconds
	uint8_t port);		// Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos.

int rc_mav_send_sys_status(
	uint32_t onboard_control_sensors_present,// Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	uint32_t onboard_control_sensors_enabled,// Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	uint32_t onboard_control_sensors_health,// Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
	uint16_t load,				// Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
	uint16_t voltage_battery,		// Battery voltage, in millivolts (1 = 1 millivolt)
	int16_t current_battery,		// Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
	uint16_t drop_rate_comm,		// Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	uint16_t errors_comm,			// Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
	uint16_t errors_count1,			// Autopilot-specific errors
	uint16_t errors_count2,			// Autopilot-specific errors
	uint16_t errors_count3,			// Autopilot-specific errors
	uint16_t errors_count4,			// Autopilot-specific errors
	int8_t battery_remaining);		// Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery

int rc_mav_att_pos_mocap(
	float q[4],	// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	float x,	// X position in meters (NED)
	float y,	// Y position in meters (NED)
	float z);	// Z position in meters (NED)



// receiving initialization
int rc_mav_init_listener(uint16_t port);
int rc_mav_cleanup_listener();

// assign a callback function to be called when a message is received.
int rc_mav_set_callback(int msg_id, void (*func)(void));
int rc_mav_set_callback_all(void (*func)(void));

// returns the last received packet of type message
int rc_mav_is_new_msg(int msg_id);
int rc_mav_get_msg(int msg_id, mavlink_message_t* msg);
int64_t rc_mav_ns_since_last_msg(int msg_id);
int rc_mav_print_msg_name(int msg_id);
int rc_mav_id_of_last_msg();
/*
// extra helper functions for common packets so the user doesn't have to decode messages
int rc_mav_get_heartbeat_full(__mavlink_heartbeat_t* data);
int rc_mav_get_attitude(___mavlink_attitude_t* data);
int rc_mav_get_attitude_quaternion(___mavlink_attitude_quaternion_t* data);
int rc_mav_get_local_position_setpoint(___mavlink__local_position_setpoint_t* data);
int rc_mav_get_global_position_int(___mavlink__global_position_int_t* data);
int rc_mav_get_global_position_setpoint_int(___mavlink__global_position_setpoint_int_t* data);
int rc_mav_get_global_vision_position_estimate(___mavlink__global_vision_position_estimate_t* data);
int rc_mav_get_battery(___mavlink__battery_t* data);
int rc_mav_get_gps_raw_int(___mavlink__gps_raw_int_t* data);
int rc_mav_get_manual_control(___mavlink__manual_control_t* data);
int rc_mav_get_local_position_ned(___mavlink__local_position_ned_t* data);
int rc_mav_get_rc_channels_scaled(___mavlink__rc_channels_scaled_t* data);
int rc_mav_get_raw_pressure(___mavlink___raw_pressure_t* data);
int rc_mav_get_servo_output_raw(__mavlink__servo_output_raw_t* data);
int rc_mav_get_sys_status(__mavlink_sys_status_t);
int rc_mav_get_att_pos_mocap(__mavlink_att_pos_mocap_t);
*/
#endif /* RC_MAVLINK_UDP */

