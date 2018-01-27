/*******************************************************************************
* mavlink_udp.h
*******************************************************************************/

#ifndef RC_MAVLINK_UDP
#define RC_MAVLINK_UDP

#define _GNU_SOURCE	// for pthread_timedjoin_np
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>	// for specific integer types
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <sys/time.h>
#include <arpa/inet.h>	// Sockets & networking
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>


#include <rc/mavlink/common/mavlink.h> // MAVLINK
#include <rc/mavlink/mavlink_types.h> // MAVLINK

// default port for UDP communications
#define RC_MAV_DEFAULT_UDP_PORT		14551



/**
 * @brief Initialize a UDP port for sending and receiving
 *
 * Initialize a UDP port for sending and receiving. Additionally starts a listening
 * thread that handles incomming packets and makes them available with the remaining
 * functions in this API.
 *
 * @param[in]  system_id    The system id of this device tagged in outgoing packets
 * @param[in]  dest_ip      The destination ip, can be changed later with rc_mav_set_dest_ip
 * @param[in]  port         Port to listen on and send to
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_init(uint8_t system_id, const char* dest_ip, uint16_t port);


int rc_mav_set_dest_ip(const char* dest_ip);

/**
 * @brief      { function_description }
 *
 * @param[in]  system_id  The system identifier
 *
 * @return     { description_of_the_return_value }
 */
int rc_mav_set_system_id(uint8_t system_id);

int rc_mav_cleanup();




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


// attitude telemetry packets
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


// position telemetry packets
int rc_mav_send_local_position_ned(
	float x,		// X Position
	float y,		// Y Position
	float z,		// Z Position
	float vx,		// X Speed
	float vy,		// Y Speed
	float vz);		// Z Speed

int rc_mav_send_global_position_int(
	int32_t lat,		// Latitude, expressed as * 1E7
	int32_t lon,		// Longitude, expressed as * 1E7
	int32_t alt,		// Altitude in meters, expressed as * 1000 (millimeters), above MSL
	int32_t relative_alt,	// Altitude above ground in meters, expressed as * 1000 (millimeters)
	int16_t vx,		// Ground X Speed (Latitude), expressed as m/s * 100
	int16_t vy,		// Ground Y Speed (Longitude), expressed as m/s * 100
	int16_t vz,		// Ground Z Speed (Altitude), expressed as m/s * 100
	uint16_t hdg);		// Compass heading in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX

// setpoint packets
int rc_mav_send_set_position_target_local_ned(
	float x, /*< X Position in NED	frame in meters*/
	float y, /*< Y Position in NED	frame in meters*/
	float z, /*< Z Position in NED	frame in meters (note, altitude is negative in NED)*/
	float vx, /*< X velocity in NED	frame in meter / s*/
	float vy, /*< Y velocity in NED	frame in meter / s*/
	float vz, /*< Z velocity in NED	frame in meter / s*/
	float afx, /*< X acceleration or	force (if bit 10 of type_mask is set) in NED	frame in meter / s^2 or N*/
	float afy, /*< Y acceleration or	force (if bit 10 of type_mask is set) in NED	frame in meter / s^2 or N*/
	float afz, /*< Z acceleration or	force (if bit 10 of type_mask is set) in NED	frame in meter / s^2 or N*/
	float yaw, /*< yaw setpoint in rad*/
	float yaw_rate, /*< yaw rate setpoint in rad/s*/
	uint16_t type_mask, /*< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the	floats afx afy afz should be interpreted as	force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is	force setpoint, bit 11: yaw, bit 12: yaw rate*/
	uint8_t target_system, /*< System ID*/
	uint8_t target_component, /*< Component ID*/
	uint8_t coordinate_frame); /*< Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9*/


int rc_mav_send_set_position_target_global_int(
	int32_t lat_int, /*< X Position in WGS84	frame in 1e7 * degrees*/
	int32_t lon_int, /*< Y Position in WGS84	frame in 1e7 * degrees*/
	float alt, /*< Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT*/
	float vx, /*< X velocity in NED	frame in meter / s*/
	float vy, /*< Y velocity in NED	frame in meter / s*/
	float vz, /*< Z velocity in NED	frame in meter / s*/
	float afx, /*< X acceleration or	force (if bit 10 of type_mask is set) in NED	frame in meter / s^2 or N*/
	float afy, /*< Y acceleration or	force (if bit 10 of type_mask is set) in NED	frame in meter / s^2 or N*/
	float afz, /*< Z acceleration or	force (if bit 10 of type_mask is set) in NED	frame in meter / s^2 or N*/
	float yaw, /*< yaw setpoint in rad*/
	float yaw_rate, /*< yaw rate setpoint in rad/s*/
	uint16_t type_mask, /*< Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the	floats afx afy afz should be interpreted as	force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is	force setpoint, bit 11: yaw, bit 12: yaw rate*/
	uint8_t coordinate_frame); /*< Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11*/


// other telemetry (MAV to Ground)
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


// other control (Ground to MAV)
int rc_mav_send_manual_control(
	int16_t x,		// X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle.
	int16_t y,		// Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle.
	int16_t z,		// Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle.
	int16_t r,		// R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle.
	uint16_t buttons,	// A bitfield corresponding to the joystick buttons' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1.
	uint8_t target);	// The system to be controlled.


int rc_mav_att_pos_mocap(
	float q[4],	// Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
	float x,	// X position in meters (NED)
	float y,	// Y position in meters (NED)
	float z);	// Z position in meters (NED)



// receiving initialization


// assign a callback function to be called when a message is received.
int rc_mav_set_callback(int msg_id, void (*func)(void));
int rc_mav_set_callback_all(void (*func)(void));
int rc_mav_set_connection_lost_callback(uint64_t timeout_ns);
int rc_mav_get_connection_state();

// returns the last received packet of type message
int rc_mav_is_new_msg(int msg_id);
int rc_mav_get_msg(int msg_id, mavlink_message_t* msg);
int64_t rc_mav_ns_since_last_msg(int msg_id);
int64_t rc_mav_ns_since_last_msg_any(int msg_id);
int rc_mav_print_msg_name(int msg_id);
int rc_mav_id_of_last_msg();




// extra helper functions for common packets so the user doesn't have to decode messages
int rc_mav_get_heartbeat(mavlink_heartbeat_t* data);

// attitude telemetry packets
int rc_mav_get_attitude(mavlink_attitude_t* data);
int rc_mav_get_attitude_quaternion(mavlink_attitude_quaternion_t* data);

// position telemetry packets
int rc_mav_get_local_position_ned(mavlink_local_position_ned_t* data);
int rc_mav_get_global_position_int(mavlink_global_position_int_t* data);

// setpoint packets
int rc_mav_get_set_position_target_local_ned(mavlink_set_position_target_local_ned_t* data);
int rc_mav_get_set_position_target_global_int(mavlink_set_position_target_global_int_t* data);

// other telemetry
int rc_mav_get_gps_raw_int(mavlink_gps_raw_int_t* data);
int rc_mav_get_raw_pressure(mavlink_raw_pressure_t* data);
int rc_mav_get_servo_output_raw(mavlink_servo_output_raw_t* data);
int rc_mav_get_sys_status(mavlink_sys_status_t* data);

// other control (Ground to MAV)
int rc_mav_get_manual_control(mavlink_manual_control_t* data);
int rc_mav_get_att_pos_mocap(mavlink_att_pos_mocap_t* data);

#endif /* RC_MAVLINK_UDP */

