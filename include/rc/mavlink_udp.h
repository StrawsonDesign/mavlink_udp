/**
 * @file mavlink_udp.h
 *
 * @brief      C interface for communicating with mavlink over UDP in
 *             Linux/Windows
 *
 *             Uses common mavlink v2 packets generated from official mavlink
 *             source (https://github.com/mavlink/mavlink). Also see
 *             mavlink_udp_helpers.h for additional helper functions for most
 *             commonly used packets.
 *
 * @author     James Strawson & Henry Gaudet
 *
 * @date       1/24/2018
 */

#ifndef RC_MAVLINK_UDP_H
#define RC_MAVLINK_UDP_H


#include <stdint.h>	// for specific integer types
// these are directly from the mavlink source
#include <rc/mavlink/common/mavlink.h>
#include <rc/mavlink/mavlink_types.h>
#include <rc/mavlink/mavlink_udp_helpers.h>


#define RC_MAV_DEFAULT_UDP_PORT	14551


/**
 * @brief      Initialize a UDP port for sending and receiving.
 *
 *             Initialize a UDP port for sending and receiving. Additionally
 *             starts a listening thread that handles incomming packets and
 *             makes them available with the remaining functions in this API.
 *
 * @param[in]  system_id  The system id of this device tagged in outgoing
 *                        packets
 * @param[in]  dest_ip    The destination ip, can be changed later with
 *                        rc_mav_set_dest_ip
 * @param[in]  port       Port to listen on and send to
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_init(uint8_t system_id, const char* dest_ip, uint16_t port);

/**
 * @brief      Sets the destination ip address for sent packets.
 *
 *
 *
 * @param[in]  dest_ip  The destination ip
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_set_dest_ip(const char* dest_ip);

/**
 * @brief      Sets the system identifier
 *
 *             The system_id is included with every mavlink packet sent so that
 *             the receiver can identitfy what sent the current packet. We
 *             suggest every mavlink device in a network to have a unique id.
 *             The system_id is set during rc_mav_init but can be changed
 *             afterwards with this function.
 *
 * @param[in]  system_id  The system identifier
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_set_system_id(uint8_t system_id);



/**
 * @brief      Closes UDP port and stops network port listening thread
 *
 *             This is a blocking function call that returns after the network
 *             port listening thread has exited and the UDP port has closed. If
 *             the thread fails to exit before timeout a warning message is
 *             displayed and the function returns -1 to indicate there was an
 *             issue. This should be called before your program exits.
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_cleanup();




/**
 * @brief      Sends any user-packed mavlink message
 *
 *             To construct your own mavlink_message_t from the packet
 *             definitions in include/rc/mavlink/common/, follow this example
 *             snippet and substitute in the functions for packing the packet
 *             you wish to send. rc/mavlink_udp_helpers.h provides many helper
 *             functions to pack and send the most common  mavlink packets so
 *             that you do not need to use this function always.
 *
 * @code{.c}
 * mavlink_message_t msg;
 * mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_ALL, &msg, 0, 0, 0, 0, 0);
 * rc_mav_send_msg(msg)){
 * @endcode
 *
 * @param[in]  msg   The message to be sent
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_send_msg(mavlink_message_t msg);


/**
 * @brief      assign a callback function to be called when a particular message
 *             is received.
 *
 *             If a general callback function has been set with
 *             rc_mav_set_callback_all, this message-specific callback will be
 *             called after the general callback.
 *
 * @param[in]  msg_id  The message identifier
 * @param[in]  func    The callabck function pointer
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_set_callback(int msg_id, void (*func)(void));



/**
 * @brief      Sets a callback function which is called when any packet arrives.
 *
 *             If a message-specific callback function has been set with
 *             rc_mav_set_callback(), this general callback will be called
 *             before the message-specific callback.
 *
 * @param[in]  func  The callback function
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_set_callback_all(void (*func)(void));



/**
 * @brief      Sets a callback function to be called when any heartbeat signal
 *             has not been received for 3 seconds indicating a likely dropped
 *             connection.
 *
 * @param[in]  func  The callback function
 *
 * @return     0 on success, -1 on failure
 */
int rc_mav_set_connection_lost_callback(void (*func)(void));



/**
 * @brief      Gets the connection state as determined by received heartbeat
 *             packets.
 *
 *             Mavlink uses heartbeat packets generally sent at 1s intervals to
 *             determine if an active connection is open. This function will
 *             return 0 until a heartbeat packet has been received this will
 *             return 1. The connection state will transition to lost (0) if 3
 *             seconds passes without receiving a heartbeat packet.
 *
 * @return     Returns 1 if connection is active, 0 otherwise.
 */
int rc_mav_get_connection_state();

/**
 * @brief      Inidcates if a particular message type has been received by not
 *             read by the user yet.
 *
 * @param[in]  msg_id  The message identifier to check
 *
 * @return     1 if new message is available, otherwise 0
 */
int rc_mav_is_new_msg(int msg_id);

/**
 * @brief      Fetches the last received message of type msg_id
 *
 * @param[in]  msg_id  The message identifier to fetch
 * @param[out] msg     place to write to message struct to
 *
 * @return     returns 0 on success. Returns -1 on failure, for example if no
 *             message of type msg_id has been received.
 */
int rc_mav_get_msg(int msg_id, mavlink_message_t* msg);

/**
 * @brief      Fetches the system ID of the sender of the last received message.
 *             of type msg_id
 *
 * @param[in]  msg_id  The message identifier of the packet type to check
 *
 * @return     Returns the system ID on success. Returns -1 on failure, for
 *             example if no message of type msg_id has been received of type
 *             msg_id.
 */
int rc_mav_get_sys_id_of_last_msg(int msg_id);


/**
 * @brief      Fetches the system ID of the sender of the last received message.
 *
 * @return     Returns the system ID on success. Returns -1 on failure, for
 *             example if no message has been received.
 */
int rc_mav_get_sys_id_of_last_msg_any();

/**
 * @brief      Fetches the number of nanoseconds since the last message of type
 *             msg_id has been received.
 *
 * @param[in]  msg_id  The message identifier of the packet type to check
 *
 * @return     Returns the number of nanoseconds since the last message of type
 *             msg_id has been received. Returns -1 on failure, for
 *             example if no message has been received of type msg_id.
 */
int64_t rc_mav_ns_since_last_msg(int msg_id);

/**
 * @brief      Fetches the number of nanoseconds since any packet has been
 *             received.
 *
 * @return     Returns the number of nanoseconds since any message has been
 *             received. Returns -1 on failure, for example if no message has
 *             been received.
 */
int64_t rc_mav_ns_since_last_msg_any();

/**
 * @brief      Returns the msg_id of the last received packet.
 *
 * @return     Returns the msg_id of the last received packet. Returns -1 on
 *             failure, for example if no message has been received.
 */
int rc_mav_id_of_last_msg();

/**
 * @brief      Prints to stdout a human-readable name for a message type.
 *
 * @param[in]  msg_id  The message identifier to print
 *
 * @return     Returns 0 on success, or -1 on failure such as if msg_id is invalid.
 */
int rc_mav_print_msg_name(int msg_id);






#endif /* RC_MAVLINK_UDP_H */

