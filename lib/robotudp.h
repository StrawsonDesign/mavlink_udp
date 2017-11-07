//
//  robotudp.h
//  robot-udp
//
//  Created by Trevor Irwin on 3/28/17.
//  Copyright © 2017 Trevor Irwin. All rights reserved.
//

#ifndef robotudp_h
#define robotudp_h

#include <stdio.h>  // Various standard libraries
#include <stdlib.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <sys/time.h>

#include <arpa/inet.h> // Sockets & networking
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>

#include "mavlink/mavlink.h" // MAVLINK

#include "common.h" // Custom error functions & roboticscape placeholder

// Networking Macros
#define SOCK_BUF_LEN MAVLINK_MAX_PACKET_LEN
#define RC_MAV_DEFAULT_PORT 8888

// MAVLINK Macros
#define RC_MAV_SYSTEM_ID 1          // CHANGE THIS LATER
#define RC_MAV_TARGET_ID 1          // CHANGE THIS LATER (AND REPLACE IT)
#define RC_MAV_COMPONENT_ID 200     // CHANGE THIS LATER
#define RC_MAV_TYPE 13              // Hexarotor
#define RC_MAV_AUTOPILOT 8          // Invalid Autopilot
#define RC_MAV_MODE 0               // No mode -- CHANGE THIS LATER
#define RC_MAV_STATE 3              // Standby -- CHANGE THIS LATER

#define RC_SERVER_SLEEP_TIME 100000 // 100 ms

// Types
typedef int (*heartbeat_func_t)();
typedef int (*attitude_func_t)(float, float, float, float, float, float);
typedef int (*control_func_t)(int16_t, int16_t, int16_t, int16_t, uint16_t);

// TO-DO - Add socket type (server/client) and include checks in each
// method to ensure user does not attempt to e.g. send messages from a server
typedef struct rc_mavlink_socket_t{
    int initialized;
    int sock_fd;
    int recv_len;
    int last_msg_len;
    socklen_t sock_len;
    struct sockaddr_in sock_me;
    struct sockaddr_in sock_other;
    uint8_t mav_buf[MAVLINK_MAX_PACKET_LEN];
    char sock_buf[SOCK_BUF_LEN];
    heartbeat_func_t heartbeat_func;
    attitude_func_t attitude_func;
    control_func_t control_func;
} rc_mavlink_socket_t;

// Public Function Declarations

// Setup functions
rc_mavlink_socket_t* rc_mav_init_client(int port, char* dest_ip);
rc_mavlink_socket_t* rc_mav_init_server(int port);
pthread_t* rc_mav_init_listener(rc_mavlink_socket_t* sock);

// Cleanup functions
int rc_mav_cleanup_listener(pthread_t* listener_thread);
int rc_mav_cleanup_socket(rc_mavlink_socket_t* sock);

// Send message functions
int rc_mav_send_heartbeat(rc_mavlink_socket_t* sock);
int rc_mav_send_attitude(rc_mavlink_socket_t* sock, float roll, float pitch, float yaw,
                         float rollspeed, float pitchspeed, float yawspeed);
int rc_mav_send_control(rc_mavlink_socket_t* sock, int16_t x, int16_t y, int16_t z,
                        int16_t r, uint16_t buttons);

// Bind "interrupt" functions (not actually interrupts, but they are called when
// a listener thread receives the relevant packet
int rc_mav_set_heartbeat_func(rc_mavlink_socket_t* sock, int (*func)(void));
int rc_mav_set_attitude_func(rc_mavlink_socket_t* sock,
                             int (*func)(float, float, float, float, float, float));
int rc_mav_set_control_func(rc_mavlink_socket_t* sock, control_func_t func);



#endif /* robotudp_h */

