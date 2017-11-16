/*******************************************************************************
* mavlink_udp.c
*******************************************************************************/

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

#include "mavlink/mavlink.h" // MAVLINK

// Networking Macros
#define SOCK_BUF_LEN MAVLINK_MAX_PACKET_LEN
#define RC_MAV_DEFAULT_PORT 8888

// MAVLINK Macros
#define RC_MAV_SYSTEM_ID	1	// CHANGE THIS LATER
#define RC_MAV_TARGET_ID	1	// CHANGE THIS LATER (AND REPLACE IT)
#define RC_MAV_COMPONENT_ID	200	// CHANGE THIS LATER
#define RC_MAV_TYPE		13	// Hexarotor
#define RC_MAV_AUTOPILOT	8	// Invalid Autopilot
#define RC_MAV_MODE		0	// No mode -- CHANGE THIS LATER
#define RC_MAV_STATE		3	// Standby -- CHANGE THIS LATER

#define RC_SERVER_SLEEP_TIME	100000	// 100 ms


// placeholder for button functions
void rc_null_func();

// function pointers for button handlers
void (*pause_pressed_func)(void)	= &rc_null_func;

void  heartbeat_func;
attitude_func_t attitude_func;
control_func_t control_func;


// Private Function Declarations
rc_mavlink_socket_t* _rc_mav_init_common();
void* _rc_mav_listen_for_udp(rc_mavlink_socket_t* sock);
int _rc_mav_handle_mav_packet(rc_mavlink_socket_t* sock);
int _rc_mav_interpret_udp_packet(rc_mavlink_socket_t* sock);
int _rc_mav_send_message(rc_mavlink_socket_t* sock, mavlink_message_t* msg);


/*******************************************************************************
* @ void rc_null_func()
*
* A simple function that just returns. This exists so function pointers can be
* set to do nothing such as button and imu interrupt handlers.
*******************************************************************************/
void rc_null_func(){
	return;
}



// Function Definitions
rc_mavlink_socket_t* _rc_mav_init_common(){
	// Allocate memory for socket, and zero/nullset all of it
	rc_mavlink_socket_t* sock = (rc_mavlink_socket_t*)calloc(1, sizeof(rc_mavlink_socket_t));

	// Set socket length
	sock->sock_len = sizeof(sock->sock_me);

	// Create the UDP socket
	if ((sock->sock_fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) < 0){
		die("socket");
		return NULL;
	}

	return sock;
}

rc_mavlink_socket_t* rc_mav_init_client(int port, char* dest_ip){
	// Common socket setup
	rc_mavlink_socket_t* sock = _rc_mav_init_common();

	if (sock == NULL){
		printf("ERROR: Could not create RC UDP socket\n");
		return NULL;
	}

	// Convert IP Address
	if (inet_aton(dest_ip, &(sock->sock_other.sin_addr)) == 0){
		die("inet_aton");
		return NULL;
	}

	// Address config
	sock->sock_other.sin_family = AF_INET;
	sock->sock_other.sin_port = htons(port);

	// Mark initialized
	sock->initialized = 1;

	return sock;
}

rc_mavlink_socket_t* rc_mav_init_server(int port){
	// Common socket setup
	rc_mavlink_socket_t* sock = _rc_mav_init_common();

	if (sock == NULL){
		printf("ERROR: Could not create RC UDP socket\n");
		return NULL;
	}

	// Address config
	sock->sock_me.sin_family = AF_INET;
	sock->sock_me.sin_port = htons(port);
	sock->sock_me.sin_addr.s_addr = htonl(INADDR_ANY);

	// Bind the socket
	if (bind(sock->sock_fd, &(sock->sock_me), sizeof(sock->sock_me)) != 0){
		die("bind");
		return NULL;
	}

	// Mark the socket non-blocking
	// Otherwise the pthread blocks and never gets to usleep to yield time
	if (fcntl(sock->sock_fd, F_SETFL, O_NONBLOCK) != 0){
		die("fctnl (setting non-blocking flag on socket)");
		return NULL;
	}

	// Mark initialized
	sock->initialized = 1;

	return sock;
}


int rc_mav_cleanup_socket(rc_mavlink_socket_t* sock){
	if (sock->initialized){
		close(sock->sock_fd);
		sock->initialized = 0;
		free(sock);
		return 0;
	}else{
		printf("ERR: Socket not initialized or already cleaned up.\n");
		return -1;
	}
}

int _rc_mav_send_message(rc_mavlink_socket_t* sock, mavlink_message_t* msg){
	if (!sock->initialized){
		printf("ERR: Socket not initialized! Cannot send message.\n");
		return -1;
	}

	sock->last_msg_len = mavlink_msg_to_send_buffer(sock->mav_buf, msg);
	if (sock->last_msg_len < 0){
		die("mavlink_msg_to_send_buffer");
	}

	int bytes_sent = sendto(sock->sock_fd, sock->mav_buf, sock->last_msg_len,
							0, &(sock->sock_other), sock->sock_len);
	if (bytes_sent < 0){
		die("sendto");
	}

	printf("Sent MAV message with length: %d.\n", sock->last_msg_len);
	printf("Data: %s\n", sock->mav_buf);

	// Clear the buffer
	memset(sock->mav_buf, 0, SOCK_BUF_LEN);

	return 0;
}






////////////////////////////////////////////////////////////////////////////////
//  LISTENING STUFF
////////////////////////////////////////////////////////////////////////////////

int rc_mav_init_listener(int port){
	pthread_t* listener_thread = (pthread_t*)malloc(sizeof(pthread_t));
	pthread_create(listener_thread, NULL, &_rc_mav_listen_for_udp, NULL);
	return listener_thread;
}

int rc_mav_cleanup_listener(pthread_t* listener_thread){
	pthread_join(*listener_thread, NULL);
	free(listener_thread);
	return 0;
}




// local thread blocking on UDP socket
// exits on shutdown flag
void* udp_listener(){
	while(shutdown_flag == 0){

		// Read message
		sock->last_msg_len = recvfrom(sock->sock_fd, sock->sock_buf, SOCK_BUF_LEN,
					0, &(sock->sock_other), &(sock->sock_len));

		// Handle message
		if (sock->last_msg_len != 0){
			if (errno == ETIMEDOUT){
				// This error is ignorable.
			}else if (errno == EAGAIN || errno == EWOULDBLOCK){
				// This error tells us that recvfrom would have blocked, but didn't.
				// In other words, it is ignorable, but it also means that we did not
				// receive a packet so we should continue rather than interpreting.
				continue;
			}else{
				// Print any other errors.
				die("recvfrom");
				continue;
			}
		}
		handle_mav_packet();
	}
	return 0;
}

int handle_mav_packet(rc_mavlink_socket_t* sock){
	//uint8_t c = sock->sock_buf[0];
	mavlink_message_t msg;
	mavlink_status_t status;
	int msg_received = 0;

	for (int i = 0; i < sock->last_msg_len; ++i){
		msg_received = mavlink_parse_char(0, sock->sock_buf[i], &msg, &status);

		if (msg_received > 0){
			//uint8_t msg_buf[MAVLINK_MAX_PACKET_LEN];
			//unsigned int messageLength = mavlink_msg_to_send_buffer(msg_buf, &message)
			switch (msg.msgid){
				case MAVLINK_MSG_ID_HEARTBEAT:{
					mavlink_heartbeat_t pk;
					mavlink_msg_heartbeat_decode(&msg, &pk);
					//					printf("[DEBUG] Got HEARTBEAT message:\n");
					if (sock->heartbeat_func != NULL){
						sock->heartbeat_func();
					}
					break;
				}
				case MAVLINK_MSG_ID_ATTITUDE:{
					mavlink_attitude_t pk;
					mavlink_msg_attitude_decode(&msg, &pk);
					//					printf("[DEBUG] Got ATTITUDE message:\n");
					if (sock->attitude_func != NULL){
						sock->attitude_func(pk.roll, pk.pitch, pk.yaw, pk.rollspeed,
											pk.pitchspeed, pk.yawspeed);
					}
					break;
				}
				case MAVLINK_MSG_ID_MANUAL_CONTROL:{
					mavlink_manual_control_t pk;
					mavlink_msg_manual_control_decode(&msg, &pk);
					//					printf("[DEBUG] Got CONTROL message:\n");
					if (sock->control_func != NULL){
						sock->control_func(pk.x, pk.y, pk.z, pk.r, pk.buttons);
					}
					break;
				}
			}
		}
	}

	if (msg_received == 0){
		printf("ERROR: INVALID MESSAGE WITH ERROR %d\n", msg_received);
	}

	return 0;
}

// Packing Methods

int rc_mav_send_heartbeat(rc_mavlink_socket_t* sock){
	// Allocate memory for message
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(RC_MAV_SYSTEM_ID, RC_MAV_COMPONENT_ID, &msg,
							   RC_MAV_TYPE, RC_MAV_AUTOPILOT, RC_MAV_MODE,
							   RC_MAV_MODE, RC_MAV_STATE);
	if(_rc_mav_send_message(sock, &msg) != 0){
		die("send_mav_message");
		return -1;
	}
	return 0;
}

int rc_mav_send_attitude(rc_mavlink_socket_t* sock, float roll, float pitch, float yaw,
						 float rollspeed, float pitchspeed, float yawspeed){
	// Allocate memory for message
	mavlink_message_t msg;
	struct timespec time;
	if (clock_gettime(CLOCK_REALTIME, &time) != 0){
		die("clock_gettime");
	}
	mavlink_msg_attitude_pack(RC_MAV_SYSTEM_ID, RC_MAV_COMPONENT_ID, &msg,
							  time.tv_sec, roll, pitch, yaw, rollspeed,
							  pitchspeed, yawspeed);

	if(_rc_mav_send_message(sock, &msg) != 0){
		die("send_mav_message");
		return -1;
	}
	return 0;
}

int rc_mav_send_control(rc_mavlink_socket_t* sock, int16_t x, int16_t y, int16_t z,
						int16_t r, uint16_t buttons){
	// Allocate memory for message
	mavlink_message_t msg;
	mavlink_msg_manual_control_pack(RC_MAV_SYSTEM_ID, RC_MAV_COMPONENT_ID, &msg,
									RC_MAV_TARGET_ID, x, y, z, r, buttons);

	if(_rc_mav_send_message(sock, &msg) != 0){
		die("send_mav_message");
		return -1;
	}
	return 0;
}

int rc_mav_set_heartbeat_func(rc_mavlink_socket_t* sock, int (*func)(void)){
	sock->heartbeat_func = func;
	return 0;
}

int rc_mav_set_attitude_func(rc_mavlink_socket_t* sock,
							 int (*func)(float, float, float, float, float, float)){
	sock->attitude_func = func;
	return 0;
}

int rc_mav_set_control_func(rc_mavlink_socket_t* sock, control_func_t func){
	sock->control_func = func;
	return 0;
}




//int print_raw_udp_packet(){
//	// Print results
//	printf("Received packet from %s:%d\nData: %s\n",
//		   inet_ntoa(sock_other.sin_addr), ntohs(sock_other.sin_port), buf);
//	fflush(stdout);
//
//	// Clear the buffer
//	memset(buf,0,SOCK_BUF_LEN);
//
//	return 0;
//}
