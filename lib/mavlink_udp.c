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
#include <perror.h>

#include "mavlink/mavlink.h" // MAVLINK

#define BUFFER_LENGTH 512 // common networking buffer size
#define MAX_UNIQUE_MSG_TYPES 256
#define SOCKET_TIMEOUT

// function declarations
static void rc_null_func();
static void* listening_func(void*);

// sending stuff
send_init_flag=0;
int send_sock_fd;
struct sockaddr_in send_sock_addr;
uint8_t current_system_id;


// listening stuff
void callbacks(void*)[MAX_UNIQUE_MSG_TYPES];
int received_flag[MAX_UNIQUE_MSG_TYPES];
uint64_t ns_of_last_msg[MAX_UNIQUE_MSG_TYPES];
mavlink_message_t messages[MAX_UNIQUE_MSG_TYPES];
pthread_t listen_thread;
int shutdown_flag=0;
int listening_flag=0;



/*******************************************************************************
* void null_func()
*
* A simple function that just returns. This exists so callback pointers can be
* set to do nothing
*******************************************************************************/
static void null_func(){
	return;
}


int rc_mav_init_sender(uint16_t port, const char* dest_ip, uint8_t system_id){
	// sanity checks
	if(sending_init_flag!=0){
		fprintf(stderr, "WARNING, trying to init mavlink sender when it's already initialized\n");
		return -1;
	}
	if(dest_ip==NULL){
		fprintf(stderr, "ERROR: in rc_mav_init_sender received NULL dest_ip string\n");
		return -1;
	}
	// open socket for UDP packets
	if((send_sock_fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))<0){
		perror("ERROR: in rc_mav_init_sender:");
		return -1;
	}
	// fill out rest of sockaddr_in struct
	send_sock_addr.sin_family = AF_INET;
	send_sock_addr.sin_port = htons(port);
	// set address
	if(rc_mav_set_dest_ip(dest_ip)){
		return -1;
	}
	send_init_flag=1;
	current_system_id = system_id;
	return 0;
}

int rc_mav_set_dest_ip(const char* dest_ip){
	// sanity check
	if(dest_ip==NULL){
		fprintf(stderr, "ERROR: in rc_mav_set_dest_ip received NULL dest_ip string\n");
		return -1;
	}
	// convert ip string to byte order, works for ipv4 & v6
	if(inet_pton(AF_INET,dest_ip, &send_sock_addr.sin_addr)==0){
		perror("ERROR: in rc_mav_set_dest_ip:");
		return NULL;
	}
	return 0;
}

int rc_mav_set_system_id(uint8_t system_id){
	current_system_id = system_id;
	return 0;
}

int rc_mav_cleanup_sender(){
	close(send_sock_fd);
	send_init_flag=0;
	return 0;
}



// generic packet sending function
int rc_mav_send_msg(mavlink_message_t msg){
	if(send_init_flag==0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, sender not initialized\n");
		return -1;
	}
	uint8_t buf[BUFFER_LENGTH];
	int msg_len = mavlink_msg_to_send_buffer(buf, msg);
	if(msg_len<0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, unable to pack message for sending\n");
		return -1;
	}
	int bytes_sent = sendto(send_sock_fd, buf, msg_len, 0, &send_sock_addr,
						sizeof(struct sockaddr_in));
	if(bytes_sent!=msg_len){
		perror("ERROR: in rc_mav_send_msg: ");
		return -1;
	}
	return 0;
}

int rc_mav_send_heartbeat_full(uint32_t custom_mode,uint8_t type,uint8_t autopilot,
				uint8_t base_mode, uint8_t system_status){
	// sanity check
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
	if(rc_mav_send_msg(msg)){
		fprintf()
		return -1;
	}

}


/// listening stuff
	// bind port
	if(bind(send_sock_fd, &send_sock_addr, sizeof(struct sockaddr_in))){
		perror("ERROR: in rc_mav_set_dest_ip bind:");
		return -1;
	}
