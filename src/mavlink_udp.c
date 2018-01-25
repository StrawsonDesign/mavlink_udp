/*******************************************************************************
* mavlink_udp.c
*******************************************************************************/
#include "mavlink_udp.h"


#define BUFFER_LENGTH		512 // common networking buffer size
#define MAX_UNIQUE_MSG_TYPES	256
#define LISTEN_SOCKET_TIMEOUT	1
#define MAX_PENDING_CONNECTIONS	32

// function declarations


// sending stuff
static int sending_init_flag=0;
static int send_sock_fd;
static struct sockaddr_in send_sock_addr;
static uint8_t current_system_id;


// listening stuff
static void (*callbacks[MAX_UNIQUE_MSG_TYPES])(void);
static void (*callback_all)(void); // called when any packet arrives
static int received_flag[MAX_UNIQUE_MSG_TYPES];
static uint64_t ns_of_last_msg[MAX_UNIQUE_MSG_TYPES];
static mavlink_message_t messages[MAX_UNIQUE_MSG_TYPES];
static pthread_t listen_thread;
static int listen_shutdown_flag=0;
static int listening_flag=0;
static int listening_init_flag=0;


/*******************************************************************************
* void rc_null_func()
*
* A simple function that just returns. This exists so callback pointers can be
* set to do nothing
*******************************************************************************/
static void rc_null_func(){
	return;
}

/*******************************************************************************
* @ uint64_t rc_nanos_since_epoch()
*
* Returns the number of nanoseconds since epoch using system CLOCK_REALTIME
* This function itself takes about 1100ns to complete at 1ghz under ideal
* circumstances.
*******************************************************************************/
static uint64_t rc_nanos_since_epoch(){
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
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
	sending_init_flag=1;
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
	sending_init_flag=0;
	return 0;
}



// generic packet sending function
int rc_mav_send_msg(mavlink_message_t msg){
	if(sending_init_flag==0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, sender not initialized\n");
		return -1;
	}
	uint8_t buf[BUFFER_LENGTH];
	int msg_len = mavlink_msg_to_send_buffer(buf, &msg);
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

int rc_mav_send_heartbeat(uint32_t custom_mode,uint8_t type,uint8_t autopilot,
				uint8_t base_mode, uint8_t system_status){
	// sanity check
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(current_system_id, MAV_COMP_ID_ALL, &msg, type, autopilot, base_mode, custom_mode, system_status);
	if(rc_mav_send_msg(msg)){
		fprintf(stderr, "ERROR: in rc_mav_send_heartbeat, failed to send\n");
		return -1;
	}
	return 0;
}

int rc_mav_send_heartbeat_abbreviated(){
	// sanity check
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(current_system_id, MAV_COMP_ID_ALL, &msg, 0, 0, 0, 0, 0);
	if(rc_mav_send_msg(msg)){
		fprintf(stderr, "ERROR: in rc_mav_send_heartbeat_abbreviated, failed to send\n");
		return -1;
	}
	return 0;
}





/// listening stuff
int rc_mav_init_listener(uint16_t port){
	// sanity checks
	if(listening_init_flag!=0){
		fprintf(stderr, "WARNING, trying to init mavlink listener when it's already initialized\n");
		return -1;
	}
	// open socket for UDP packets
	if((listen_sock_fd=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))<0){
		perror("ERROR: in rc_mav_init_listener:");
		return -1;
	}
	// fill out rest of sockaddr_in struct
	listen_sock_addr.sin_family = AF_INET;
	listen_sock_addr.sin_port = htons(port);

	// set timeout
	struct timeval tv;
	tv.tv_sec = LISTEN_SOCKET_TIMEOUT;
	tv.tv_usec = 0;
	if(setsockopt(listen_sock_fd,SOL_SOCKET,SO_RCVTIMEO,&tv,sizeof(struct timeval))){
		perror("ERROR: in rc_mav_init_listener. failed to set socket timeout: ");
		return -1;
	}

	// bind port
	if(bind(listen_sock_fd, &listen_sock_addr, sizeof(struct sockaddr_in))){
		perror("ERROR: in rc_mav_init_listener");
		return -1;
	}
	/*
	// probably not necessary
	// tell system to listen on the port we just created
	if(listen(listen_sock_fd,MAX_PENDING_PACKETS)){
		perror("ERROR: in rc_mav_init_listener, failed to listen() sock_fd: ");
		return -1;
	}
	*/

	// reset all callback functions
	callback_all=rc_null_func;
	for(i=0;i<MAX_UNIQUE_MSG_TYPES;i++) callback[i]=rc_null_func;
	// finally make sure
	listening_shutdown_flag=0;
	pthread_create(&listen_thread,NULL,listening_func,NULL);
	return 0;
}

int rc_mav_cleanup_listener(){
	if(listening_init_flag==0){
		fprintf(stderr, "WARNING, trying to cleanup mavlink listener when it's not running\n");
		return -1;
	}
	// set flag so thread shuts itself down
	listening_shutdown_flag=1;
	// wait for thread to join
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 2;
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(listen_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: in rc_mav_cleanup_listener, exit timeout\n");
		return;
	}
	listening_flag=0
	return 0;
}

void* listening_func(__unused void* ptr){
	ssize_t recsize;
	uint8_t buf[BUFFER_LENGTH];
	socklen_t fromlen;
	mavlink_message_t msg;
	mavlink_status_t parse_status;

	// tell the rest of the program the thread has started
	listening_flag=1;
	// wait until thread is told to shutdown
	while(listening_shutdown_flag==0){
		memset(buf, 0, BUFFER_LENGTH);
		recsize = recvfrom(listen_sock_fd, (void *)buf, BUFFER_LENGTH, 0, listen_sock_addr, &fromlen);
		if(recsize<=0) continue; // some problem, ignore it

		// Something received - print out all bytes and parse packet
		#ifdef DEBUG
		printf("Bytes Received: %d\n", (int)recsize);
		int i;
		for(i=0;i<recsize;i++) printf("%d ",buf[i]);
		printf("\n");
		#endif

		// do mavlink's silly byte-wise parsing method
		for (i = 0; i<recsize; ++i){
			// parse on channel 0(MAVLINK_COMM_0)
			if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &parse_status)){
				#ifdef DEBUG
				printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
				#endif
				ns_of_last_msg[msg.msgid]=rc_nanos_since_epoch();

				received_flag[msg.msgid]=1;
				messages[msg.msgid]=msg;
				// run the generic callback
				callback_all();


				void callbacks(void*)[MAX_UNIQUE_MSG_TYPES];

			}
		}
	}

	listening_flag=0;
}