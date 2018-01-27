/*******************************************************************************
* mavlink_udp.c
*******************************************************************************/
#include <rc/mavlink_udp.h>

#define BUFFER_LENGTH		512 // common networking buffer size
#define MAX_UNIQUE_MSG_TYPES	256
#define HEARTBEAT_TIMEOUT	3
#define MAX_PENDING_CONNECTIONS	32
#define LOCALHOST_IP "127.0.0.1"


// connection stuff
static int init_flag=0;
static int sock_fd;
static int current_port;
static struct sockaddr_in my_address ;
static struct sockaddr_in dest_address;
static uint8_t system_id;

// callbacks
static void (*callbacks[MAX_UNIQUE_MSG_TYPES])(void);
static void (*callback_all)(void); // called when any packet arrives
static void (*connection_lost_callback)(void);

// flags and info populated by the listening thread
static int received_flag[MAX_UNIQUE_MSG_TYPES];
static uint64_t ns_of_last_msg[MAX_UNIQUE_MSG_TYPES];
static uint64_t ns_of_last_msg_any;
static mavlink_message_t messages[MAX_UNIQUE_MSG_TYPES];

// thread startup and shutdown flags
static pthread_t listener_thread;
static int shutdown_flag=0;
static int listening_flag=0;
static int listening_init_flag=0;


// private local functions
static void* __rc_mav_listen();
static int __rc_mav_recv_msg();
static void __null_func();
static uint64_t __rc_nanos_since_epoch();
static int __address_init(struct sockaddr_in * address, const char* dest_ip, int port);



/*******************************************************************************
* void __null_func()
*
* A simple function that just returns. This exists so callback pointers can be
* set to do nothing
*******************************************************************************/
static void __null_func(){
	return;
}

/*******************************************************************************
* @ uint64_t __rc_nanos_since_epoch()
*
* Returns the number of nanoseconds since epoch using system CLOCK_REALTIME
* This function itself takes about 1100ns to complete at 1ghz under ideal
* circumstances.
*******************************************************************************/
static uint64_t __rc_nanos_since_epoch(){
	struct timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	return ((uint64_t)ts.tv_sec*1000000000)+ts.tv_nsec;
}

int rc_mav_init(uint8_t sysid, const char* dest_ip, uint16_t port){
	int i;

	// sanity checks
	if(init_flag!=0){
		fprintf(stderr, "WARNING, trying to init mavlink connection when it's already initialized!\n");
		return -1;
	}

	if(dest_ip==NULL){
		fprintf(stderr, "ERROR: in rc_mav_init received NULL dest_ip string\n");
		return -1;
	}

	// save port globally for other functions to use
	current_port = port;

	// set all the callback pointers to something sane
	callback_all = __null_func;
	connection_lost_callback = __null_func;
	for(i=0;i<MAX_UNIQUE_MSG_TYPES;i++) callbacks[i] = __null_func;

	// open socket for UDP packets
	if((sock_fd=socket(AF_INET, SOCK_DGRAM, 0)) < 0){
		perror("ERROR: in rc_mav_init: ");
		return -1;
	}

	// fill out rest of sockaddr_in struct
	if (__address_init(&my_address, 0, current_port) != 0){
		fprintf(stderr, "ERROR: in rc_mav_init: couldn't set local address\n");
		return -1;
	}

	// bind address to listening port
	if(bind(sock_fd, (struct sockaddr *) &my_address, sizeof my_address) < 0){
		perror("ERROR: in rc_mav_init: ");
		return -1;
	}

	// set destination address
	if(__address_init(&dest_address, dest_ip, current_port) != 0){
		fprintf(stderr, "ERROR: in rc_mav_init: couldn't set destination address");
		return -1;
	}

	// signal initialization finished
	init_flag=1;
	system_id=sysid;

	// spawn listener thread
	if (pthread_create(&listener_thread, NULL, __rc_mav_listen, NULL) < 0){
		perror("ERROR: in rc_mav_init: ");
		return -1;
	}

	return 0;
}

// background thread for handling packets
void* __rc_mav_listen(){
	fprintf(stderr, "listening...\n");
	listening_flag=1;
	while (shutdown_flag==0){
		__rc_mav_recv_msg();
	}
	printf("exiting rc_mav_listen\n");
	return 0;
}

// pulls one msg from the network buffer
int __rc_mav_recv_msg(){
	int i;
	ssize_t num_bytes_rcvd;;
	uint8_t buf[BUFFER_LENGTH];
	socklen_t addr_len = sizeof my_address;
	mavlink_message_t msg;
	mavlink_status_t parse_status;

	memset(buf, 0, BUFFER_LENGTH);
	num_bytes_rcvd = recvfrom(sock_fd, buf, BUFFER_LENGTH, 0, (struct sockaddr *) &my_address, &addr_len);

	// do mavlink's silly byte-wise parsing method
	for (i = 0; i<num_bytes_rcvd; ++i){
		// parse on channel 0(MAVLINK_COMM_0)
		if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &parse_status)){
			#ifdef DEBUG
			printf("\nReceived packet: SYSID: %d, MSG ID: %d\n", msg.sysid, msg.msgid);
			#endif
			// update timestamps and received flag
			ns_of_last_msg[msg.msgid]=__rc_nanos_since_epoch();
			ns_of_last_msg_any = ns_of_last_msg[msg.msgid];
			received_flag[msg.msgid]=1;
			// save local copy of message
			messages[msg.msgid]=msg;
			// run the generic callback
			callback_all();
			// run the msg-specific callback
			callbacks[msg.msgid]();

		}
	}

	return 0;
}

// generic packet sending function
int rc_mav_send_msg(mavlink_message_t msg){
	if(init_flag == 0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, socket not initialized\n");
		return -1;
	}
	uint8_t buf[BUFFER_LENGTH];
	memset(buf, 0, BUFFER_LENGTH);
	int msg_len = mavlink_msg_to_send_buffer(buf, &msg);
	if(msg_len < 0){
		fprintf(stderr, "ERROR: in rc_mav_send_msg, unable to pack message for sending\n");
		return -1;
	}
	int bytes_sent = sendto(sock_fd, buf, msg_len, 0, (struct sockaddr *) &dest_address,
							sizeof dest_address);
	if(bytes_sent != msg_len){
		perror("ERROR: in rc_mav_send_msg: ");
		return -1;
	}
	return 0;
}

int rc_mav_send_heartbeat_abbreviated(){
	// sanity check
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_ALL, &msg, 0, 0, 0, 0, 0);
	if(rc_mav_send_msg(msg)){
		fprintf(stderr, "ERROR: in rc_mav_send_heartbeat_abbreviated, failed to send\n");
		return -1;
	}
	return 0;
}


int rc_mav_send_heartbeat(uint32_t custom_mode,uint8_t type,uint8_t autopilot,
							uint8_t base_mode, uint8_t system_status){
	// sanity check
	mavlink_message_t msg;
	mavlink_msg_heartbeat_pack(system_id, MAV_COMP_ID_ALL, &msg, type, autopilot, base_mode, custom_mode, system_status);
	if(rc_mav_send_msg(msg)){
		fprintf(stderr, "ERROR: in rc_mav_send_heartbeat, failed to send\n");
		return -1;
	}
	return 0;
}


int rc_mav_set_dest_ip(const char* dest_ip){

	return __address_init(&dest_address,dest_ip,current_port);

}

int __address_init(struct sockaddr_in * address, const char* dest_ip, int port){
	// sanity check
	if(address == NULL || port < 1){
		fprintf(stderr, "ERROR: in __address_init: received NULL address struct\n");
		return -1;
	}
	memset((char*) address, 0, sizeof address);
	address->sin_family = AF_INET;
	address->sin_port = htons(port);
	address->sin_addr.s_addr = ((long)dest_ip==0) ? htonl(INADDR_ANY) : inet_addr(dest_ip);
	return 0;
}

int rc_mav_set_system_id(uint8_t sys_id){
	system_id = system_id;
	return 0;
}


int rc_mav_cleanup(){
	if(init_flag==0 || listening_flag==0){
		fprintf(stderr, "WARNING, trying to cleanup mavlink listener when it's not running\n");
		return -1;
	}
	shutdown_flag=1;
	listening_flag=0;

	// wait for thread to join
	struct timespec thread_timeout;
	clock_gettime(CLOCK_REALTIME, &thread_timeout);
	thread_timeout.tv_sec += 2;
	int thread_err = 0;
	thread_err = pthread_timedjoin_np(listener_thread, NULL, &thread_timeout);
	if(thread_err == ETIMEDOUT){
		printf("WARNING: in rc_mav_cleanup_listener, exit timeout\n");
		return -1;
	}
	close(sock_fd);
	init_flag=0;
	return 0;
}


/*
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
				ns_of_last_msg[msg.msgid]=__rc_nanos_since_epoch();

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
*/



// callback setters
int rc_mav_set_callback(int msg_id, void (*func)(void)){
	if(msg_id<0 || msg_id>MAX_UNIQUE_MSG_TYPES){
		fprintf(stderr,"ERROR: in rc_mav_set_callback, msg_id out of bounds\n");
		return -1;
	}
	if(func==NULL){
		fprintf(stderr,"ERROR: in rc_mav_set_callback, received NULL pointer\n");
		return -1;
	}
	callbacks[msg_id]=func;
	return 0;
}


int rc_mav_set_callback_all(void (*func)(void)){
	if(func==NULL){
		fprintf(stderr,"ERROR: in rc_mav_set_callback_all, received NULL pointer\n");
		return -1;
	}
	callback_all=func;
	return 0;
}

int rc_mav_set_connection_lost_callback(void (*func)(void)){
	if(func==NULL){
		fprintf(stderr,"ERROR: in rc_mav_set_connection_lost_callback, received NULL pointer\n");
		return -1;
	}
	connection_lost_callback=func;
	return 0;
}

int rc_mav_get_sys_id_of_last_msg(int msg_id){
	if(msg_id<0 || msg_id>MAX_UNIQUE_MSG_TYPES){
		fprintf(stderr,"ERROR: in rc_mav_get_sys_id_of_last_msg, msg_id out of bounds\n");
		return -1;
	}
	return messages[msg_id].sysid;
}