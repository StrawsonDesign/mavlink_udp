// basic heartbeat tester, sends heartbeat packets every second and
// prints to the screen when one has been received
//

#include <stdio.h>
#include <rc/mavlink_udp.h>

#define LOCALHOST_IP	"127.0.0.1"
#define MY_SYS_ID	1


// called by the rc_mav lib whenever a heartbeat is received
void heartbeat_callback_function(){
	int sysid = rc_mav_get_sys_id_of_last_msg(MAVLINK_MSG_ID_HEARTBEAT);
	printf("received heartbeat from sysid:%d \n", sysid);
	return;
}


int main(int argc, char * argv[])
{
	if (rc_mav_init(MY_SYS_ID, LOCALHOST_IP, RC_MAV_DEFAULT_UDP_PORT) < 0){
		return -1;
	}
	// set the heartbeat callback to print something when receiving
	rc_mav_set_callback(MAVLINK_MSG_ID_HEARTBEAT, heartbeat_callback_function);
	printf("Sending heartbeat message every 1 second...\n");
	while(1){
		sleep(1);
		rc_mav_send_heartbeat_abbreviated();
	}
	//rc_mav_cleanup();
}