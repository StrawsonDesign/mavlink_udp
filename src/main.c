#include "mavlink_udp.h"

int main(int argc, char * argv[])
{
    const char * my_address = "127.0.0.1";
    if (rc_mav_init(1, my_address, 40000) < 0){
        return -1;
    }
    while (1){
        sleep(1);
        rc_mav_send_heartbeat_abbreviated();
    }
    //rc_mav_cleanup();
}