//
//  main.c
//  robot-udp
//
//  Created by Trevor Irwin on 2/5/17.
//  Copyright © 2017 Trevor Irwin. All rights reserved.
//

// Includes
#include "../../robotudp.h"

// Macros
#define SERVER_IP "127.0.0.1"
#define INPUT_WAIT_TIME 1000000 // 1000 ms

// Global Variables
char in_buf[SOCK_BUF_LEN];
rc_mavlink_socket_t* mav_sock;

// Function Declarations
int setup_client();

int main(int argc, const char * argv[]){
    
    setup_common();
    
    set_state(RUNNING);
    
    setup_client();
    
    printf("Client Started\nPlease type a message.\n");
    
    // Make the stdin socket non-blocking
    if (fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK) != 0){
        return die("fcntl (setting non-blocking flag)");
    }
    
    while(get_state() != EXITING){
        usleep(INPUT_WAIT_TIME);
        
//        if (get_user_input() != 0){
//            set_state(EXITING);
//        }
        rc_mav_send_heartbeat(mav_sock);
        rc_mav_send_attitude(mav_sock, 0.3f, 0.1f, 0.2f, 1.1f, 1.2f, 1.3f);
        rc_mav_send_control(mav_sock, 1, 2, 3, 4, 8);
    }
    
    // Clean exit
    printf("Exiting cleanly. \n");
    
    // Cleanup socket
    rc_mav_cleanup_socket(mav_sock);
    
    return 0;
}


int setup_client(){
    mav_sock = rc_mav_init_client(RC_MAV_DEFAULT_PORT, SERVER_IP);
    //printf("Port: %d\n", ntohs(mav_sock->sock_me.sin_port));
    return 0;
}




//int get_user_input(){
//    // Get user input and send over udp
//    while (read(STDIN_FILENO, in_buf, SOCK_BUF_LEN) > 0)
//    {
//        // Send a packet
//        printf("Sending packet %s\n", in_buf);
//        if (sendto(sock_fd, in_buf, SOCK_BUF_LEN, 0, &sock_other, sock_len) < 0){
//            die("sendto");
//        }
//        // Clear the buffer
//        memset(in_buf, 0, SOCK_BUF_LEN);
//    }
//    return 0;
//}
