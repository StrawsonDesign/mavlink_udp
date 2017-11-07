//
//  main.c
//  robot-udp
//
//  Created by Trevor Irwin on 2/3/17.
//  Copyright © 2017 Trevor Irwin. All rights reserved.
//

// Includes
#include "../../robotudp.h"

// Macros
#define MAIN_SLEEP_TIME 3000000 // 3 s

// Global Variables
rc_mavlink_socket_t* mav_sock;
pthread_t* mav_server_thread;

// Function Declarations
int setup_listener();

// Function Definitions
int main(int argc, const char * argv[]){

    setup_common();
    
    set_state(RUNNING);
    
    printf("STARTING SERVER.\n");
    usleep(20 * 1000);
    
    printf("Setting up packet listener...\n");
    
    setup_listener();
    
    // Loop until the server is closed
    while (get_state() != EXITING){
        printf("Listening for packets...\n");
        fflush(stdout);
        usleep(MAIN_SLEEP_TIME);
    }
    
    // Clean exit
    printf("Exiting cleanly. \n");
    
    // Cleanup socket and server
    rc_mav_cleanup_listener(mav_server_thread);
    rc_mav_cleanup_socket(mav_sock);
    
    return 0;
}


int print_heartbeat(){
    printf("Heartbeat received!\n");
    return 0;
}

int print_attitude(float roll, float pitch, float yaw, float rollspeed,
            float pitchspeed, float yawspeed){
    printf("Pitch: %f\tYaw: %f\t Roll: %f\n", pitch, yaw, roll);
    return 0;
}

int print_control(int16_t x, int16_t y, int16_t z, int16_t r, uint16_t buttons){
    printf("X Axis: %d\tY Axis: %d\tZ Axis: %d\t"
           "R Axis: %d\t Buttons: %d\n", x, y, z, r, buttons);
    return 0;
}

int setup_listener(){
    mav_sock = rc_mav_init_server(RC_MAV_DEFAULT_PORT);
    mav_server_thread = rc_mav_init_listener(mav_sock);
    rc_mav_set_heartbeat_func(mav_sock, &print_heartbeat);
    rc_mav_set_attitude_func(mav_sock, &print_attitude);
    rc_mav_set_control_func(mav_sock, &print_control);
    printf("Setup listener on port: %d.\n", ntohs(mav_sock->sock_me.sin_port));
    return 0;
}

