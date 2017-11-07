//
//  common.c
//  robot-udp
//
//  Created by Trevor Irwin on 3/29/17.
//  Copyright © 2017 Trevor Irwin. All rights reserved.
//

// Includes
#include "common.h"

// Global Variables
volatile enum program_state_t program_state;

// Function Definitions
void int_handler(int dummy){
    program_state = EXITING;
}

int die(char* s){
    perror(s);
    program_state = EXITING;
    return -1;
}

void set_state(enum program_state_t new_state){
    program_state = new_state;
}
enum program_state_t get_state(){
    return program_state;
}

void setup_common(){
    signal(SIGINT, int_handler);
}
