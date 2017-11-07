//
//  common.h
//  robot-udp
//
//  Created by Trevor Irwin on 2/10/17.
//  Copyright © 2017 Trevor Irwin. All rights reserved.
//

#ifndef common_h
#define common_h

// Includes
#include <stdio.h>
#include <signal.h>
#include <pthread.h>
#include <errno.h>
#include <sys/time.h>

#include <arpa/inet.h> // sockets
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>


// Types
enum program_state_t { RUNNING, EXITING };

// Function Declarations
void int_handler(int dummy);
int die(char* s);
void set_state(enum program_state_t new_state);
enum program_state_t get_state();
void setup_common();


#endif /* common_h */
