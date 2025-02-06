#ifndef CONFIG_H
#define CONFIG_H

#include <stdio.h>

#ifdef DEBUG_MODE
    #define LOG(msg) printf("DEBUG: %s\n", msg)
#else
    #define LOG(msg)
#endif

#define BUFFER_SIZE MAX_BUFFER_SIZE

void print_version();

#endif  // CONFIG_H
