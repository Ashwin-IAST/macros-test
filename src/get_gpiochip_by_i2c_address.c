/*
 *  GPIO Chip Locator Utility
 *
 *  File: get_gpiochip_by_i2c_address.c
 *  Author: E.Burley
 *  Date: 08/19/2024
 *
 *  Description:
 *  This utility provides a function to locate the name of a GPIO chip that
 *  is associated with a specific I2C bus and device address. It traverses
 *  the sysfs entries in Linux to find the corresponding GPIO chip directory
 *  for the given I2C device.
 *
 *  The function `find_gpiochip_by_i2c_bus_and_address` returns the name of
 *  the GPIO chip as a string, which can be used in subsequent GPIO operations.
 *
 *  Usage:
 *      char* gpiochip = find_gpiochip_by_i2c_bus_and_address("i2c-0", "0023");
 *      if (gpiochip) {
 *          printf("Found GPIO chip: %s\n", gpiochip);
 *          free(gpiochip);  // Remember to free the allocated memory
 *      }
 *
 *  Parameters:
 *      - i2c_bus: The I2C bus identifier (e.g., "i2c-0").
 *      - i2c_address: The I2C device address as a string (e.g., "0023").
 *
 *  Return:
 *      - A dynamically allocated string containing the GPIO chip name if found,
 *        or NULL if no matching GPIO chip is found or if an error occurs.
 *
 *  Dependencies:
 *      - Standard C libraries: stdio.h, dirent.h, string.h, stdlib.h
 *
 *
 */

#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <stdlib.h>

char* find_gpiochip_by_i2c_bus_and_address(const char* i2c_bus, const char* i2c_address) {
    DIR *dir;
    struct dirent *entry;
    char path[1024], *gpiochip = NULL;

    snprintf(path, sizeof(path), "/sys/class/i2c-dev/%s/subsystem/%s/device/%s-%s", i2c_bus, i2c_bus, i2c_bus + 4, i2c_address);    
    // Open the directory for the specific I2C device
    dir = opendir(path);
    if (!dir) {
        perror("Failed to open I2C device directory");
        return NULL;
    }

    // Read entries in the directory
    while ((entry = readdir(dir)) != NULL) {
        // Check if the entry name starts with "gpiochip"
        if (strncmp(entry->d_name, "gpiochip", 8) == 0) {
            gpiochip = strdup(entry->d_name);
            break;
        }
    }

    closedir(dir);
    return gpiochip;
}
