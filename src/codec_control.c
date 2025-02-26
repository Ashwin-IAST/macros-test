/*
 *  Codec and Amplifier Control Utility
 *
 *  File: codec_control.c
 *  Author: E.Burley
 *  Date: 08/19/2024
 *
 *  Description:
 *  This utility controls the reset state of the audio codec module and TAS5441 amplifiers
 *  on an embedded Linux system. It uses GPIO lines to assert or deassert control pins,
 *  and I2C commands to configure the devices.
 *
 *  Usage:
 *      codec_control -on                          // Turn on the codec
 *      codec_control -off                         // Turn off the codec
 *      codec_control -mute                        // Mute the codec
 *      codec_control -unmute                      // Unmute the codec
 *      codec_control -volume <gain_value>         // Set playback gain level (-63.5 to +24 dB in 0.5 dB steps)
 *      codec_control -micgain <gain_value>        // Set microphone gain level (0 to +59.5 dB in 0.5 dB steps)
 *      codec_control -micbias <bias_value>        // Set microphone bias level (0, 2V, 2V5, AVDD)
 *      codec_control -amp1 <command> [value]      // Control Amplifier 1
 *      codec_control -amp2 <command> [value]      // Control Amplifier 2
 *
 *  Amplifier Commands:
 *      on                                      // Turn on the amplifier
 *      off                                     // Turn off the amplifier
 *      mute                                    // Mute the amplifier
 *      unmute                                  // Unmute the amplifier
 *      gain <20|26|32|36>                      // Set amplifier gain
 *      status                                  // Read amplifier status
 *
 *  Dependencies:
 *      - libgpiod: GPIO library for accessing GPIO lines
 *      - get_gpiochip_by_i2c_address.h: Custom header for locating the GPIO chip
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <gpiod.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include "get_gpiochip_by_i2c_address.h"

#define CONSUMER "codec_module_control"

#define HUB_A_I2C_CS_LINE_NUM 14
#define EN_LINE_NUM           15    // line number for ~CODEC_RESET

#define R_MUTE_LINE_NUM       24
#define R_STANDBY_LINE_NUM    25
#define L_MUTE_LINE_NUM       27
#define L_STANDBY_LINE_NUM    28

#define I2C_BUS               "i2c-0"
#define I2C_ADDRESS_D         "0075"
#define I2C_ADDRESS_H         "0022"

#define MAX_MIC_GAIN_DB       59.5  // Maximum microphone gain in dB
#define MIN_MIC_GAIN_DB       0.0   // Minimum microphone gain in dB

#define MAX_PLAYBACK_GAIN_DB  24.0  // Maximum playback gain in dB
#define MIN_PLAYBACK_GAIN_DB -63.5  // Minimum playback gain in dB

#define TAS5441_I2C_ADDRESS   0x6C  // I2C address for TAS5441 amplifiers

// Define MICBIAS values
#define MICBIAS_POWERED_DOWN 0x00
#define MICBIAS_2V           0x40
#define MICBIAS_2_5V         0x80
#define MICBIAS_AVDD         0xC0

// Amplifier commands enumeration
enum {
    AMP_CMD_NONE,
    AMP_CMD_ON,
    AMP_CMD_OFF,
    AMP_CMD_MUTE,
    AMP_CMD_UNMUTE,
    AMP_CMD_GAIN,
    AMP_CMD_STATUS,
    AMP_CMD_CONTROL
};

// Function to print usage instructions
void print_usage(const char *prog_name) {
    printf("Usage:\n");
    printf("  %s -on                          // Turn on the codec and initialize settings\n", prog_name);
    printf("  %s -off                         // Turn off the codec\n", prog_name);
    printf("  %s -mute                        // Mute the codec output\n", prog_name);
    printf("  %s -unmute                      // Unmute the codec output\n", prog_name);
    printf("  %s -volume <gain_value>         // Set playback gain level (%.1f to %.1f dB in 0.5 dB steps)\n",
           prog_name, MIN_PLAYBACK_GAIN_DB, MAX_PLAYBACK_GAIN_DB);
    printf("  %s -micgain <gain_value>        // Set microphone gain level (%.1f to %.1f dB in 0.5 dB steps)\n",
           prog_name, MIN_MIC_GAIN_DB, MAX_MIC_GAIN_DB);
    printf("  %s -micbias <bias_value>        // Set microphone bias level (0V, 2V, 2.5V, AVDD)\n", prog_name);
    printf("  %s -amp1 <command> [value]      // Control Amplifier 1\n", prog_name);
    printf("  %s -amp2 <command> [value]      // Control Amplifier 2\n", prog_name);
    printf("  %s -h | --help                  // Display this help message\n", prog_name);

    printf("\nAmplifier Commands:\n");
    printf("  on                              // Turn on the amplifier\n");
    printf("  off                             // Turn off the amplifier\n");
    printf("  mute                            // Mute the amplifier\n");
    printf("  unmute                          // Unmute the amplifier\n");
    printf("  gain <20|26|32|36>              // Set amplifier gain in dB\n");
    printf("  status                          // Read amplifier status\n");

    printf("\nExamples:\n");
    printf("  %s -on\n", prog_name);
    printf("  %s -volume 10.5\n", prog_name);
    printf("  %s -micgain 30.0\n", prog_name);
    printf("  %s -amp1 gain 26\n", prog_name);
    printf("  %s -amp2 status\n", prog_name);
    printf("\n");
}

// Function to check if a file exists
static int file_exists(const char *filename) {
    struct stat buffer;
    return (stat(filename, &buffer) == 0);
}

// Function to check if GPIO chips are ready
static int gpiochip_exists(const char *chip_name) {
    struct gpiod_chip *chip = gpiod_chip_open_by_name(chip_name);
    if (chip) {
        gpiod_chip_close(chip);
        return 1;
    }
    return 0;
}

// Function to send I2C command to TAS5441 amplifier
static int tas5441_send_i2c_command(int amplifier, uint8_t reg, uint8_t *data, int length, int read_flag) {
    int file;
    const char *filename = "/dev/i2c-5";
    uint8_t addr = TAS5441_I2C_ADDRESS;
    int ret;

    // Control HUB_A_I2C_CS to select the amplifier
    struct gpiod_chip *chip_cs;
    struct gpiod_line *cs_line;
    char *gpiochip_cs = find_gpiochip_by_i2c_bus_and_address(I2C_BUS, I2C_ADDRESS_D); // Adjust if necessary
    if (!gpiochip_cs) {
        fprintf(stderr, "GPIO chip for HUB_A_I2C_CS not found.\n");
        return -1;
    }

    chip_cs = gpiod_chip_open_by_name(gpiochip_cs);
    if (!chip_cs) {
        perror("gpiod_chip_open failed for HUB_A_I2C_CS");
        free(gpiochip_cs);
        return -1;
    }

    cs_line = gpiod_chip_get_line(chip_cs, HUB_A_I2C_CS_LINE_NUM);
    if (!cs_line) {
        perror("gpiod_chip_get_line failed for HUB_A_I2C_CS");
        gpiod_chip_close(chip_cs);
        free(gpiochip_cs);
        return -1;
    }

    gpiod_line_request_output(cs_line, CONSUMER, 0);
    // Set HUB_A_I2C_CS to select the amplifier
    gpiod_line_set_value(cs_line, amplifier == 1 ? 0 : 1); // 0 for Amp1, 1 for Amp2

    // Open the I2C device
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        gpiod_line_release(cs_line);
        gpiod_chip_close(chip_cs);
        free(gpiochip_cs);
        return -1;
    }

    // Specify the address of the I2C device
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(file);
        gpiod_line_release(cs_line);
        gpiod_chip_close(chip_cs);
        free(gpiochip_cs);
        return -1;
    }

    if (read_flag) {
        struct i2c_rdwr_ioctl_data packets;
        struct i2c_msg messages[2];

        // Message to write the register address
        messages[0].addr  = addr;
        messages[0].flags = 0; // Write
        messages[0].len   = 1;
        messages[0].buf   = &reg;

        // Message to read the data
        messages[1].addr  = addr;
        messages[1].flags = I2C_M_RD; // Read
        messages[1].len   = length;
        messages[1].buf   = data;

        // Combine the messages into a single transaction
        packets.msgs  = messages;
        packets.nmsgs = 2;

        if (ioctl(file, I2C_RDWR, &packets) < 0) {
            perror("Failed to perform combined write/read transaction");
            // Error handling
        }
    } else {
        // Write register address and data
        uint8_t buffer[1 + length];
        buffer[0] = reg;
        memcpy(&buffer[1], data, length);
        if (write(file, buffer, length + 1) != length + 1) {
            perror("Failed to write data to i2c bus");
            close(file);
            gpiod_line_release(cs_line);
            gpiod_chip_close(chip_cs);
            free(gpiochip_cs);
            return -1;
        }
    }

    // Close the I2C device and release GPIO
    close(file);
    gpiod_line_release(cs_line);
    gpiod_chip_close(chip_cs);
    free(gpiochip_cs);

    return 0;
}

// Function to control TAS5441 amplifier
static int control_tas5441(int amplifier, int command, int value) {
    uint8_t data;
    int ret;

    switch (command) {
        case AMP_CMD_ON:
            // Implement amplifier turn-on sequence if needed
            printf("Turning on Amplifier %d\n", amplifier);
            // Additional code to turn on the amplifier can be added here
            break;
        case AMP_CMD_OFF:
            // Implement amplifier turn-off sequence if needed
            printf("Turning off Amplifier %d\n", amplifier);
            // Additional code to turn off the amplifier can be added here
            break;
        case AMP_CMD_MUTE:
            printf("Muting Amplifier %d\n", amplifier);
            // Set Mute bit in Control Register (Assuming D5 is Mute)
            data = 0x78 | 0x20; // Set D5 to 1
            ret = tas5441_send_i2c_command(amplifier, 0x03, &data, 1, 0);
            if (ret != 0) return ret;
            break;
        case AMP_CMD_UNMUTE:
            printf("Unmuting Amplifier %d\n", amplifier);
            // Clear Mute bit in Control Register
            data = 0x78 & ~0x20; // Clear D5
            ret = tas5441_send_i2c_command(amplifier, 0x03, &data, 1, 0);
            if (ret != 0) return ret;
            break;
        case AMP_CMD_GAIN:
            printf("Setting gain of Amplifier %d to %d dB\n", amplifier, value);
            // Set Gain bits in Control Register
            data = 0x78; // Default Control Register value (switching frequency 400kHz, SpeakerGuard disabled)
            if (value == 20) {
                data &= ~(0xC0); // Clear D7 and D6 for 20 dB
            } else if (value == 26) {
                data = (data & ~0xC0) | 0x40; // Set D6=1, D7=0 for 26 dB
            } else if (value == 32) {
                data = (data & ~0xC0) | 0x80; // Set D7=1, D6=0 for 32 dB
            } else if (value == 36) {
                data |= 0xC0; // Set D7=1, D6=1 for 36 dB
            } else {
                fprintf(stderr, "Invalid gain value for amplifier. Valid values: 20, 26, 32, 36 dB.\n");
                return -1;
            }
            ret = tas5441_send_i2c_command(amplifier, 0x03, &data, 1, 0);
            if (ret != 0) return ret;
            break;
        case AMP_CMD_STATUS:
            printf("Reading status of Amplifier %d\n", amplifier);
            // Read Status Register
            ret = tas5441_send_i2c_command(amplifier, 0x01, &data, 1, 1);
            if (ret != 0) return ret;
            printf("Status Register 1: 0x%02X\n", data);
            if(data != 0)
            {
                if(data & 0x04)
                {
                    printf(" - A load-diagnostics fault has occurred.\n");
                }
                if (data & 0x08)
                {
                    printf(" - Overcurrent shutdown has occurred.\n");
                }
                if (data & 0x10)
                {
                    printf(" - PVDD undervoltage has occurred.\n");
                }
                if (data & 0x20)
                {
                    printf(" - PVDD overvoltage has occurred.\n");
                }
                if (data & 0x40)
                {
                    printf(" - DC offset protection has occurred.\n");
                }
                if (data & 0x80)
                {
                    printf(" - Overtemperature shutdown has occurred.\n");
                }
            }
            else
            {
                printf(" - No errors.\n");
            }

            ret = tas5441_send_i2c_command(amplifier, 0x02, &data, 1, 1);
            if (ret != 0) return ret;
            printf("Status Register 2: 0x%02X\n", data);
            if (data != 0)
            {
                if(data & 0x01)
                {
                    printf(" - Output short to PVDD is present.\n");
                }
                if(data & 0x02)
                {
                    printf(" - Output short to ground is present.\n");
                }
                if(data & 0x04)
                {
                    printf(" - Open load is present.\n");
                }
                if(data & 0x08)
                {
                    printf(" - Shorted load is present.\n");
                }
                if(data & 0x10)
                {
                    printf(" - In a fault condition\n");
                }
                if(data & 0x20)
                {
                    printf(" - Performing load diagnostics\n");
                }
                if(data & 0x40)
                {
                    printf(" - In mute mode\n");
                }
                if(data & 0x80)
                {
                    printf(" - In play mode\n");
                }
            }
            else
            {
                printf(" - No errors.\n");
            }
            break;
        case AMP_CMD_CONTROL:
            ret = tas5441_send_i2c_command(amplifier, 0x03, &data, 1, 1);
            if (ret != 0) return ret;
            if((data & 0xC0) == 0)
            {
                printf(" - Gain set to 20 dB\n");
            }
            else if((data & 0xC0) == 1)
            {
                printf(" - Gain set to 26 dB\n");
            }
            else if((data & 0xC0) == 2)
            {
                printf(" - Gain set to 32 dB\n");
            }
            else if((data & 0xC0) == 3)
            {
                printf(" - Gain set to 36 dB\n");
            }
            if((data & 0x38) == 0)
            {
                printf(" - SpeakerGuard protection circuitry set to 5-V peak.\n");
            }
            else if((data & 0x38) == 1)
            {
                printf(" - SpeakerGuard protection circuitry set to 5.9-V peak.\n");
            }
            else if((data & 0x38) == 2)
            {
                printf(" - SpeakerGuard protection circuitry set 7-V peak.\n");
            }
            else if((data & 0x38) == 3)
            {
                printf(" - SpeakerGuard protection circuitry set to 8.4-V peak.\n");
            }
            else if((data & 0x38) == 4)
            {
                printf(" - SpeakerGuard protection circuitry set to 9.8-V peak.\n");
            }
            else if((data & 0x38) == 5)
            {
                printf(" - SpeakerGuard protection circuitry set to 11.8-V peak.\n");
            }
            else if((data & 0x38) == 6)
            {
                printf(" - SpeakerGuard protection circuitry set to 14-V peak.\n");
            }
            else if((data & 0x38) == 7)
            {
                printf(" - SpeakerGuard protection circuitry set to disabled.\n");
            }
            if ((data & 0x01) == 1)
            {
                printf(" - Switching frequency set to 500 khz.\n");
            }
            else
            {
                printf(" - Switching frequency set to 400 khz.\n");
            }
            break;
        default:
            fprintf(stderr, "Unknown amplifier command.\n");
            return -1;
    }
    return 0;
}

static int send_i2c_command(int file, int reg, int value) {
    unsigned char buffer[2];
    buffer[0] = reg;
    buffer[1] = value;

    if (write(file, buffer, 2) != 2) {
        perror("Failed to write to the i2c bus");
        return -1;
    }
    return 0;
}

static int setup_codec(void) {
    int file;
    const char *filename = "/dev/i2c-5";
    int addr = 0x18;

    // Open the I2C device
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        return -1;
    }

    // Specify the address of the I2C device
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(file);
        return -1;
    }

    // Send the I2C commands (replicating the bash script)
    // MCLK = 12.288 MHz
    send_i2c_command(file, 0x01, 0x08);
    // Route Line1LP to Left ADC, Power up Left ADC
    send_i2c_command(file, 0x13, 0x04);
    // Route Line1RP to Right ADC, Power up Right ADC
    send_i2c_command(file, 0x16, 0x04);
    // Unmute Left PGA, set mic gain to default 20 dB
    send_i2c_command(file, 0x0F, 0x28); // L Mic Gain 20 dB
    // Unmute Right PGA, set mic gain to default 20 dB
    send_i2c_command(file, 0x10, 0x28); // R Mic Gain 20 dB
    // Route Left Data to Left ADC, Route Right Data to Right ADC
    send_i2c_command(file, 0x07, 0x0A);
    // Power up Left and Right DACs
    send_i2c_command(file, 0x25, 0xC0);
    // Unmute Left Digital Volume Control, set playback gain to 0 dB
    send_i2c_command(file, 0x2B, 0x00);
    // Unmute Right Digital Volume Control, set playback gain to 0 dB
    send_i2c_command(file, 0x2C, 0x00);
    // Route Left DAC output to Left Line Outs
    send_i2c_command(file, 0x52, 0x80);
    // Route Right DAC output to Right Line Outs
    send_i2c_command(file, 0x5C, 0x80);
    // Power up Left line out +/- (differential), set gain to 0 dB
    send_i2c_command(file, 0x56, 0x09);
    // Power up Right line out +/- (differential), set gain to 0 dB
    send_i2c_command(file, 0x5D, 0x09);
    // MICBIAS - 0xC0 = AVDD
    send_i2c_command(file, 0x19, MICBIAS_AVDD);

    // Close the I2C device
    close(file);

    return 0;
}

// Function to set playback gain (volume)
static int set_playback_gain(double gain_dB) {
    if (gain_dB < MIN_PLAYBACK_GAIN_DB || gain_dB > MAX_PLAYBACK_GAIN_DB) {
        fprintf(stderr, "Playback gain must be between %.1f and %.1f dB.\n", MIN_PLAYBACK_GAIN_DB, MAX_PLAYBACK_GAIN_DB);
        return -1;
    }
    int register_value = (int)(gain_dB * -2); // Convert dB to register value (negative slope)

    int file;
    const char *filename = "/dev/i2c-5";
    int addr = 0x18;

    // Open the I2C device
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        return -1;
    }

    // Specify the address of the I2C device
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(file);
        return -1;
    }

    // Unmute Left Digital Volume Control, set playback gain
    send_i2c_command(file, 0x2B, register_value & 0xFF); // Left Playback Volume
    // Unmute Right Digital Volume Control, set playback gain
    send_i2c_command(file, 0x2C, register_value & 0xFF); // Right Playback Volume

    // Close the I2C device
    close(file);

    return 0;
}

// Function to set microphone gain
static int set_mic_gain(double gain_dB) {
    if (gain_dB < MIN_MIC_GAIN_DB || gain_dB > MAX_MIC_GAIN_DB) {
        fprintf(stderr, "Microphone gain must be between %.1f and %.1f dB.\n", MIN_MIC_GAIN_DB, MAX_MIC_GAIN_DB);
        return -1;
    }
    int register_value = (int)(gain_dB * 2 + 0.5); // Convert dB to register value

    int file;
    const char *filename = "/dev/i2c-5";
    int addr = 0x18;

    // Open the I2C device
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        return -1;
    }

    // Specify the address of the I2C device
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(file);
        return -1;
    }

    // Unmute Left PGA, set mic gain
    send_i2c_command(file, 0x0F, register_value & 0x7F); // L Mic Gain (bits D6-D0)
    // Unmute Right PGA, set mic gain
    send_i2c_command(file, 0x10, register_value & 0x7F); // R Mic Gain (bits D6-D0)

    // Close the I2C device
    close(file);

    return 0;
}

static int set_micbias(int micbias_value) {
    int file;
    const char *filename = "/dev/i2c-5";
    int addr = 0x18; // Codec I2C address

    // Open the I2C device
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("Failed to open the i2c bus");
        return -1;
    }

    // Specify the address of the I2C device
    if (ioctl(file, I2C_SLAVE, addr) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(file);
        return -1;
    }

    // Write the MICBIAS value to register 0x19
    if (send_i2c_command(file, 0x19, micbias_value) != 0) {
        fprintf(stderr, "Failed to write MICBIAS value to the codec.\n");
        close(file);
        return -1;
    }

    // Close the I2C device
    close(file);

    printf("MICBIAS set to 0x%02X.\n", micbias_value);
    return 0;
}

static int wait_until_ready(void)
{
    int max_retries = 60; // Maximum number of retries
    int retry_count = 0;

    // Wait for /dev/i2c-5 to be available
    while (!file_exists("/dev/i2c-5")) {
        //if (retry_count >= max_retries) {
            //fprintf(stderr, "Error: /dev/i2c-5 not available after %d seconds.\n", max_retries);
            //return 1;
        //}
        printf("Waiting for /dev/i2c-5 to be ready...(demo dec 12)\n");
        sleep(1);
        //retry_count++;
    }

    // Reset retry count for GPIO chips
    retry_count = 0;

    // Wait for GPIO chips to be ready
    while (!gpiochip_exists("gpiochip2") || !gpiochip_exists("gpiochip3")) {
        //if (retry_count >= max_retries) {
            //fprintf(stderr, "Error: GPIO chips not available after %d seconds.\n", max_retries);
            //return 1;
       //}
        printf("Waiting for GPIO chips to be ready...\n");
        sleep(1);
        //retry_count++;
    }

    return 0;
}

int main(int argc, char **argv) {
    struct gpiod_chip *chip_d;
    struct gpiod_chip *chip_h;
    struct gpiod_line *en_line;
    struct gpiod_line *r_mute_line;
    struct gpiod_line *r_standby_line;
    struct gpiod_line *l_mute_line;
    struct gpiod_line *l_standby_line;

    int ret;
    int command = 0; // 1: on, 2: off, 3: mute, 4: unmute
    double gain_dB = 0.0;

    // Variables for amplifier control
    int amplifier = 0; // 1 or 2
    int amp_command = AMP_CMD_NONE;
    int amp_value = 0;
    if(wait_until_ready() == 1)
    {
        exit(EXIT_FAILURE);
    }

    if (argc < 2) {
        fprintf(stderr, "Error: Missing arguments.\n");
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    // Check for help option
    if (strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "--help") == 0) {
        print_usage(argv[0]);
        return EXIT_SUCCESS;
    }
    else
    {
        // Command-line parsing
        if (strcmp(argv[1], "-amp1") == 0 || strcmp(argv[1], "-amp2") == 0) {
            amplifier = (strcmp(argv[1], "-amp1") == 0) ? 1 : 2;
            if (argc < 3) {
                fprintf(stderr, "Usage: %s -amp%d <command> [value]\n", argv[0], amplifier);
                return EXIT_FAILURE;
            }
            if (strcmp(argv[2], "on") == 0) {
                amp_command = AMP_CMD_ON;
            } else if (strcmp(argv[2], "off") == 0) {
                amp_command = AMP_CMD_OFF;
            } else if (strcmp(argv[2], "mute") == 0) {
                amp_command = AMP_CMD_MUTE;
            } else if (strcmp(argv[2], "unmute") == 0) {
                amp_command = AMP_CMD_UNMUTE;
            } else if (strcmp(argv[2], "gain") == 0) {
                if (argc < 4) {
                    fprintf(stderr, "Usage: %s -amp%d gain <20|26|32|36>\n", argv[0], amplifier);
                    return EXIT_FAILURE;
                }
                amp_command = AMP_CMD_GAIN;
                amp_value = atoi(argv[3]);
                if (amp_value != 20 && amp_value != 26 && amp_value != 32 && amp_value != 36) {
                    fprintf(stderr, "Invalid gain value for amplifier. Valid values: 20, 26, 32, 36 dB.\n");
                    return EXIT_FAILURE;
                }
            } else if (strcmp(argv[2], "status") == 0) {
                amp_command = AMP_CMD_STATUS;
            } else if (strcmp(argv[2], "control") == 0) {
                amp_command = AMP_CMD_CONTROL;
            } else {
                fprintf(stderr, "Invalid amplifier command.\n");
                return EXIT_FAILURE;
            }
            // Execute amplifier command
            if (control_tas5441(amplifier, amp_command, amp_value) != 0) {
                fprintf(stderr, "Failed to control amplifier %d.\n", amplifier);
                return EXIT_FAILURE;
            }
            return EXIT_SUCCESS;
        } else if (strcmp(argv[1], "-volume") == 0) {
            if (argc != 3) {
                fprintf(stderr, "Usage: %s -volume <gain_value>\n", argv[0]);
                return EXIT_FAILURE;
            }
            gain_dB = atof(argv[2]);
            if (set_playback_gain(gain_dB) != 0) {
                fprintf(stderr, "Failed to set playback gain.\n");
                return EXIT_FAILURE;
            }
            return EXIT_SUCCESS;
        } else if (strcmp(argv[1], "-micgain") == 0) {
            if (argc != 3) {
                fprintf(stderr, "Usage: %s -micgain <gain_value>\n", argv[0]);
                return EXIT_FAILURE;
            }
            gain_dB = atof(argv[2]);
            if (set_mic_gain(gain_dB) != 0) {
                fprintf(stderr, "Failed to set microphone gain.\n");
                return EXIT_FAILURE;
            }
            return EXIT_SUCCESS;
        } else if (strcmp(argv[1], "-micbias") == 0) {
            if (argc != 3) {
                fprintf(stderr, "Usage: %s -micbias <value>\n", argv[0]);
                return EXIT_FAILURE;
            }
            int micbias_value = MICBIAS_AVDD;
            // Parse the micbias value
            if (strcmp(argv[2], "0V") == 0 || strcmp(argv[2], "0x00") == 0) {
                micbias_value = MICBIAS_POWERED_DOWN;
            } else if (strcmp(argv[2], "2V") == 0 || strcmp(argv[2], "0x40") == 0) {
                micbias_value = MICBIAS_2V;
            } else if (strcmp(argv[2], "2.5V") == 0 || strcmp(argv[2], "0x80") == 0) {
                micbias_value = MICBIAS_2_5V;
            } else if (strcmp(argv[2], "AVDD") == 0 || strcmp(argv[2], "0xC0") == 0) {
                micbias_value = MICBIAS_AVDD;
            } else {
                fprintf(stderr, "Invalid MICBIAS value. Valid values are '0V', '2V', '2.5V', 'AVDD', or hex values 0x00, 0x40, 0x80, 0xC0.\n");
                return EXIT_FAILURE;
            }
            if (set_micbias(micbias_value) != 0) {
                fprintf(stderr, "Failed to set MICBIAS.\n");
                return EXIT_FAILURE;
            }
            return EXIT_SUCCESS;
        } else if (strcmp(argv[1], "-on") == 0) {
            command = 1;
        } else if (strcmp(argv[1], "-off") == 0) {
            command = 2;
        } else if (strcmp(argv[1], "-mute") == 0) {
            command = 3;
        } else if (strcmp(argv[1], "-unmute") == 0) {
            command = 4;
        } else {
            fprintf(stderr, "Invalid argument. Use -on, -off, -mute, -unmute, -volume <gain_value>, -micgain <gain_value>, or -amp1/amp2 <command> [value].\n");
            return EXIT_FAILURE;
        }

        // For commands that require GPIO manipulation
        char *gpiochip_d = find_gpiochip_by_i2c_bus_and_address(I2C_BUS, I2C_ADDRESS_D);
        if (gpiochip_d) {
            printf("Found GPIO chip D: %s\n", gpiochip_d);
        } else {
            fprintf(stderr, "GPIO chip D not found.\n");
            return EXIT_FAILURE;
        }

        char *gpiochip_h = find_gpiochip_by_i2c_bus_and_address(I2C_BUS, I2C_ADDRESS_H);
        if (gpiochip_h) {
            printf("Found GPIO chip H: %s\n", gpiochip_h);
        } else {
            fprintf(stderr, "GPIO chip H not found.\n");
            free(gpiochip_d);
            return EXIT_FAILURE;
        }

        chip_d = gpiod_chip_open_by_name(gpiochip_d);
        if (!chip_d) {
            perror("gpiod_chip_open failed for chip D");
            free(gpiochip_d);
            free(gpiochip_h);
            return EXIT_FAILURE;
        }

        chip_h = gpiod_chip_open_by_name(gpiochip_h);
        if (!chip_h) {
            perror("gpiod_chip_open failed for chip H");
            gpiod_chip_close(chip_d);
            free(gpiochip_d);
            free(gpiochip_h);
            return EXIT_FAILURE;
        }

        // Free allocated gpiochip names as they are no longer needed
        free(gpiochip_d);
        free(gpiochip_h);

        // Get lines
        en_line = gpiod_chip_get_line(chip_d, EN_LINE_NUM);
        r_mute_line = gpiod_chip_get_line(chip_h, R_MUTE_LINE_NUM);
        r_standby_line = gpiod_chip_get_line(chip_h, R_STANDBY_LINE_NUM);
        l_mute_line = gpiod_chip_get_line(chip_h, L_MUTE_LINE_NUM);
        l_standby_line = gpiod_chip_get_line(chip_h, L_STANDBY_LINE_NUM);

        // Configure lines as outputs with initial values
        gpiod_line_request_output(en_line, CONSUMER, 0);
        gpiod_line_request_output(r_mute_line, CONSUMER, 0);
        gpiod_line_request_output(r_standby_line, CONSUMER, 0);
        gpiod_line_request_output(l_mute_line, CONSUMER, 0);
        gpiod_line_request_output(l_standby_line, CONSUMER, 0);

        if (command == 1) { // -on
            // Sequence to turn on the codec
            gpiod_line_set_value(en_line, 1);
            gpiod_line_set_value(r_mute_line, 0);
            gpiod_line_set_value(l_mute_line, 0);
            gpiod_line_set_value(r_standby_line, 1);
            gpiod_line_set_value(l_standby_line, 1);

            printf("Setting up codec...\n");
            if (setup_codec() != 0) {
                fprintf(stderr, "Failed to set up codec.\n");
                // Cleanup GPIO
                gpiod_line_release(en_line);
                gpiod_line_release(r_mute_line);
                gpiod_line_release(l_mute_line);
                gpiod_line_release(r_standby_line);
                gpiod_line_release(l_standby_line);
                gpiod_chip_close(chip_d);
                gpiod_chip_close(chip_h);
                return EXIT_FAILURE;
            }
        } else if (command == 2) { // -off
            gpiod_line_set_value(en_line, 0);
            gpiod_line_set_value(r_mute_line, 1);
            gpiod_line_set_value(l_mute_line, 1);
            gpiod_line_set_value(r_standby_line, 0);
            gpiod_line_set_value(l_standby_line, 0);
        } else if (command == 3) { // -mute
            gpiod_line_set_value(r_mute_line, 1);
            gpiod_line_set_value(l_mute_line, 1);
        } else if (command == 4) { // -unmute
            gpiod_line_set_value(r_mute_line, 0);
            gpiod_line_set_value(l_mute_line, 0);
        }

        // Cleanup GPIO
        gpiod_line_release(en_line);
        gpiod_line_release(r_mute_line);
        gpiod_line_release(l_mute_line);
        gpiod_line_release(r_standby_line);
        gpiod_line_release(l_standby_line);

        gpiod_chip_close(chip_d);
        gpiod_chip_close(chip_h);
    }
    return EXIT_SUCCESS;
}
