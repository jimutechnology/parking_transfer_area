/*
 * socket_can.h
 *
 *  Created on: 2020年3月17日
 *      Author: wxp
 */

#ifndef __GPIO_H_
#define __GPIO_H_

#include <stdint.h>
#include "string.h"
#include <iostream>
#include <unistd.h>
#include "ros/ros.h"
#include <poll.h>
#include "stdlib.h"
#include "stdio.h"
#include "unistd.h"
#include "fcntl.h" //define O_WRONLY and O_RDONLY


#define SYSFS_GPIO_EXPORT "/sys/class/gpio/export"
#define SYSFS_GPIO_RST_DIR_VAL "OUT"
#define SYSFS_GPIO_RST_VAL_H "1"
#define SYSFS_GPIO_RST_VAL_L "0"

using namespace std;

class Gpio
{
public:
    char export_pin_value[5];

public:
    Gpio(char pin_value[])
    {
        memcpy(this->export_pin_value,pin_value,sizeof(this->export_pin_value));
    }

    ~Gpio() 
    {

    }

    int gpio_export(int pin)
    {
        char buffer[64];
        int len;
        int fd;

        fd = open("/sys/class/gpio/export", O_WRONLY);
        if (fd < 0) 
        {
            printf("Failed to open export for writing!\n");
            return(-1);
        }
        len = snprintf(buffer, sizeof(buffer), "%d", pin);
        if (write(fd, buffer, len) < 0) 
        {
            printf("Failed to export gpio!");
            return -1;
        }
        close(fd);
        return 0;
    }

    int gpio_unexport(int pin)
    {
        char buffer[64];
        int len;
        int fd;

        fd = open("/sys/class/gpio/unexport", O_WRONLY);
        if (fd < 0) 
        {
            printf("Failed to open unexport for writing!\n");
            return -1;
        }

        len = snprintf(buffer, sizeof(buffer), "%d", pin);
        if (write(fd, buffer, len) < 0) 
        {
            printf("Failed to unexport gpio!");
            return -1;
        }
        close(fd);
        return 0;
    }
    int gpio_direction(int pin, int dir)
    {
        static const char dir_str[] = "in\0out";
        char path[64];
        int fd;

        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/direction", pin);
        fd = open(path, O_WRONLY);
        if (fd < 0) 
        {
            printf("Failed to open gpio direction for writing!\n");
            return -1;
        }

        if (write(fd, &dir_str[dir == 0 ? 0 : 3], dir == 0 ? 2 : 3) < 0) 
        {
            printf("Failed to set direction!\n");
            return -1;
        }

        close(fd);
        return 0;
    }

    int gpio_write(int pin, int value)
    {
        static const char values_str[] = "01";
        char path[64];
        int fd;

        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
        fd = open(path, O_WRONLY);
        if (fd < 0) 
        {
            printf("Failed to open gpio%d value for writing!\n",pin);
            return -1;
        }

        if (write(fd, &values_str[value == 0 ? 0 : 1], 1) < 0) 
        {
            printf("Failed to write value!\n");
            return -1;
        }

        close(fd);
        return 0;
    }

    static int gpio_read(int pin)
    {
        char path[64];
        char value_str[3];
        int fd;

        snprintf(path, sizeof(path), "/sys/class/gpio/gpio%d/value", pin);
        fd = open(path, O_RDONLY);
        if (fd < 0) 
        {
            printf("Failed to open gpio value for reading!\n");
            return -1;
        }

        if (read(fd, value_str, 3) < 0) 
        {
            printf("Failed to read value!\n");
            return -1;
        }

        close(fd);
        return (atoi(value_str));
    }
};

#endif /* MOTOR_CANLIB_H_ */
