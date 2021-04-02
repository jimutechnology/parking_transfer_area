/*
 * ros_serial.h
 *
 *  Created on: 2018年8月8日
 *      Author: dongyu
 */

#ifndef TRANSFER_AREA_H_
#define TRANSFER_AREA_H_
#include <cstring>
#include <string.h>
#include "ros/ros.h"
#include "transfer_area/Door.h"

using namespace std;
// pin list

//光幕
#define  EZ_SCREEN_INSIDE                       5
#define  EZ_SCREEN_OUTSIDE                      19

//防夹
#define  CLAMP_INSIDE                           6
#define  CLAMP_OUTSIDE                          16

//限位开关
#define  LIMIT_UP_INSIDE                        13
#define  LIMIT_UP_OUTSIDE                       20

#define  LIMIT_DOWN_INSIDE                      12
#define  LIMIT_DOWN_OUTSIDE                     26

#define DOOR_OUTSIDE_OPEN_GPIO_PIN				15
#define DOOR_OUTSIDE_CLOSE_GPIO_PIN				22
#define DOOR_OUTSIDE_STOP_GPIO_PIN				18

#define DOOR_INSIDE_OPEN_GPIO_PIN				14
#define DOOR_INSIDE_CLOSE_GPIO_PIN				27
#define DOOR_INSIDE_STOP_GPIO_PIN				17

#define INSIDE_DISPLAY_PIN_1    				24
#define INSIDE_DISPLAY_PIN_2    				9
#define INSIDE_DISPLAY_PIN_3    				11
#define INSIDE_DISPLAY_PIN_4    				7

#define OUTSIDE_DISPLAY_PIN_1    				23
#define OUTSIDE_DISPLAY_PIN_2    				10
#define OUTSIDE_DISPLAY_PIN_3    				25
#define OUTSIDE_DISPLAY_PIN_4    				8

#define CHASSIS_CHECK                           21

#define CMD_TIMEOUT_VALUE       15*1000     //25*1000ms

#define DOOR_ACTION_OPEN    0
#define DOOR_ACTION_CLOSE   1
#define DOOR_ACTION_STOP    2
#define DOOR_OUTSIDE        0
#define DOOR_INSIDE         1

#define DOOR_NUM 2

struct Sensor_State {
    bool ez_screen;
    bool clamp;
    bool limit_up;
    bool limit_down;
    uint8_t is_update;
};

struct Door_Ctrl {
    bool up;
    bool down;
    bool stop;
    bool is_update;
    bool time_start_flag;
    uint16_t time_clear_count;
};


typedef enum
{
  M_STOP = 0,
  M_UP = 1,
  M_DOWN = 2
}motor_cmd;

typedef enum
{
  D_FORWARD = 1,
  D_OK = 2,
  D_FORWARD_Q = 3,
  D_BACK = 4
}dispaly_cmd;

typedef enum
{
  FORWARD = 1,
  OK = 2,
  WIDE = 5,
  FREE = 0,
  LONG = 5,
  LOW  = 6,
  SHORT = 5,
  NARROW = 5,
  LEFT = 3,
  RIGHT = 3,
  YAW  = 3,
  STEERING = 3,
  BACKWARD  = 3,
  LEAVE = 6
}screen_state;

typedef enum
{
  DOOR_NULL = -1,
  DOOR_OPENED = 0,
  DOOR_CLOSED = 1
}DOOR_STATE;

typedef enum
{
  SENSOR_UPDATE_NULL = 0,
  EZ_SCREEN_UPDATE = 1,
  CLAMP_UPDATE = 2,
  LIMIIT_UP_UPDATE = 4,
  LIMIIT_DOWN_UPDATE = 8
}SENSOR_UPDATE;

struct Display_Ctrl {
    uint8_t code;
    bool is_update;
};

struct Trans_Area_Data {
	  struct Sensor_State 	  inside_sensor_state;
    struct Sensor_State 	  outside_sensor_state; 
    struct Door_Ctrl        inside_door_ctrl;
    struct Door_Ctrl        outside_door_ctrl;
    struct Display_Ctrl     inside_display_ctrl;
    struct Display_Ctrl     outside_display_ctrl;
    DOOR_STATE              inside_door_state;
    DOOR_STATE              outside_door_state;
    bool                    sensor_chassis_check;
    bool                    sensor_chassis_is_update;
};


struct Door_Cmd_Action
{
    uint8_t action;
    string  command_id;
    uint16_t wait_count;
    bool    is_vaild;
};
struct Door_Cmd_Record {
    struct Door_Cmd_Action  inside;
    struct Door_Cmd_Action  outside;
};

typedef enum
{
  CMD_UNLOCKED = 0x00,
  CMD_LOCKED   = 0x01
} CMD_LockTypeDef;

struct Door_Para
{
 	int8_t door_id;
 	int8_t position;
};

struct Door_Para_Group
{
 	string frame_id;
 	Door_Para cmd_group[4];
 	CMD_LockTypeDef Lock;
};

typedef enum
{
	CMD_EMPTY		=  0,
	CMD_READLY		=  1,
	CMD_SENDED		=  2,
	CMD_RUNNING		=  3,
	CMD_PREFETCH	=  4,
	CMD_FINISH		=  5,
	CMD_ERROR		=  6,
	CMD_TIMEOUT		=  7
}Cmd_Reply;

typedef enum
{
	ACTION_EMPTY		=  0,
	ACTION_SENDED		=  1,
    ACTION_RUNNING		=  2,
}Door_Action_Status;

typedef enum
{
	ACTION_OPEN		    =  0,
	ACTION_CLOSE		=  1,
    ACTION_STOP		    =  2,
	ACTION_CLEAR		=  3,
}Door_Action;

#endif /* TRANSFER_AREA_H_ */
