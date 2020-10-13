/*
 *  area_control.cpp
 *
 *  Created on: 2020
 *      Author: wxp
 */
#include <iostream>
#include <mutex>
#include "area_control.h"
#include "std_msgs/String.h"
#include "transfer_area/InfoOut.h"
#include "transfer_area/CommandReply.h"
#include "transfer_area/ScreenCmd.h"
#include "transfer_area/DoorCmd.h"
#include "transfer_area/LightCmd.h"
#include "transfer_area/LightCurtainState.h"
#include "transfer_area/UnderpanDetectionState.h"
#include "../../actuator/gpio/gpio.h"

using namespace std;

#define AREA_CONTROL "area_control"

#define POWER_STATUS_PIN    254
#define POWER_STATUS_PIN_STR	"254"

class AreaControl: public Gpio{
public:
	ros::NodeHandle     nh;

	ros::Publisher      error_info_pub;
    ros::Publisher      warning_info_pub;

    ros::Publisher      cmd_reply_pub;
    ros::Publisher      light_curtain_state_pub;
    ros::Publisher      underpan_detection_state_pub;

    ros::Subscriber     door_cmd_sub;
    ros::Subscriber     screen_cmd_sub;
    ros::Subscriber     light_cmd_sub;
    


    transfer_area::LightCurtainState light_curtain_state;
    transfer_area::UnderpanDetectionState underpan_detection_state;

    string inside_light_cmd_command_id;
    string outside_light_cmd_command_id;

    struct Trans_Area_Data  trans_area_data;
    uint16_t door_cmd_running_time=0;

    transfer_area::Door Door_type;

    Door_Action_Status outside_door_action_status = ACTION_EMPTY;
    Door_Action_Status inside_door_action_status = ACTION_EMPTY;
    Door_Action outside_door_action = ACTION_CLEAR;
    Door_Action inside_door_action = ACTION_CLEAR;

    Door_Para_Group door_para_group;
    transfer_area::CommandReply door_cmd_reply_data;

public:
    AreaControl() :
		Gpio(POWER_STATUS_PIN_STR)
    {

    }

    ~AreaControl() 
	{
    }
	

	void run(void);
	bool AreaControl_Init(void);

	void Warning(int16_t warning_code, string message) const;
    void Error(int16_t error_code, string message) const;

    void rx_door_cmd_data(const transfer_area::DoorCmd &msg);
    void rx_screen_cmd_data(const transfer_area::ScreenCmd& msg);
    void rx_light_cmd_data(const transfer_area::LightCmd& msg);

    int8_t get_door_state(uint8_t id);
    bool get_state_inside_ez_screen(void);
    bool get_state_outside_ez_screen(void);
    bool get_state_inside_clamp(void);
    bool get_state_outside_clamp(void);

    bool get_state_inside_limit_up(void);
    bool get_state_inside_limit_down(void);
    bool get_state_outside_limit_up(void);
    bool get_state_outside_limit_down(void);
    bool get_state_chassis_check(void);

    void set_code_inside_display(uint8_t num);
    void set_code_outside_display(uint8_t num);

    void transfer_area_sensor_clear(void);

    void send_door_action(transfer_area::Door door);
    Cmd_Reply door_cmd_check(Door_Para ap);
    void outside_door_thread(void);
    void inside_door_thread(void);

    void update_pin_state(void);
    void door_cmd_excute(void);
};

void AreaControl::Warning(int16_t warning_code, string message) const
{
	static transfer_area::InfoOut warning_info;
	warning_info.error_code = warning_code;
	warning_info.message = message;
	warning_info_pub.publish(warning_info);
}

// publish errors to specific ROS topic
void AreaControl::Error(int16_t error_code, string message) const
{
	static transfer_area::InfoOut error_info;
	error_info.error_code = error_code;
	error_info.message = message;
	error_info_pub.publish(error_info);
}

bool AreaControl::AreaControl_Init(void)
{
	// gpio_export(POWER_STATUS_PIN);
	// gpio_direction(POWER_STATUS_PIN,0);     //IN

    memset(&trans_area_data,0,sizeof(trans_area_data));
    trans_area_data.inside_sensor_state.limit_up = !(gpio_read(LIMIT_UP_INSIDE));
    trans_area_data.inside_sensor_state.limit_down = !(gpio_read(LIMIT_DOWN_INSIDE));
    trans_area_data.outside_sensor_state.limit_up  = !(gpio_read(LIMIT_UP_OUTSIDE));
    trans_area_data.outside_sensor_state.limit_down = !(gpio_read(LIMIT_DOWN_OUTSIDE));

    if(trans_area_data.inside_sensor_state.limit_up && !trans_area_data.inside_sensor_state.limit_down)
        trans_area_data.inside_door_state = DOOR_OPENED;
    else if(trans_area_data.inside_sensor_state.limit_down && !trans_area_data.inside_sensor_state.limit_up)
        trans_area_data.inside_door_state = DOOR_CLOSED;
    else
        trans_area_data.inside_door_state = DOOR_NULL;
    
    if(trans_area_data.outside_sensor_state.limit_up && !trans_area_data.outside_sensor_state.limit_down)
        trans_area_data.outside_door_state = DOOR_OPENED;
    else if(trans_area_data.outside_sensor_state.limit_down && !trans_area_data.outside_sensor_state.limit_up)
        trans_area_data.outside_door_state = DOOR_CLOSED;
    else
        trans_area_data.outside_door_state = DOOR_NULL;



    error_info_pub = nh.advertise<transfer_area::InfoOut>(string("error_info"), 3);
	warning_info_pub = nh.advertise<transfer_area::InfoOut>(string("warning_info"), 3);

    cmd_reply_pub = nh.advertise<transfer_area::CommandReply>(string("cmd_reply"), 3);
	light_curtain_state_pub = nh.advertise<transfer_area::LightCurtainState>(string("light_curtain_state"), 3);
    underpan_detection_state_pub = nh.advertise<transfer_area::UnderpanDetectionState>(string("underpan_detection_state"), 3);

    door_cmd_sub = nh.subscribe("door_cmd",  5, &AreaControl::rx_door_cmd_data, this);
    screen_cmd_sub = nh.subscribe("screen_cmd",  5, &AreaControl::rx_screen_cmd_data, this);
    light_cmd_sub = nh.subscribe("light_cmd",  5, &AreaControl::rx_light_cmd_data, this);

	return true;
}


int8_t AreaControl::get_door_state(uint8_t id)
{
    if(id == DOOR_INSIDE)
    {
        if(trans_area_data.inside_sensor_state.limit_up && !trans_area_data.inside_sensor_state.limit_down)
            trans_area_data.inside_door_state = DOOR_OPENED;
        else if(trans_area_data.inside_sensor_state.limit_down && !trans_area_data.inside_sensor_state.limit_up)
            trans_area_data.inside_door_state = DOOR_CLOSED;
        else
            trans_area_data.inside_door_state = DOOR_NULL;
        return trans_area_data.inside_door_state;
    }
    else if(id == DOOR_OUTSIDE)
    {
        if(trans_area_data.outside_sensor_state.limit_up && !trans_area_data.outside_sensor_state.limit_down)
            trans_area_data.outside_door_state = DOOR_OPENED;
        else if(trans_area_data.outside_sensor_state.limit_down && !trans_area_data.outside_sensor_state.limit_up)
            trans_area_data.outside_door_state = DOOR_CLOSED;
        else
            trans_area_data.outside_door_state = DOOR_NULL;
        return trans_area_data.outside_door_state;
    }
}

bool AreaControl::get_state_inside_ez_screen(void)
{
    return trans_area_data.inside_sensor_state.ez_screen;
}

bool AreaControl::get_state_outside_ez_screen(void)
{
    return trans_area_data.outside_sensor_state.ez_screen;
}

bool AreaControl::get_state_inside_clamp(void)
{
    return trans_area_data.inside_sensor_state.clamp;
}

bool AreaControl::get_state_outside_clamp(void)
{
    return trans_area_data.outside_sensor_state.clamp;
}

bool AreaControl::get_state_inside_limit_up(void)
{
    return trans_area_data.inside_sensor_state.limit_up;
}

bool AreaControl::get_state_inside_limit_down(void)
{
    return trans_area_data.inside_sensor_state.limit_down;
}

bool AreaControl::get_state_outside_limit_up(void)
{
    return trans_area_data.outside_sensor_state.limit_up;
}

bool AreaControl::get_state_outside_limit_down(void)
{
    return trans_area_data.outside_sensor_state.limit_down;
}

bool AreaControl::get_state_chassis_check(void)
{
    return trans_area_data.sensor_chassis_check;
}


void AreaControl::set_code_inside_display(uint8_t num)
{
    trans_area_data.inside_display_ctrl.code = num;

    trans_area_data.inside_display_ctrl.is_update = true;
}

void AreaControl::set_code_outside_display(uint8_t num)
{
    trans_area_data.outside_display_ctrl.code = num;

    trans_area_data.outside_display_ctrl.is_update = true;
}



void AreaControl::outside_door_thread(void)
{
    static uint16_t start_time_tick;
    if(get_state_outside_clamp() == true){
        outside_door_action = ACTION_OPEN;
        outside_door_action_status = ACTION_SENDED;
    }
    if(outside_door_action_status==ACTION_SENDED){
        gpio_write(DOOR_OUTSIDE_OPEN_GPIO_PIN, 0);
        gpio_write( DOOR_OUTSIDE_CLOSE_GPIO_PIN, 0);
        gpio_write( DOOR_OUTSIDE_STOP_GPIO_PIN, 0);
        switch(outside_door_action){
            case ACTION_OPEN: 
                gpio_write( DOOR_OUTSIDE_OPEN_GPIO_PIN, 1);
                break;
            case ACTION_CLOSE:
                gpio_write( DOOR_OUTSIDE_CLOSE_GPIO_PIN, 1);
                break;
            case ACTION_STOP:
                gpio_write( DOOR_OUTSIDE_STOP_GPIO_PIN, 1);
                break;
            default:
                break;
        }
        start_time_tick=0;
        outside_door_action_status = ACTION_RUNNING;
    }
    else if(outside_door_action_status = ACTION_RUNNING){
        start_time_tick++;
        if(start_time_tick>600){
            gpio_write( DOOR_OUTSIDE_OPEN_GPIO_PIN, 0);
            gpio_write( DOOR_OUTSIDE_CLOSE_GPIO_PIN, 0);
            gpio_write( DOOR_OUTSIDE_STOP_GPIO_PIN, 0);
            outside_door_action_status = ACTION_EMPTY;
            start_time_tick=0;
        }
    }
}
void AreaControl::inside_door_thread(void)
{
    static uint16_t start_time_tick;
    if(get_state_inside_clamp() == true){
        inside_door_action = ACTION_OPEN;
        inside_door_action_status = ACTION_SENDED;
        return;
    }
    if(inside_door_action_status==ACTION_SENDED){
        printf("action: %d\r\n", inside_door_action);
        gpio_write( DOOR_INSIDE_OPEN_GPIO_PIN, 0);
        gpio_write( DOOR_INSIDE_CLOSE_GPIO_PIN, 0);
        gpio_write( DOOR_INSIDE_STOP_GPIO_PIN, 0);
        switch(inside_door_action){
            case ACTION_OPEN: 
                gpio_write( DOOR_INSIDE_OPEN_GPIO_PIN, 1);
                break;
            case ACTION_CLOSE:
                gpio_write( DOOR_INSIDE_CLOSE_GPIO_PIN, 1);
                break;
            case ACTION_STOP:
                gpio_write( DOOR_INSIDE_STOP_GPIO_PIN, 1);
                printf("stop\r\n");
                break;
            default:
                break;
        }
        start_time_tick=0;
        inside_door_action_status = ACTION_RUNNING;
    }
    else if(inside_door_action_status = ACTION_RUNNING){
        start_time_tick++;
        if(start_time_tick>600){
            gpio_write( DOOR_INSIDE_OPEN_GPIO_PIN, 0);
            gpio_write( DOOR_INSIDE_CLOSE_GPIO_PIN, 0);
            gpio_write( DOOR_INSIDE_STOP_GPIO_PIN, 0);
            inside_door_action_status = ACTION_EMPTY;
            start_time_tick=0;
        }
    }
}


void AreaControl::send_door_action(transfer_area::Door door)
{
    if(door.id == door.INSIDE){
        if(door.position == door.OPEN){
            inside_door_action = ACTION_OPEN;
        }
        else if(door.position == door.CLOSE){
            inside_door_action = ACTION_CLOSE;
        }
        inside_door_action_status = ACTION_SENDED;
    }

    if(door.id == door.OUTSIDE){
        if(door.position == door.OPEN){
            outside_door_action = ACTION_OPEN;
        }
        else if(door.position == door.CLOSE){
            outside_door_action = ACTION_CLOSE;
        }
        outside_door_action_status = ACTION_SENDED;
    }
    door_cmd_running_time=0;
}

Cmd_Reply AreaControl::door_cmd_check(Door_Para ap)
{
    if(ap.door_id == Door_type.INSIDE){
        if((ap.position == Door_type.OPEN && get_state_inside_limit_up()==true) ||
            (ap.position == Door_type.CLOSE && get_state_inside_limit_down()==true))
        {
            inside_door_action = ACTION_STOP;
            inside_door_action_status = ACTION_SENDED;
            return CMD_FINISH;
        }
    }
    if(ap.door_id == Door_type.OUTSIDE){
        if((ap.position == Door_type.OPEN && get_state_outside_limit_up()==true) ||
            (ap.position == Door_type.CLOSE && get_state_outside_limit_down()==true))
        {
            outside_door_action = ACTION_STOP;
            outside_door_action_status = ACTION_SENDED;
            return CMD_FINISH;
        }
    }
}

void AreaControl::door_cmd_excute(void)
{
    if(door_cmd_reply_data.state == CMD_RUNNING)
    {
        door_para_group.Lock = CMD_LOCKED;
        uint8_t cmd_cnt = 0;
        for(uint8_t i = 0; i < DOOR_NUM; i++)
        {
            if(door_para_group.cmd_group[i].door_id == i+1)
            {
                Cmd_Reply ret = CMD_EMPTY;
                ret = door_cmd_check(door_para_group.cmd_group[i]);
                if(ret == CMD_FINISH)
                {
                    door_para_group.cmd_group[i].door_id = 0;
                    cmd_cnt ++;
                }
                else if(ret == CMD_ERROR || ret == CMD_TIMEOUT)
                {
                    door_cmd_reply_data.state = (int32_t)ret;
                    door_cmd_reply_data.message = "door command execute error";
                    cmd_reply_pub.publish(door_cmd_reply_data);
                    memset(&door_para_group, 0, sizeof(Door_Para_Group));
                    door_cmd_reply_data.state = CMD_EMPTY;
                    break;
                }
            }
            else
            {
                cmd_cnt++;
            }
        }
        if(cmd_cnt >= DOOR_NUM)
        {
            door_cmd_reply_data.state = CMD_FINISH;
            door_cmd_reply_data.message = "door command execute finish";
            cmd_reply_pub.publish(door_cmd_reply_data);
            memset(&door_para_group, 0, sizeof(Door_Para_Group));
            door_para_group.Lock = CMD_UNLOCKED;
        }
        door_cmd_running_time ++;
        if(door_cmd_running_time>CMD_TIMEOUT_VALUE)
        {
            door_cmd_reply_data.state = CMD_TIMEOUT;
            door_cmd_reply_data.message = "door command execute timeout";
            cmd_reply_pub.publish(door_cmd_reply_data);
            memset(&door_para_group, 0, sizeof(Door_Para_Group));
            door_cmd_reply_data.state = CMD_EMPTY;
        }
        
    }
}

void AreaControl::rx_door_cmd_data(const transfer_area::DoorCmd &msg) 
{
    if(msg.header.frame_id == " ")
		return;
	if(door_para_group.Lock == CMD_LOCKED){
		transfer_area::CommandReply reply_data;
		string cmd_id;
		cmd_id = msg.header.frame_id;
		reply_data.message = "door cmd is running";
		reply_data.state = door_cmd_reply_data.STATE_ERROR;
		reply_data.command_id = cmd_id;
		cmd_reply_pub.publish(reply_data);
		return;
	}
		
	memset(&door_para_group, 0, sizeof(Door_Para_Group));

    door_para_group.frame_id = msg.header.frame_id;
	bool legal_cmd = (msg.door_length > 0 && msg.door_length <= DOOR_NUM) ? true : false;
	vector<int> door_id_list;

	uint8_t door_id = -1;
	for(uint8_t i = 0; i < msg.door_length; i++)
	{
		door_id = msg.door[i].id;
		// check if two commands point to a single door
		if(find(door_id_list.begin(), door_id_list.end(), door_id) != door_id_list.end()){
			legal_cmd = false;
			door_cmd_reply_data.message = "door id repeat ";
			break;
		}
		door_id_list.push_back(door_id);
		if(door_id > 0 && door_id <= DOOR_NUM){
			door_para_group.cmd_group[door_id - 1].door_id = msg.door[i].id;
			door_para_group.cmd_group[door_id - 1].position = msg.door[i].position;
			send_door_action(msg.door[i]);
		}
		else
		{
			legal_cmd = false;
			door_cmd_reply_data.message = "door id is out of range";
			door_cmd_reply_data.state = door_cmd_reply_data.STATE_ERROR;
		}
	}

	if(legal_cmd)
	{
		door_cmd_reply_data.state = door_cmd_reply_data.STATE_RUNNING;
		door_cmd_reply_data.message = "receive door command ";
	}
	door_cmd_reply_data.command_id = door_para_group.frame_id;
	cmd_reply_pub.publish(door_cmd_reply_data);
}

void AreaControl::rx_screen_cmd_data(const transfer_area::ScreenCmd& msg) 
{
    if(msg.id == 0)
	{
		set_code_inside_display(msg.state);
	}
	if(msg.id == 1)
	{
		set_code_outside_display(msg.state);
	}
}

void AreaControl::rx_light_cmd_data(const transfer_area::LightCmd& msg) 
{
    transfer_area::CommandReply reply_data;
    if(msg.id == 0)
	{
        inside_light_cmd_command_id = msg.header.frame_id;
		reply_data.command_id = inside_light_cmd_command_id;
	}
	if(msg.id == 1)
	{
        outside_light_cmd_command_id = msg.header.frame_id;
		reply_data.command_id = outside_light_cmd_command_id;
	}

    printf("recv light cmd id[%d] state[%d]\r\n",msg.id,msg.state);
    // reply_data.command_id = msg.id;
	// reply_data.state = transfer_area::CommandReply::STATE_SENDED;
	// reply_data.error_code = 0;
	// reply_data.message = "";
	// cmd_reply_pub.publish(&reply_data);
}

void AreaControl::run(void)
{
	static uint16_t task_cnt=0;
	task_cnt++;

    update_pin_state();
    //控制命令执行
    outside_door_thread();
    inside_door_thread();
    door_cmd_excute();
    //内侧传感器状态检测
    if(trans_area_data.inside_sensor_state.is_update != 0)
    {
        if((trans_area_data.inside_sensor_state.is_update & EZ_SCREEN_UPDATE) != 0)
        {
             if(trans_area_data.inside_sensor_state.ez_screen)
            {
                light_curtain_state.timestamp = ros::Time::now();
                light_curtain_state.id = 0;
                light_curtain_state.state = true;
                light_curtain_state_pub.publish(light_curtain_state);
            }
            else
            {
                light_curtain_state.timestamp = ros::Time::now();
                light_curtain_state.id = 0;
                light_curtain_state.state = false;
                light_curtain_state_pub.publish(light_curtain_state);
            }
            trans_area_data.inside_sensor_state.is_update &= (~EZ_SCREEN_UPDATE);
        }
        if((trans_area_data.inside_sensor_state.is_update & CLAMP_UPDATE) != 0)
        {
            trans_area_data.inside_sensor_state.is_update &= (~CLAMP_UPDATE);
        }
    }
    //外侧传感器状态检查
    if(trans_area_data.outside_sensor_state.is_update != 0)
    {
        if((trans_area_data.outside_sensor_state.is_update & EZ_SCREEN_UPDATE) != 0)
        {
             if(trans_area_data.outside_sensor_state.ez_screen)
            {
                light_curtain_state.timestamp = ros::Time::now();
                light_curtain_state.id = 1;
                light_curtain_state.state = true;
                light_curtain_state_pub.publish(light_curtain_state);
            }
            else
            {
                light_curtain_state.timestamp = ros::Time::now();
                light_curtain_state.id = 1;
                light_curtain_state.state = false;
                light_curtain_state_pub.publish(light_curtain_state);
            }
            trans_area_data.outside_sensor_state.is_update &= (~EZ_SCREEN_UPDATE);
        }

        if((trans_area_data.outside_sensor_state.is_update & CLAMP_UPDATE) != 0)
        {
            trans_area_data.outside_sensor_state.is_update &= (~CLAMP_UPDATE);
        }



    }
    if(trans_area_data.sensor_chassis_is_update)
    {
        if(trans_area_data.sensor_chassis_check)
        {
            underpan_detection_state.timestamp = ros::Time::now();
            underpan_detection_state.state = false;         //不符合
            underpan_detection_state_pub.publish(underpan_detection_state);
        }
        else
        {
            ;
        }
        trans_area_data.sensor_chassis_is_update = false;
    }


    //前门  提示屏
    if(trans_area_data.inside_display_ctrl.is_update)
    {
        gpio_write( INSIDE_DISPLAY_PIN_1, (int)(trans_area_data.inside_display_ctrl.code & 0x01));
        gpio_write( INSIDE_DISPLAY_PIN_2, (int)(trans_area_data.inside_display_ctrl.code & 0x02));
        gpio_write( INSIDE_DISPLAY_PIN_3, (int)(trans_area_data.inside_display_ctrl.code & 0x04));
        gpio_write( INSIDE_DISPLAY_PIN_4, (int)(trans_area_data.inside_display_ctrl.code & 0x08));
        trans_area_data.inside_display_ctrl.is_update = false;
    }

    //后门  提示屏
    if(trans_area_data.outside_display_ctrl.is_update)
    {
        gpio_write( OUTSIDE_DISPLAY_PIN_1, (int)(trans_area_data.outside_display_ctrl.code & 0x01));
        gpio_write( OUTSIDE_DISPLAY_PIN_2, (int)(trans_area_data.outside_display_ctrl.code & 0x02));
        gpio_write( OUTSIDE_DISPLAY_PIN_3, (int)(trans_area_data.outside_display_ctrl.code & 0x04));
        gpio_write( OUTSIDE_DISPLAY_PIN_4, (int)(trans_area_data.outside_display_ctrl.code & 0x08));
        trans_area_data.outside_display_ctrl.is_update = false;
    }
}


void AreaControl::transfer_area_sensor_clear(void)
{
    if(0 == gpio_read(EZ_SCREEN_INSIDE))
    {
        if(trans_area_data.inside_sensor_state.ez_screen)
        {
            trans_area_data.inside_sensor_state.is_update |= EZ_SCREEN_UPDATE;
            trans_area_data.inside_sensor_state.ez_screen = false;
            printf("reset screen at inside\r\n");    
        }
    }

    if(0 == gpio_read(EZ_SCREEN_OUTSIDE))
    {
        if(trans_area_data.outside_sensor_state.ez_screen)
        {
            trans_area_data.outside_sensor_state.is_update |= EZ_SCREEN_UPDATE;
            trans_area_data.outside_sensor_state.ez_screen = false;
            printf("reset screen at outside\r\n");
        } 
    }

    if(0 == gpio_read(CLAMP_INSIDE))
    {
        if(trans_area_data.inside_sensor_state.clamp)
        {
            trans_area_data.inside_sensor_state.is_update |= CLAMP_UPDATE;
            trans_area_data.inside_sensor_state.clamp = false;
            printf("reset clamp at inside\r\n"); 
        }
    }

    if(0 == gpio_read(CLAMP_OUTSIDE))
    {   
        if(trans_area_data.outside_sensor_state.clamp)
        {
            trans_area_data.outside_sensor_state.is_update |= CLAMP_UPDATE;
            trans_area_data.outside_sensor_state.clamp = false;
            printf("reset clamp at outside\r\n"); 
        }
    }

    if(1 == gpio_read(LIMIT_UP_INSIDE))
    {
        if(trans_area_data.inside_sensor_state.limit_up)
        {
            trans_area_data.inside_sensor_state.is_update |= LIMIIT_UP_UPDATE;
            trans_area_data.inside_sensor_state.limit_up = false;
            printf("reset limit up at inside\r\n");
        }
         
    }

    if(1 == gpio_read(LIMIT_UP_OUTSIDE))
    {
        if(trans_area_data.outside_sensor_state.limit_up)
        {
            trans_area_data.outside_sensor_state.is_update |= LIMIIT_UP_UPDATE;
            trans_area_data.outside_sensor_state.limit_up = false;
            printf("reset limit up at outside\r\n"); 
        }
    }

    if(1 == gpio_read(LIMIT_DOWN_INSIDE))
    {
        if(trans_area_data.inside_sensor_state.limit_down)
        {
            trans_area_data.inside_sensor_state.is_update |= LIMIIT_DOWN_UPDATE;
            trans_area_data.inside_sensor_state.limit_down = false;
            printf("reset limit down at inside\r\n"); 
        }

    }

    if(1 == gpio_read(LIMIT_DOWN_OUTSIDE))
    {
        if(trans_area_data.outside_sensor_state.limit_down)
        {
            trans_area_data.outside_sensor_state.is_update |= LIMIIT_DOWN_UPDATE;
            trans_area_data.outside_sensor_state.limit_down = false;
            printf("reset limit down at outside\r\n"); 
        }
    }
    
    if(0 == gpio_read(CHASSIS_CHECK))
    {
        if(trans_area_data.sensor_chassis_check)
        {
            trans_area_data.sensor_chassis_check = false;
            trans_area_data.sensor_chassis_is_update = true;
            printf("reset chassis check\r\n");
        } 
    }

}


void AreaControl::update_pin_state(void)
{
    if(1 == gpio_read(EZ_SCREEN_INSIDE))         //前门光幕
    {
        usleep(15000);//15ms
        if(1 == gpio_read(EZ_SCREEN_INSIDE))
        {
            trans_area_data.inside_sensor_state.ez_screen = true;
            trans_area_data.inside_sensor_state.is_update |= EZ_SCREEN_UPDATE;
            printf("check screen at inside\r\n"); 
        }
    }
    if(1 == gpio_read(EZ_SCREEN_OUTSIDE))        //后门光幕
    {
        usleep(15000);//15ms
        if(1 == gpio_read(EZ_SCREEN_OUTSIDE))
        {
            trans_area_data.outside_sensor_state.ez_screen = true;
            trans_area_data.outside_sensor_state.is_update |= EZ_SCREEN_UPDATE;
            printf("check screen at outside\r\n"); 
        }
    }
    if(1 == gpio_read(CLAMP_INSIDE))             //前门防夹
    {
        usleep(15000);//15ms
        if(1 == gpio_read(CLAMP_INSIDE))
        {
            trans_area_data.inside_sensor_state.clamp = true;
            trans_area_data.inside_sensor_state.is_update |= CLAMP_UPDATE;
            printf("check clamp at inside\r\n"); 
        }
    }
    if(1 == gpio_read(CLAMP_OUTSIDE))            //后门防夹
    {
        usleep(15000);//15ms
        if(1 == gpio_read(CLAMP_OUTSIDE))
        {
            trans_area_data.outside_sensor_state.clamp = true;
            trans_area_data.outside_sensor_state.is_update |= CLAMP_UPDATE;
            printf("check clamp at outside\r\n"); 
        }
    }
    if(0 == gpio_read(LIMIT_UP_INSIDE))          //前门上限位
    {
        usleep(15000);//15ms
        if(0 == gpio_read(LIMIT_UP_INSIDE))
        {
            trans_area_data.inside_sensor_state.limit_up = true;
            trans_area_data.inside_sensor_state.is_update |= LIMIIT_UP_UPDATE;
            printf("check limit up at inside\r\n"); 
        }
    }
    if(0 == gpio_read(LIMIT_UP_OUTSIDE))          //后门  上限位
    {
        usleep(15000);//15ms
        if(0 == gpio_read(LIMIT_UP_OUTSIDE))
        {
            trans_area_data.outside_sensor_state.limit_up = true;
            trans_area_data.outside_sensor_state.is_update |= LIMIIT_UP_UPDATE;
            printf("check limit up at outside\r\n"); 
        }
    }
    if(0 == gpio_read(LIMIT_DOWN_INSIDE))        //前门  下限位
    {
        usleep(15000);//15ms
        if(0 == gpio_read(LIMIT_DOWN_INSIDE))
        {
            trans_area_data.inside_sensor_state.limit_down = true;
            trans_area_data.inside_sensor_state.is_update |= LIMIIT_DOWN_UPDATE;
            printf("check limit down at inside\r\n"); 
        }
    }
    if(0 == gpio_read(LIMIT_DOWN_OUTSIDE))        //后门  下限位
    {
        usleep(15000);//15ms
        if(0 == gpio_read(LIMIT_DOWN_OUTSIDE))
        {
            trans_area_data.outside_sensor_state.limit_down = true;
            trans_area_data.outside_sensor_state.is_update |= LIMIIT_DOWN_UPDATE;
            printf("check limit down at outside\r\n"); 
        }
    }
    if(1 == gpio_read(CHASSIS_CHECK))       //地盘检测
    {
        usleep(15000);//15ms
        if(1 == gpio_read(CHASSIS_CHECK))
        {
            trans_area_data.sensor_chassis_check = true;
            trans_area_data.sensor_chassis_is_update = true;
            printf("check chassis check\r\n");
        }
    }

}

/* robot_command main application */
int main(int argc, char **argv)
{
    ros::init(argc, argv, AREA_CONTROL);

    ros::Rate r(100);

    AreaControl area_ctrl;

    // if (!area_control.init())
    //     return EXIT_FAILURE;

	area_ctrl.AreaControl_Init();

    while(ros::ok())
    {
        area_ctrl.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
