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
#include "transfer_area/MotorPosition.h"
#include "transfer_area/MotorCmd.h"
#include "transfer_area/MotorLock.h"
#include "transfer_area/InfoOut.h"
#include "transfer_area/LightCurtainState.h"
#include "transfer_area/ScreenCmd.h"
#include "car_scanner/CarInfo.h"
#include "car_scanner/WheelArray.h"
#include "car_scanner/Wheel.h"
#include "car_scanner/CarState.h"
#include "../../actuator/gpio/gpio.h"
#include <sys/time.h>

using namespace std;

#define AREA_CONTROL "area_control"

// #define ENABLE_MOTOR_AUTO_DOWN

#define  AREA_CONTROL_HZ        100
#define  MOTOR_UP_TIMEOUT       5*60*AREA_CONTROL_HZ
#define  MOTOR_AUTO_DOWN_TIMEOUT       35*AREA_CONTROL_HZ
#define  CMD_TIMEOUT            6*AREA_CONTROL_HZ
#define  SCREEN_TIGGER_TIMEOUT  1*AREA_CONTROL_HZ
#define  LIDAR_READY_CLEAR_TIMEOUT      1*AREA_CONTROL_HZ/2
#define  SCREEN_TIGGER_WAIT_TIMEOUT     3*60*AREA_CONTROL_HZ

#define  OUTSIDE_SCREEN_ID      1
#define  INSIDE_SCREEN_ID       0

#define  GPIO_VALUE_HIGH        1
#define  GPIO_VALUE_LOW         0

#define  M_LIMIT_UP_PIN         26
#define  M_LIMIT_DOWN_PIN       20   //触发是低电平
#define  EZ_SCREEN_INSIDE_PIN   19
#define  EZ_SCREEN_OUTSIDE_PIN  5
#define  MOTOR_UP_PIN           15
#define  MOTOR_DOWN_PIN         22
#define  DISPLAY_A1_PIN         24
#define  DISPLAY_A2_PIN         9
#define  DISPLAY_A3_PIN         11
#define  DISPLAY_A4_PIN         25   //7   /*由于树梅派gpio7被用作spi，暂时使用gpio25（外提示屏的3脚代替）*/

#define  M_LIMIT_UP_PIN_STR     "20"
#define  M_LIMIT_DOWN_PIN_STR   "26"
#define  EZ_SCREEN_INSIDE_PIN_STR    "19"
#define  EZ_SCREEN_OUTSIDE_PIN_STR    "5"
#define  MOTOR_UP_PIN_STR       "15"
#define  MOTOR_DOWN_PIN_STR     "22"
#define  DISPLAY_A1_PIN_STR     "24"
#define  DISPLAY_A2_PIN_STR     "9"
#define  DISPLAY_A3_PIN_STR     "11"
#define  DISPLAY_A4_PIN_STR     "25"    //7   /*由于树梅派gpio7被用作spi，暂时使用gpio25（外提示屏的3脚代替）*/

using namespace car_scanner;
using namespace std;

class AreaControl{
public:
	ros::NodeHandle     nh;

	ros::Publisher      error_info_pub;
    ros::Publisher      warning_info_pub;

    ros::Publisher      light_curtain_pub;
    ros::Publisher      motor_pos_pub;

    ros::Subscriber     car_state_sub;
    ros::Subscriber     car_info_sub;
    ros::Subscriber     wheel_info_sub;
    ros::Subscriber     screen_cmd_sub;
    ros::Subscriber     motor_cmd_sub;
    ros::Subscriber     motor_lock_sub;
    

    transfer_area::LightCurtainState light_curtain_data;
    transfer_area::MotorPosition motor_pos_data;
    
    // transfer_area::ScreenCmd screen_data;

    motor_cmd m_cmd = M_STOP;
    bool delay_flag = false;
    motor_poistion m_pos = M_POSITION_UNKNOWN;
    bool is_up_limit_active = M_LIMIT_UP->gpio_read(M_LIMIT_UP_PIN) == GPIO_VALUE_LOW;
    bool is_down_limit_active = M_LIMIT_DOWN->gpio_read(M_LIMIT_DOWN_PIN) == GPIO_VALUE_LOW;
    bool is_up_limit_active_last = is_up_limit_active;
    bool is_down_limit_active_last = is_down_limit_active;


    dispaly_cmd d_cmd = D_FORWARD_Q;

    car_scanner::CarState car_state;
    car_scanner::CarInfo car_info;

    // intput
    Gpio *M_LIMIT_UP = nullptr;
	Gpio *M_LIMIT_DOWN = nullptr;
    Gpio *EZ_SCREEN_OUTSIDE = nullptr;
	Gpio *EZ_SCREEN_INSIDE = nullptr;

    // output
    Gpio *MOTOR_UP = nullptr;
	Gpio *MOTOR_DOWN = nullptr;

    Gpio *DISPLAY_A1 = nullptr;     //rad-forward
	Gpio *DISPLAY_A2 = nullptr;     //green-ok
    Gpio *DISPLAY_A3 = nullptr;     //yellow-forward-quiet
	Gpio *DISPLAY_A4 = nullptr;     //back

    cmd_state c_state = STATE_EMPTY;
    car_check_step car_check_state = C_NONE;
    bool car_check_state_flag = false;

    bool is_lidar_scan_wheel[2] = {false,false};
    bool is_screen_tigger[2] = {false,false};

    struct timeval stamp;
    uint64_t lidar_scan_update_time[2];
    uint64_t wheel_info_update_time;
    uint64_t now;

    bool is_motor_lock = false;

public:
    AreaControl()
    {
		M_LIMIT_UP = new Gpio(M_LIMIT_UP_PIN_STR);
		M_LIMIT_DOWN = new Gpio(M_LIMIT_DOWN_PIN_STR);
        EZ_SCREEN_OUTSIDE = new Gpio(EZ_SCREEN_OUTSIDE_PIN_STR);
        EZ_SCREEN_INSIDE = new Gpio(EZ_SCREEN_INSIDE_PIN_STR);

        MOTOR_UP = new Gpio(MOTOR_UP_PIN_STR);
        MOTOR_DOWN = new Gpio(MOTOR_DOWN_PIN_STR);

        DISPLAY_A1 = new Gpio(DISPLAY_A1_PIN_STR);
		DISPLAY_A2 = new Gpio(DISPLAY_A2_PIN_STR);
        DISPLAY_A3 = new Gpio(DISPLAY_A3_PIN_STR);
        DISPLAY_A4 = new Gpio(DISPLAY_A4_PIN_STR);
    }

    ~AreaControl() 
	{
        MOTOR_UP->gpio_write(MOTOR_UP_PIN,GPIO_VALUE_LOW);
        MOTOR_DOWN->gpio_write(MOTOR_DOWN_PIN,GPIO_VALUE_LOW);
        
        delete M_LIMIT_UP;
        delete M_LIMIT_DOWN;
        delete EZ_SCREEN_OUTSIDE;
        delete EZ_SCREEN_INSIDE;
        delete MOTOR_UP;
        delete MOTOR_DOWN;
        delete DISPLAY_A1;
        delete DISPLAY_A2;
        delete DISPLAY_A3;
        delete DISPLAY_A4;
    }
	

	void run(void);
	bool AreaControl_Init(void);

	void Warning(int16_t warning_code, string message) const;
    void Error(int16_t error_code, string message) const;

    void rx_car_info_data(const car_scanner::CarInfo msg);
    void rx_wheel_info_data(const car_scanner::WheelArray msg);
    void rx_car_state_data(const car_scanner::CarState msg);

    void area_cmd_update(void);
    bool set_motor_cmd(motor_cmd cmd);
    void motor_cmd_update(void);
    void motor_control(void);
    void display_control(void);
    void update_ez_screen_state(void);
    void rx_screen_cmd(const transfer_area::ScreenCmd msg);
    void rx_motor_cmd(const transfer_area::MotorCmd msg);
    void rx_motor_lock(const transfer_area::MotorLock msg);
    
    motor_poistion update_motor_position(void);

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
    error_info_pub = nh.advertise<transfer_area::InfoOut>(string("error_info"), 3);
	warning_info_pub = nh.advertise<transfer_area::InfoOut>(string("warning_info"), 3);

    light_curtain_pub = nh.advertise<transfer_area::LightCurtainState>(string("light_curtain_state"), 3);
    motor_pos_pub = nh.advertise<transfer_area::MotorPosition>(string("motor_pos"), 3);
    
    car_state_sub = nh.subscribe("car_state",  5, &AreaControl::rx_car_state_data, this);
    car_info_sub = nh.subscribe("car_info",  5, &AreaControl::rx_car_info_data, this);
    wheel_info_sub = nh.subscribe("wheel_info",  5, &AreaControl::rx_wheel_info_data, this);
    screen_cmd_sub = nh.subscribe("screen_cmd",  5, &AreaControl::rx_screen_cmd, this);
    motor_cmd_sub = nh.subscribe("motor_cmd",  5, &AreaControl::rx_motor_cmd, this);
    motor_lock_sub = nh.subscribe("motor_lock",  5, &AreaControl::rx_motor_lock, this);
    


	return true;
}

void AreaControl::rx_car_info_data(const car_scanner::CarInfo msg)
{    
    memcpy(&car_info,&msg,sizeof(car_scanner::CarInfo));
}
void AreaControl::rx_wheel_info_data(const car_scanner::WheelArray msg)
{
    gettimeofday(&stamp, NULL);
    wheel_info_update_time = stamp.tv_sec*1000000 + stamp.tv_usec;
    if(msg.header.frame_id == "laser_1")
    {
        lidar_scan_update_time[0] = stamp.tv_sec*1000000 + stamp.tv_usec;
    }
    

    if(msg.header.frame_id == "laser_2")
    {
        lidar_scan_update_time[1] = stamp.tv_sec*1000000 + stamp.tv_usec;
    }
}

void AreaControl::rx_car_state_data(const car_scanner::CarState msg)
{
    #ifdef ENABLE_MOTOR_AUTO_DOWN
    static uint16_t available_cnt = 0;
    if(msg.is_car_available)
    {
        available_cnt++;
        if(available_cnt > MOTOR_AUTO_DOWN_TIMEOUT)
        {
            set_motor_cmd(M_DOWN);
        }
    }
    else
    {
        available_cnt = 0;
    }
    #endif

    memcpy(&car_state,&msg,sizeof(car_scanner::CarState));
}

void AreaControl::rx_screen_cmd(const transfer_area::ScreenCmd msg)
{
    if(msg.id == 1)
    {
        if(msg.state == 0)
        {
            d_cmd = D_FORWARD_Q;
        }
        if(msg.state == 1)
        {
            d_cmd = D_FORWARD;
        }
        else if(msg.state == 2)
        {
            d_cmd = D_OK;
        }
        else if(msg.state == 3 || msg.state == 5 || msg.state == 6)
        {
            d_cmd = D_BACK;
        }
    }
}

void AreaControl::rx_motor_cmd(const transfer_area::MotorCmd msg)
{
    if(msg.cmd == msg.CMD_UP)
    {
        set_motor_cmd(M_UP);
    }
    else if(1)//car_state.is_car_available)
    {
        if(msg.cmd == msg.CMD_DOWN)
        {
            set_motor_cmd(M_DOWN);
        }
    }
}
void AreaControl::rx_motor_lock(const transfer_area::MotorLock msg)
{
    is_motor_lock = msg.enable_lock;
}

bool AreaControl::set_motor_cmd(motor_cmd cmd)
{
    // 调度加锁
    if(cmd == M_UP && is_motor_lock)
        return false;
    if(m_pos == M_POSITION_UP && cmd == M_UP)
        return true;
    if(m_pos == M_POSITION_DOWN && cmd == M_DOWN)
        return true;
    //printf("test-a\r\n");
    if(c_state != STATE_RUNNING)
    {
        //printf("test-b\r\n");
        c_state = STATE_RUNNING;
        delay_flag = true;

        m_cmd = cmd;
        if(cmd == M_DOWN)
            car_check_state = C_NONE;
        return true;      
    }
    return false;
}
motor_poistion AreaControl::update_motor_position(void)
{
    is_up_limit_active_last = is_up_limit_active;
    is_down_limit_active_last = is_down_limit_active;
    is_up_limit_active = M_LIMIT_UP->gpio_read(M_LIMIT_UP_PIN) == GPIO_VALUE_LOW;
    is_down_limit_active = M_LIMIT_DOWN->gpio_read(M_LIMIT_DOWN_PIN) == GPIO_VALUE_LOW;

    // 上下限位同时跳变，异常情况，无法确定位置
    if((is_up_limit_active_last != is_up_limit_active) && (is_down_limit_active_last != is_down_limit_active))
        m_pos = M_POSITION_UNKNOWN;//error
    else if(is_up_limit_active_last != is_up_limit_active)
    {
        // 上限位由deactive-->active 
        if(is_up_limit_active)
            m_pos = M_POSITION_UP; 
        // 上限位由active-->deactive 
    }
    else if(is_down_limit_active_last != is_down_limit_active)
    {
        // 下限位由deactive-->active   
        if(is_down_limit_active)
            m_pos = M_POSITION_DOWN;
        // 下限位由active-->deactive  
    }
    else if(is_up_limit_active && !is_down_limit_active)
    {
        m_pos = M_POSITION_UP;
    }
    else if(!is_up_limit_active && is_down_limit_active)
    {
        m_pos = M_POSITION_DOWN;
    }
    // 上 下限位都deactive
    else if((!is_up_limit_active) && (!is_down_limit_active))
    {
        m_pos = M_POSITION_MID;
    }

    if(is_lidar_scan_wheel[1])
    {
        // m_pos = M_POSITION_UP;
        motor_pos_data.state = M_POSITION_UP;
    }
    else
    {
        motor_pos_data.state = m_pos;
    }
    motor_pos_pub.publish(motor_pos_data);

    return m_pos;
}
void AreaControl::motor_control(void)
{     
    static uint16_t cmd_timeoutcnt = 0;

    // 确保升降指令执行不超过CMD_TIMEOUT(10s)
    if(c_state == STATE_RUNNING)
    {
        cmd_timeoutcnt++;
        if(cmd_timeoutcnt > CMD_TIMEOUT)
        {
            c_state = STATE_ERROR;
        }
    }
    else
    {
        cmd_timeoutcnt = 0;
    }
    
    // 升 降 停  执行
    if(c_state != STATE_RUNNING || (is_up_limit_active && is_down_limit_active) || (m_pos == M_POSITION_DOWN && m_cmd == M_DOWN) || (m_pos == M_POSITION_UP && m_cmd == M_UP))
    // if((is_up_limit_active && is_down_limit_active) /*|| (m_cmd == M_STOP) */|| ((is_up_limit_active_last != is_up_limit_active) && is_up_limit_active) || ((is_down_limit_active_last != is_down_limit_active) && is_down_limit_active))// && M_LIMIT_DOWN->gpio_read(M_LIMIT_DOWN_PIN) == GPIO_VALUE_HIGH)
    {
        // printf("motor limit up tigger\r\n");
        MOTOR_UP->gpio_write(MOTOR_UP_PIN,GPIO_VALUE_LOW);
        MOTOR_DOWN->gpio_write(MOTOR_DOWN_PIN,GPIO_VALUE_LOW);
        if(m_cmd == M_UP || m_cmd == M_DOWN)
        {
            c_state = STATE_FINISH;
        }
        m_cmd = M_STOP;
        return;
    }
    if(c_state == STATE_RUNNING)
    {
        if(m_cmd == M_DOWN)
        {
            MOTOR_UP->gpio_write(MOTOR_UP_PIN,GPIO_VALUE_LOW);
            MOTOR_DOWN->gpio_write(MOTOR_DOWN_PIN,GPIO_VALUE_HIGH);
            if(delay_flag)
            {
                usleep(60000);      //60ms
                delay_flag = false;
            }
        }
        else if(m_cmd == M_UP && (!is_motor_lock))
        {
            MOTOR_UP->gpio_write(MOTOR_UP_PIN,GPIO_VALUE_HIGH);
            MOTOR_DOWN->gpio_write(MOTOR_DOWN_PIN,GPIO_VALUE_LOW);
            if(delay_flag)
            {
                usleep(60000);      //60ms
                delay_flag = false;
            }
        }
    }
}

void AreaControl::motor_cmd_update(void)
{
    static uint16_t timeoutcnt = 0;
    static uint16_t  screen_tigger_timeout = 0;
    static uint16_t  lidar_ready_timeout = 0;
    
    // 确保升降机在上升状态下不会超过 MOTOR_UP_TIMEOUT  
    if(m_pos == M_POSITION_UP)
    {
        timeoutcnt++;
        if(timeoutcnt > MOTOR_UP_TIMEOUT)
        {
            if(set_motor_cmd(M_DOWN))
            {
                timeoutcnt = 0;
            }
        }
    }
    else
    {
        timeoutcnt = 0;
    }


    // step:1
    // 交接区内没有车
    if((!is_lidar_scan_wheel[0]) && (!is_lidar_scan_wheel[1]))
    {
        if(m_pos != M_POSITION_DOWN)
            set_motor_cmd(M_DOWN);
        car_check_state = C_TRANSFER_EMPTY;
    }
    else
    {
        ;
        // if(car_check_state == C_TRANSFER_EMPTY)
        //     car_check_state = C_NONE;
    }
    

    // step:2
    if(is_screen_tigger[OUTSIDE_SCREEN_ID] && car_check_state == C_TRANSFER_EMPTY)
    {
        car_check_state = C_SCREEN_TIGGER;
        screen_tigger_timeout = 0;
    }
    else
    {
        if(car_check_state == C_SCREEN_TIGGER)
        {
            screen_tigger_timeout++;
            if(screen_tigger_timeout > SCREEN_TIGGER_WAIT_TIMEOUT)
            {
                screen_tigger_timeout = 0;
                car_check_state = C_NONE;
            }
        }
    }
    // step:3
    if(is_lidar_scan_wheel[0])
    {
        if(car_check_state == C_SCREEN_TIGGER)
        {
            lidar_ready_timeout = 0;
            if(set_motor_cmd(M_UP))
                car_check_state = C_SINGLE_LIDAR_READY;
        }
        else
        {
            lidar_ready_timeout++;
            if(lidar_ready_timeout > LIDAR_READY_CLEAR_TIMEOUT)
            {
                lidar_ready_timeout = 0;
                car_check_state = C_SINGLE_LIDAR_READY;
            }
        }
        
    }
    else
    {
        lidar_ready_timeout = 0;
        if(car_check_state == C_SINGLE_LIDAR_READY)
            car_check_state = C_NONE;
    }
    //printf("state:%d\r\n",car_check_state);
}

void AreaControl::update_ez_screen_state(void)
{
    static uint16_t screen_tigger_cnt[2] = {0,0};

    if(GPIO_VALUE_LOW == EZ_SCREEN_OUTSIDE->gpio_read(EZ_SCREEN_OUTSIDE_PIN))         //前门光幕
    {
        // 消抖
        usleep(15000);//15ms
        // 确实触发
        if(GPIO_VALUE_LOW == EZ_SCREEN_OUTSIDE->gpio_read(EZ_SCREEN_OUTSIDE_PIN))
        {
            screen_tigger_cnt[OUTSIDE_SCREEN_ID]++;
            if(screen_tigger_cnt[OUTSIDE_SCREEN_ID]  > SCREEN_TIGGER_TIMEOUT)
            {
                is_screen_tigger[OUTSIDE_SCREEN_ID] = true;
                // if(car_check_state == C_NONE)
                // {
                //     // car_check_state = C_SCREEN_TIGGER;
                //     printf("screen tigger larger than 2s!\r\n");
                // }
            }

            light_curtain_data.id = OUTSIDE_SCREEN_ID;
            light_curtain_data.state = true;
            light_curtain_pub.publish(light_curtain_data);
            // printf("check screen at outside\r\n"); 
        }
        else
        {
            light_curtain_data.id = OUTSIDE_SCREEN_ID;
            light_curtain_data.state = false;
            light_curtain_pub.publish(light_curtain_data);

            screen_tigger_cnt[OUTSIDE_SCREEN_ID] = 0;
            is_screen_tigger[OUTSIDE_SCREEN_ID] = false;
        }
        
    }
    else
    {
        light_curtain_data.id = OUTSIDE_SCREEN_ID;
        light_curtain_data.state = false;
        light_curtain_pub.publish(light_curtain_data);

        screen_tigger_cnt[OUTSIDE_SCREEN_ID] = 0;
        is_screen_tigger[OUTSIDE_SCREEN_ID] = false;
    }

    if(GPIO_VALUE_LOW == EZ_SCREEN_INSIDE->gpio_read(EZ_SCREEN_INSIDE_PIN))         //后门光幕
    {
        usleep(15000);//15ms
        if(GPIO_VALUE_LOW == EZ_SCREEN_INSIDE->gpio_read(EZ_SCREEN_INSIDE_PIN))
        {
            screen_tigger_cnt[INSIDE_SCREEN_ID]++;
            if(screen_tigger_cnt[INSIDE_SCREEN_ID] > SCREEN_TIGGER_TIMEOUT)
            {
                is_screen_tigger[INSIDE_SCREEN_ID] = true;
            }
            light_curtain_data.id = INSIDE_SCREEN_ID;
            light_curtain_data.state = true;
            light_curtain_pub.publish(light_curtain_data);
            // printf("check screen at inside\r\n"); 
        }
        else
        {
            screen_tigger_cnt[INSIDE_SCREEN_ID] = 0;
            is_screen_tigger[INSIDE_SCREEN_ID] = false;
            light_curtain_data.id = INSIDE_SCREEN_ID;
            light_curtain_data.state = false;
            light_curtain_pub.publish(light_curtain_data);
        }
        
    }
    else
    {
        screen_tigger_cnt[INSIDE_SCREEN_ID] = 0;
        is_screen_tigger[INSIDE_SCREEN_ID] = false;
        light_curtain_data.id = INSIDE_SCREEN_ID;
        light_curtain_data.state = false;
        light_curtain_pub.publish(light_curtain_data);
    }
}


void AreaControl::area_cmd_update(void)
{
    ;
}



void AreaControl::display_control(void)
{
    if(d_cmd == D_FORWARD)
    {
        DISPLAY_A1->gpio_write(DISPLAY_A1_PIN,GPIO_VALUE_HIGH);
        DISPLAY_A2->gpio_write(DISPLAY_A2_PIN,GPIO_VALUE_LOW);
        DISPLAY_A3->gpio_write(DISPLAY_A3_PIN,GPIO_VALUE_LOW);
        DISPLAY_A4->gpio_write(DISPLAY_A4_PIN,GPIO_VALUE_LOW);
    }
    else if(d_cmd == D_OK)
    {
        DISPLAY_A1->gpio_write(DISPLAY_A1_PIN,GPIO_VALUE_LOW);
        DISPLAY_A2->gpio_write(DISPLAY_A2_PIN,GPIO_VALUE_HIGH);
        DISPLAY_A3->gpio_write(DISPLAY_A3_PIN,GPIO_VALUE_LOW);
        DISPLAY_A4->gpio_write(DISPLAY_A4_PIN,GPIO_VALUE_LOW);
    }
    else if(d_cmd == D_FORWARD_Q)
    {
        DISPLAY_A1->gpio_write(DISPLAY_A1_PIN,GPIO_VALUE_LOW);
        DISPLAY_A2->gpio_write(DISPLAY_A2_PIN,GPIO_VALUE_LOW);
        DISPLAY_A3->gpio_write(DISPLAY_A3_PIN,GPIO_VALUE_HIGH);
        DISPLAY_A4->gpio_write(DISPLAY_A4_PIN,GPIO_VALUE_LOW);
    }
    else if(d_cmd == D_BACK)
    {
        DISPLAY_A1->gpio_write(DISPLAY_A1_PIN,GPIO_VALUE_LOW);
        DISPLAY_A2->gpio_write(DISPLAY_A2_PIN,GPIO_VALUE_LOW);
        DISPLAY_A3->gpio_write(DISPLAY_A3_PIN,GPIO_VALUE_LOW);
        DISPLAY_A4->gpio_write(DISPLAY_A4_PIN,GPIO_VALUE_HIGH);
    }
}

void AreaControl::run(void)
{
	static uint16_t task_cnt=0;
	task_cnt++;

    gettimeofday(&stamp, NULL);
    now = stamp.tv_sec*1000000 + stamp.tv_usec;
    // printf("now:%ld,a:%ld %ld\r\n",now,wheel_info_update_time,(now - wheel_info_update_time));
    // printf("----now:%ld,a:%ld %ld\r\n",lidar_scan_update_time[0],lidar_scan_update_time[1],(now - lidar_scan_update_time[0]));

    if((now - lidar_scan_update_time[0]) < 600000)
    {
        is_lidar_scan_wheel[0] = true;
    }
    else
    {
        is_lidar_scan_wheel[0] = false;
    }

    if((now - lidar_scan_update_time[1]) < 600000)
    {
        is_lidar_scan_wheel[1] = true;
    }
    else
    {
        is_lidar_scan_wheel[1] = false;
    }
    
    // if(now - wheel_info_update_time >= 600000)
    // {

    //     is_lidar_scan_wheel[0] = false;
    //     is_lidar_scan_wheel[1] = false;
    // }
    //printf("is lidar [%d %d]\r\n",is_lidar_scan_wheel[0],is_lidar_scan_wheel[1]);
    // screen
    update_ez_screen_state();
    
    // motor
    update_motor_position();
    motor_cmd_update();
    motor_control();

    // dispaly
    display_control();
}


/* robot_command main application */
int main(int argc, char **argv)
{
    ros::init(argc, argv, AREA_CONTROL);
    ros::NodeHandle nh("");
    ros::Rate r(AREA_CONTROL_HZ);

    AreaControl area_ctrl;

    // if (!area_control.init())
    //     return EXIT_FAILURE;

	area_ctrl.AreaControl_Init();
    sleep(3);
    while(ros::ok())
    {
        area_ctrl.run();
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
