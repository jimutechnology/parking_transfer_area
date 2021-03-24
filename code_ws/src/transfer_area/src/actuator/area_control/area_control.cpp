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
#include "car_scanner/CarInfo.h"
#include "car_scanner/CarState.h"
#include "../../actuator/gpio/gpio.h"

using namespace std;

#define AREA_CONTROL "area_control"

#define  GPIO_VALUE_HIGH        1
#define  GPIO_VALUE_LOW         0

#define  M_LIMIT_UP_PIN         20
#define  M_LIMIT_DOWN_PIN       26
#define  EZ_SCREEN_F_PIN        19
#define  EZ_SCREEN_B_PIN        5
#define  MOTOR_UP_PIN           15
#define  MOTOR_DOWN_PIN         22
#define  DISPLAY_A1_PIN         24
#define  DISPLAY_A2_PIN         9
#define  DISPLAY_A3_PIN         11
#define  DISPLAY_A4_PIN         7

#define  M_LIMIT_UP_PIN_STR     "20"
#define  M_LIMIT_DOWN_PIN_STR   "26"
#define  EZ_SCREEN_F_PIN_STR    "19"
#define  EZ_SCREEN_B_PIN_STR    "5"
#define  MOTOR_UP_PIN_STR       "15"
#define  MOTOR_DOWN_PIN_STR     "22"
#define  DISPLAY_A1_PIN_STR     "24"
#define  DISPLAY_A2_PIN_STR     "9"
#define  DISPLAY_A3_PIN_STR     "11"
#define  DISPLAY_A4_PIN_STR     "7"

using namespace car_scanner;
using namespace std;

class AreaControl{
public:
	ros::NodeHandle     nh;

	ros::Publisher      error_info_pub;
    ros::Publisher      warning_info_pub;

    ros::Subscriber     car_state_sub;
    ros::Subscriber     car_info_sub;
   
    motor_cmd m_cmd = M_STOP;
    dispaly_cmd d_cmd = D_FORWARD_Q;
    bool ez_screen_f = false;
    bool ez_screen_b = false;
    car_scanner::CarState car_state;
    car_scanner::CarInfo car_info;

    // intput
    Gpio *M_LIMIT_UP = nullptr;
	Gpio *M_LIMIT_DOWN = nullptr;
    Gpio *EZ_SCREEN_F = nullptr;
	Gpio *EZ_SCREEN_B = nullptr;

    // output
    Gpio *MOTOR_UP = nullptr;
	Gpio *MOTOR_DOWN = nullptr;

    Gpio *DISPLAY_A1 = nullptr;     //rad-forward
	Gpio *DISPLAY_A2 = nullptr;     //green-ok
    Gpio *DISPLAY_A3 = nullptr;     //yellow-forward-quiet
	Gpio *DISPLAY_A4 = nullptr;     //back

    

public:
    AreaControl()
    {
		M_LIMIT_UP = new Gpio(M_LIMIT_UP_PIN_STR);
		M_LIMIT_DOWN = new Gpio(M_LIMIT_DOWN_PIN_STR);
        EZ_SCREEN_F = new Gpio(EZ_SCREEN_F_PIN_STR);
        EZ_SCREEN_B = new Gpio(EZ_SCREEN_B_PIN_STR);

        MOTOR_UP = new Gpio(MOTOR_UP_PIN_STR);
        MOTOR_DOWN = new Gpio(MOTOR_DOWN_PIN_STR);

        DISPLAY_A1 = new Gpio(DISPLAY_A1_PIN_STR);
		DISPLAY_A2 = new Gpio(DISPLAY_A2_PIN_STR);
        DISPLAY_A3 = new Gpio(DISPLAY_A3_PIN_STR);
        DISPLAY_A4 = new Gpio(DISPLAY_A4_PIN_STR);
    }

    ~AreaControl() 
	{
        delete M_LIMIT_UP;
        delete M_LIMIT_DOWN;
        delete EZ_SCREEN_F;
        delete EZ_SCREEN_B;
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
    void rx_car_state_data(const car_scanner::CarState msg);

    void area_cmd_update(void);
    void motor_control(void);
    void display_control(void);
    void update_ez_screen_state(void);

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


    car_state_sub = nh.subscribe("car_state",  5, &AreaControl::rx_car_state_data, this);
    car_info_sub = nh.subscribe("car_info",  5, &AreaControl::rx_car_info_data, this);

	return true;
}

void AreaControl::rx_car_info_data(const car_scanner::CarInfo msg)
{
    memcpy(&car_info,&msg,sizeof(car_scanner::CarInfo));
}

void AreaControl::rx_car_state_data(const car_scanner::CarState msg)
{
    memcpy(&car_state,&msg,sizeof(car_scanner::CarState));
    /*
    if(msg.is_car_available)
    {
        d_cmd = D_OK;
        m_cmd = M_UP;
    }
    else
    {
        // m_cmd = M_DOWN;
        ;
    }
    */
    
}

void AreaControl::motor_control(void)
{
    if(M_LIMIT_UP->gpio_read(M_LIMIT_UP_PIN) == GPIO_VALUE_LOW || M_LIMIT_DOWN->gpio_read(M_LIMIT_DOWN_PIN) == GPIO_VALUE_LOW || m_cmd == M_STOP)// && M_LIMIT_DOWN->gpio_read(M_LIMIT_DOWN_PIN) == GPIO_VALUE_HIGH)
    {
        MOTOR_UP->gpio_write(MOTOR_UP_PIN,GPIO_VALUE_LOW);
        MOTOR_DOWN->gpio_write(MOTOR_DOWN_PIN,GPIO_VALUE_LOW);
        m_cmd = M_STOP;
        return;
    }
    if(m_cmd == M_UP)
    {
        MOTOR_UP->gpio_write(MOTOR_UP_PIN,GPIO_VALUE_LOW);
        MOTOR_DOWN->gpio_write(MOTOR_DOWN_PIN,GPIO_VALUE_HIGH);
    }
    else if(m_cmd == M_DOWN)
    {
        MOTOR_UP->gpio_write(MOTOR_UP_PIN,GPIO_VALUE_HIGH);
        MOTOR_DOWN->gpio_write(MOTOR_DOWN_PIN,GPIO_VALUE_LOW);
    }
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


void AreaControl::update_ez_screen_state(void)
{
    if(GPIO_VALUE_LOW == EZ_SCREEN_F->gpio_read(EZ_SCREEN_F_PIN))         //前门光幕
    {
        usleep(15000);//15ms
        if(GPIO_VALUE_LOW == EZ_SCREEN_F->gpio_read(EZ_SCREEN_F_PIN))
        {
            ez_screen_f = true;
            printf("check screen at outside\r\n"); 
        }
        else
        {
            ez_screen_f = false;
        }
        
    }
    else
    {
        ez_screen_f = false;
    }

    if(GPIO_VALUE_LOW == EZ_SCREEN_B->gpio_read(EZ_SCREEN_B_PIN))         //后门光幕
    {
        usleep(15000);//15ms
        if(GPIO_VALUE_LOW == EZ_SCREEN_B->gpio_read(EZ_SCREEN_B_PIN))
        {
            ez_screen_b = true;
            printf("check screen at inside\r\n"); 
        }
        else
        {
            ez_screen_b = false;
        }
        
    }
    else
    {
        ez_screen_b = false;
    }
}

void AreaControl::area_cmd_update(void)
{

}

void AreaControl::run(void)
{
	static uint16_t task_cnt=0;
	task_cnt++;

    update_ez_screen_state();

    area_cmd_update();

    display_control();
    motor_control();
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
