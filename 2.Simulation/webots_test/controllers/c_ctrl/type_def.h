#ifndef _TYPE_DEF_
#define _TYPE_DEF_

#include <webots\accelerometer.h>
#include <webots\brake.h>
#include <webots\camera_recognition_object.h>
#include <webots\camera.h>
#include <webots\compass.h>
#include <webots\connector.h>
#include <webots\console.h>
#include <webots\device.h>
#include <webots\display.h>
#include <webots\distance_sensor.h>
#include <webots\emitter.h>
#include <webots\gps.h>
#include <webots\gyro.h>
#include <webots\inertial_unit.h>
#include <webots\joystick.h>
#include <webots\keyboard.h>
#include <webots\led.h>
#include <webots\lidar_point.h>
#include <webots\lidar.h>
#include <webots\light_sensor.h>
#include <webots\microphone.h>
#include <webots\motor.h>
#include <webots\mouse_state.h>
#include <webots\mouse.h>
#include <webots\nodes.h>
#include <webots\position_sensor.h>
#include <webots\pen.h>
#include <webots\radar_target.h>
#include <webots\radar.h>
#include <webots\radio.h>
#include <webots\range_finder.h>
#include <webots\receiver.h>
#include <webots\remote_control.h>
#include <webots\robot.h>
#include <webots\skin.h>
#include <webots\speaker.h>
#include <webots\supervisor.h>
#include <webots\touch_sensor.h>
#include <webots\types.h>

#define R       0
#define L       1

#define B       0
#define F       1
#define WHEEL   2

//电机结构体
typedef struct Motor{
    WbDeviceTag ID;
    const char* name;

    double MAX_TORQUE;
    double torque;
    double speed;//速度控制参数

}MOTOR;

//编码器结构体
typedef struct Pos {
    WbDeviceTag ID;
    const char* name;

    double position;//rad
    double position_last;

}POS;

//伺服模组结构体（包含电机和传感器）
typedef struct Servo_module{
    MOTOR motor;
    POS pos;
}SERVO_MOD;

//轮组结构体（轮腿）
typedef struct Wheelset {
    SERVO_MOD servo_mod[3];

    double endpoint_x;
    double endpoint_y;

}WHEELSET;

//
typedef struct Body {
    WHEELSET wheelset[2];//两条腿

    //机器物理参数定义
    double radius_wheel; //m

    double length_d;  //0.1m
    double length_l1; //0.2m
    double length_l2; //0.3m

    double mass_body;  //20kg
    double mass_l1;    //0.125kg
    double mass_l2;    //0.175kg
    double mass_wheel; //0.4kg


    //机器状态参数定义

    double height_of_center; //m

}BODY;
//init.h
void init_all();
void motor_init();
void position_sensor_init();
//control.h
void read_pos();
void calculating_leg(int side, double arpha, double beta);

#endif