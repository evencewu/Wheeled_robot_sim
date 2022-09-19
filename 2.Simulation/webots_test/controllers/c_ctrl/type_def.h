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

#define g   9.8

//电机定义
typedef struct Motor{
    WbDeviceTag ID;
    const char* name;

    double MAX_TORQUE;

    double torque;//输入期望力矩

    double speed;//输入期望速度

}MOTOR;

//编码器定义
typedef struct Pos {
    WbDeviceTag ID;
    const char* name;

    double position;//rad
    double position_last;

    double w;//rad/s

}POS;

//伺服模组定义
typedef struct Servo_module{
    MOTOR motor;
    POS pos;
}SERVO_MOD;

//轮组定义
typedef struct Wheelset {
    SERVO_MOD servo_mod[3];

    double endpoint_x;
    double endpoint_y;

    double endpoint_Vx;
    double endpoint_Vy;

    double W_endpoint_Fx;
    double W_endpoint_Fy;

    double W_endpoint_x;
    double W_endpoint_y;

    double W_endpoint_Vx;
    double W_endpoint_Vy;

    double endpoint_G;//重力补偿

    double jacobian[4];

}WHEELSET;

//IMU定义
typedef struct Imu {
    WbDeviceTag ID;
    const char* name;
}IMU;

//IMU_D定义
typedef struct Gyro {
    WbDeviceTag ID;
    const char* name;
}GYRO;

//ACC定义
typedef struct Acc {
    WbDeviceTag ID;
    const char* name;
}ACC;

//机器人定义
typedef struct Body {
    WHEELSET wheelset[2];//左右轮组

    IMU imu;
    GYRO gyro;
    ACC acc;

    double angle_data[9];
    double liner_data[9];

    //机体参数
    double radius_wheel; //m
    double wide;

    double length_d;  //0.1m
    double length_l1; //0.2m
    double length_l2; //0.3m

    double mass_body;  //20kg
    double mass_l1;    //0.125kg
    double mass_l2;    //0.175kg
    double mass_wheel; //0.4kg

    double J;  //
    double J_wheel;  //


    //状态参数

    double height_of_center; //m
    

    //LQR
    double L_way;  //
    double L_V;  //
    double R_way; //
    double R_V;  //

    double THETA;  //
    double D_THETA;

    double T;

    //ctrl
    double W_V;//期望速度
    double V;
    double WAY;//路程

    double F_arm;

}BODY;
//init.h
void init_all();
void motor_init();
void imu_init();
void position_sensor_init();
//control.h
void read_pos();
void read_imu();
void read_force();
void calculating_odometer();
void calculating_leg(int side, double arpha, double beta);
void leg_force_ctrl(int side);
void lqr_ctrl();
void calculating_body();


#endif