#ifndef _CONTROL_SET_
#define _CONTROL_SET_

void read_pos() {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            ROBOT.wheelset[i].servo_mod[j].pos.position_last = ROBOT.wheelset[i].servo_mod[j].pos.position;
            ROBOT.wheelset[i].servo_mod[j].pos.position = wb_position_sensor_get_value(ROBOT.wheelset[i].servo_mod[j].pos.ID);   
            ROBOT.wheelset[i].servo_mod[j].pos.w = (ROBOT.wheelset[i].servo_mod[j].pos.position_last - ROBOT.wheelset[i].servo_mod[j].pos.position) * 1000 / TIME_STEP;
        }
    }

    //轮子的速度
}

void read_imu() {
 
    ROBOT.angle_data[0] = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 0);
    ROBOT.angle_data[1] = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 2);
    ROBOT.angle_data[2] = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 1);
  
    for (int i = 0; i < 3; i++) {
        ROBOT.angle_data[3+i] = *(wb_gyro_get_values(ROBOT.gyro.ID) + i);
    }

    for (int i = 0; i < 3; i++) {
        ROBOT.liner_data[3+i] = *(wb_accelerometer_get_values(ROBOT.acc.ID) + i);
    }
    
}

void read_force() {

}


void calculating_odometer() {
    
}

void calculating_leg(int side,double arpha, double beta) {
    //足端位置解算--------------------------
    arpha = -arpha;

    double d = ROBOT.length_d;
    double l1 = ROBOT.length_l1;
    double l2 = ROBOT.length_l2;

    double p1 = (-(2*pow(l1,2)*cos(arpha + beta) + pow(d,2) + 2*pow(l1,2) - 4*pow(l2,2) + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*pow(l1,2)*cos(arpha + beta) + pow(d,2) + 2*pow(l1,2) + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)));

    ROBOT.wheelset[side].endpoint_x = (l1 * cos(arpha)) / 2 - (l1 * cos(beta)) / 2 - (l1 * sin(arpha) * pow(p1, 0.5)) / 2 + (l1 * sin(beta) * pow(p1, 0.5)) / 2;
    ROBOT.wheelset[side].endpoint_y = -(l1 * sin(arpha)) / 2 - (l1 * sin(beta)) / 2 - (d * pow(p1, 0.5)) / 2 - (l1 * cos(arpha) * pow(p1, 0.5)) / 2 - (l1 * cos(beta) * pow(p1, 0.5)) / 2;
    
    double s1 = 2 * pow(l1,2) * cos(arpha + beta) + pow(d,2) + 2 * pow(l1,2) + 2 * d * l1 * cos(arpha) + 2 * d * l1 * cos(beta);
    double s2 = -(2 * pow(l1,2) * cos(arpha + beta) + pow(d,2) + 2 * pow(l1,2) - 4 * pow(l2,2) + 2 * d * l1 * cos(arpha) + 2 * d * l1 * cos(beta)) / (2 * pow(l1,2) * cos(arpha + beta) + pow(d,2) + 2 * pow(l1,2) + 2 * d * l1 * cos(arpha) + 2 * d * l1 * cos(beta));
    double s3 = l1 * sin(arpha + beta) + d * sin(arpha);
    double s4 = pow(s2,0.5)* pow(s1,2);
    //--------------------------------------

    //雅可比矩阵求解
    // J         W_FX
    // [0  1]
    // [2  3]
    // 
    // J_T         
    // [0  2]
    // [1  3]
    //
    ROBOT.wheelset[side].jacobian[0] = (2 * pow(l1,2) * pow(l2,2) * sin(beta) * s3) / s4 - (l1 * cos(arpha) * pow(s2,0.5)) / 2 - (2 * pow(l1,2) * pow(l2,2) * sin(arpha) * s3) / s4 - (l1 * sin(arpha)) / 2;
    ROBOT.wheelset[side].jacobian[1] = (l1 * sin(beta)) / 2 + (l1 * cos(beta) * pow(s2,0.5)) / 2 - (2 * pow(l1,2) * pow(l2,2) * sin(arpha) * (l1 * sin(arpha + beta) + d * sin(beta))) / s4 + (2 * pow(l1,2) * pow(l2,2) * sin(beta) * (l1 * sin(arpha + beta) + d * sin(beta))) / s4;
    ROBOT.wheelset[side].jacobian[2] = (l1 * sin(arpha) * pow(s2,0.5)) / 2 - (l1 * cos(arpha)) / 2 - (2 * pow(l1,2) * pow(l2,2) * cos(arpha) * s3) / s4 - (2 * pow(l1,2) * pow(l2,2) * cos(beta) * s3) / s4 - (2 * d * l1 * pow(l2,2) * s3) / s4;
    ROBOT.wheelset[side].jacobian[3] = (l1 * sin(beta) * pow(s2,0.5)) / 2 - (l1 * cos(beta)) / 2 - (2 * pow(l1,2) * pow(l2,2) * cos(arpha) * (l1 * sin(arpha + beta) + d * sin(beta))) / s4 - (2 * pow(l1,2) * pow(l2,2) * cos(beta) * (l1 * sin(arpha + beta) + d * sin(beta))) / s4 - (2 * d * l1 * pow(l2,2) * (l1 * sin(arpha + beta) + d * sin(beta))) / s4;

    //速度传递求解
    ROBOT.wheelset[side].endpoint_Vx = ROBOT.wheelset[side].jacobian[0] * ROBOT.wheelset[side].servo_mod[F].pos.w - ROBOT.wheelset[side].jacobian[1] * ROBOT.wheelset[side].servo_mod[B].pos.w;
    ROBOT.wheelset[side].endpoint_Vy = ROBOT.wheelset[side].jacobian[2] * ROBOT.wheelset[side].servo_mod[F].pos.w - ROBOT.wheelset[side].jacobian[3] * ROBOT.wheelset[side].servo_mod[B].pos.w;

    //姿态
    ROBOT.F_arm = ROBOT.wide + tan(ROBOT.angle_data[0]) * ROBOT.wheelset[L].endpoint_y + tan(ROBOT.angle_data[0]) * ROBOT.wheelset[R].endpoint_y;    
    ROBOT.wheelset[L].endpoint_G = 9.8 * (ROBOT.mass_body + ROBOT.mass_l1 * 4 + ROBOT.mass_l2 * 4) * (ROBOT.wide / 2 + tan(-ROBOT.angle_data[0]) * ROBOT.wheelset[B].endpoint_y) / ROBOT.F_arm - 1;
    ROBOT.wheelset[B].endpoint_G = 9.8 * (ROBOT.mass_body + ROBOT.mass_l1 * 4 + ROBOT.mass_l2 * 4) * (ROBOT.wide / 2 + tan(-ROBOT.angle_data[0]) * ROBOT.wheelset[L].endpoint_y) / ROBOT.F_arm - 1;
}

void leg_force_ctrl(int side) {
    ROBOT.wheelset[side].W_endpoint_x = 0;
    ROBOT.wheelset[side].W_endpoint_y = -0.2;

    double P = 10;
    double D = 80;

    ROBOT.wheelset[side].W_endpoint_Vx = P * (ROBOT.wheelset[side].W_endpoint_x - ROBOT.wheelset[side].endpoint_x);
    ROBOT.wheelset[side].W_endpoint_Vy = P * (ROBOT.wheelset[side].W_endpoint_y - ROBOT.wheelset[side].endpoint_y);

    ROBOT.wheelset[side].W_endpoint_Fx = D * (ROBOT.wheelset[side].W_endpoint_Vx - ROBOT.wheelset[side].endpoint_Vx); //0.4kg;
    ROBOT.wheelset[side].W_endpoint_Fy = D * (ROBOT.wheelset[side].W_endpoint_Vy - ROBOT.wheelset[side].endpoint_Vy) + ROBOT.wheelset[side].endpoint_G;

    ROBOT.wheelset[side].servo_mod[F].motor.torque = ROBOT.wheelset[side].jacobian[0] * ROBOT.wheelset[side].W_endpoint_Fx + ROBOT.wheelset[side].jacobian[2] * ROBOT.wheelset[side].W_endpoint_Fy;
    ROBOT.wheelset[side].servo_mod[B].motor.torque = ROBOT.wheelset[side].jacobian[1] * ROBOT.wheelset[side].W_endpoint_Fx + ROBOT.wheelset[side].jacobian[3] * ROBOT.wheelset[side].W_endpoint_Fy;
}

void calculating_body() {
    //路程计算
    ROBOT.L_way = ROBOT.wheelset[L].servo_mod[WHEEL].pos.position * 2 * ROBOT.radius_wheel;
    ROBOT.R_way = -ROBOT.wheelset[R].servo_mod[WHEEL].pos.position * 2 * ROBOT.radius_wheel;

    ROBOT.THETA = -ROBOT.angle_data[2];
    ROBOT.D_THETA = -ROBOT.angle_data[5];

    ROBOT.J = 0.435038523 + (ROBOT.mass_body + ROBOT.mass_l1 * 4 + ROBOT.mass_l2 * 4) * pow(ROBOT.height_of_center, 2);
}

void lqr_ctrl(){
    float K_1 = -1.4142; float K_2 = -10.0315; float K_3 = -659.4296; float K_4 = -131.6302;

    //-14.1421 - 38.9168 - 766.2878 - 160.7060 

    float m = ROBOT.mass_body + ROBOT.mass_l1 * 4 + ROBOT.mass_l2 * 4;
    float M = ROBOT.mass_wheel;
    float l = ROBOT.height_of_center;

    float I = ROBOT.J_wheel;
    float J = ROBOT.J;

    float S_1 = (-pow(m, 2) * pow(l, 2) * g) / (J * (M + m) + M * m * pow(l, 2));
    float S_2 = (m + M) * m * g * l / (J * (M + m) + M * m * pow(l, 2));
    float S_3 = (J + m * pow(l, 2)) / ((J * (M + m) + M * m * pow(l, 2)) * (I / R + 2 * M * R));
    float S_4 = (-m * l) / ((J * (M + m) + M * m * pow(l, 2)) * (I / R + 2 * M * R));
    //以上是无用代码

    ROBOT.T = K_1* (ROBOT.L_way - ROBOT.W_WAY) + K_2 * (ROBOT.V - ROBOT.W_V) + K_3 * (ROBOT.THETA - ROBOT.W_THETA) + K_4 * ROBOT.D_THETA;

}


#endif