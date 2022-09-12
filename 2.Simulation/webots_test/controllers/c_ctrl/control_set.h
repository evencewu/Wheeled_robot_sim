#ifndef _CONTROL_SET_
#define _CONTROL_SET_

void read_pos() {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            ROBOT.wheelset[i].servo_mod[j].pos.position_last = ROBOT.wheelset[i].servo_mod[j].pos.position;
            ROBOT.wheelset[i].servo_mod[j].pos.position = wb_position_sensor_get_value(ROBOT.wheelset[i].servo_mod[j].pos.ID);      
        }
    }
    ROBOT.L_V = -((ROBOT.wheelset[L].servo_mod[WHEEL].pos.position_last - ROBOT.wheelset[L].servo_mod[WHEEL].pos.position) * 1000 * 2 * ROBOT.radius_wheel )/ TIME_STEP;
    ROBOT.R_V = ((ROBOT.wheelset[R].servo_mod[WHEEL].pos.position_last - ROBOT.wheelset[R].servo_mod[WHEEL].pos.position) * 1000 * 2 * ROBOT.radius_wheel )/ TIME_STEP;
}

void read_imu() {
 
    ROBOT.angle_data[0] = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 0);
    ROBOT.angle_data[1] = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 2);
    ROBOT.angle_data[2] = *(wb_inertial_unit_get_roll_pitch_yaw(ROBOT.imu.ID) + 1);
  
    for (int i = 0; i < 3; i++) {
        ROBOT.angle_data[3+i] = *(wb_gyro_get_values(ROBOT.gyro.ID) + i);
    }
}

void calculating_leg(int side,double arpha, double beta) {

    arpha = -arpha;

    double d = ROBOT.length_d;
    double l1 = ROBOT.length_l1;
    double l2 = ROBOT.length_l2;

    double p1 = (-(2*pow(l1,2)*cos(arpha + beta) + pow(d,2) + 2*pow(l1,2) - 4*pow(l2,2) + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta))/(2*pow(l1,2)*cos(arpha + beta) + pow(d,2) + 2*pow(l1,2) + 2*d*l1*cos(arpha) + 2*d*l1*cos(beta)));

    //第一组解
    ROBOT.wheelset[side].endpoint_x = (l1 * cos(arpha)) / 2 - (l1 * cos(beta)) / 2 - (l1 * sin(arpha) * pow(p1, 0.5)) / 2 + (l1 * sin(beta) * pow(p1, 0.5)) / 2;
    ROBOT.wheelset[side].endpoint_y = -(l1 * sin(arpha)) / 2 - (l1 * sin(beta)) / 2 - (d * pow(p1, 0.5)) / 2 - (l1 * cos(arpha) * pow(p1, 0.5)) / 2 - (l1 * cos(beta) * pow(p1, 0.5)) / 2;
    
    //第二组解
    //double x = (l1*cos(arpha))/2 - (l1*cos(beta))/2 + (l1*sin(arpha)*pow(p1,(1/2)))/2 - (l1*sin(beta)*pow(p1,(1/2)))/2
    //double y = (                                                   d*pow(p1,(1/2)))/2 - (l1*sin(beta))/2 - (l1*sin(arpha))/2 + (l1*cos(arpha)*pow(p1,(1/2)))/2 + (l1*cos(beta)*pow(p1,(1/2)))/2
    double s1 = 2 * pow(l1,2) * cos(arpha + beta) + pow(d,2) + 2 * pow(l1,2) + 2 * d * l1 * cos(arpha) + 2 * d * l1 * cos(beta);
    double s2 = -(2 * pow(l1,2) * cos(arpha + beta) + pow(d,2) + 2 * pow(l1,2) - 4 * pow(l2,2) + 2 * d * l1 * cos(arpha) + 2 * d * l1 * cos(beta)) / (2 * pow(l1,2) * cos(arpha + beta) + pow(d,2) + 2 * pow(l1,2) + 2 * d * l1 * cos(arpha) + 2 * d * l1 * cos(beta));
    double s3 = l1 * sin(arpha + beta) + d * sin(arpha);
    double s4 = pow(s2,0.5)* pow(s1,2);

    // J         
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
}

void leg_force_strl() {

}

void calculating_body() {
    ROBOT.L_way = ROBOT.wheelset[L].servo_mod[WHEEL].pos.position * 2 * ROBOT.radius_wheel;
    ROBOT.R_way = -ROBOT.wheelset[R].servo_mod[WHEEL].pos.position * 2 * ROBOT.radius_wheel;

    ROBOT.THETA = -ROBOT.angle_data[2];
    ROBOT.D_THETA = -ROBOT.angle_data[5];

    ROBOT.J = 0.435038523 + (ROBOT.mass_body + ROBOT.mass_l1 * 4 + ROBOT.mass_l2 * 4) * pow(ROBOT.height_of_center, 2);
}

void lqr_ctrl()
{
    //float K_1 = -1.0000; float K_2 = -7.1036; float K_3 = -457.9594; float K_4 = -91.4579;
    //float K_1 = -1.1402; float K_2 = -9.0074; float K_3 = -655.6447; float K_4 = -130.6195;
    float K_1 = -2.0000; float K_2 = -12.0574; float K_3 = -666.9166; float K_4 = -133.6345;


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

    ROBOT.T = K_1* ROBOT.L_way + K_2 * ROBOT.L_V + K_3 * ROBOT.THETA + K_4 * ROBOT.D_THETA;

}


#endif