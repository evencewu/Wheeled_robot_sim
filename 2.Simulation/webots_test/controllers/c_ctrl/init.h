#ifndef _INIT_
#define _INIT_

void init_all() {
    //机器人参数初始化
    ROBOT.radius_wheel = 0.09;
    ROBOT.length_d = 0.1;
    ROBOT.length_l1 = 0.2;
    ROBOT.length_l2 = 0.3;

    ROBOT.mass_body = 20;
    ROBOT.mass_l1 = 0.125;
    ROBOT.mass_l2 = 0.175;
    ROBOT.mass_wheel = 0.4;

    motor_init();
    position_sensor_init();
}

void motor_init() {
    ROBOT.wheelset[R].servo_mod[B].motor.name = "RB_1_MOTOR";
    ROBOT.wheelset[R].servo_mod[F].motor.name = "RF_1_MOTOR";
    ROBOT.wheelset[R].servo_mod[WHEEL].motor.name = "R_MOTOR";

    ROBOT.wheelset[L].servo_mod[B].motor.name = "LB_1_MOTOR";
    ROBOT.wheelset[L].servo_mod[F].motor.name = "LF_1_MOTOR";
    ROBOT.wheelset[L].servo_mod[WHEEL].motor.name = "L_MOTOR";

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            //获取电机ID
            ROBOT.wheelset[i].servo_mod[j].motor.ID = wb_robot_get_device(ROBOT.wheelset[i].servo_mod[j].motor.name);
            //assert(ROBOT.wheelset[i].servo_mod[j].motor.ID);
            //获取最大扭矩
            ROBOT.wheelset[i].servo_mod[j].motor.MAX_TORQUE = wb_motor_get_max_torque(ROBOT.wheelset[i].servo_mod[j].motor.ID);
            //printf("max_t%f\n",ROBOT.motor[i].MAX_TORQUE);

            wb_motor_set_control_pid(ROBOT.wheelset[i].servo_mod[j].motor.ID,20,0,0);

            //使能扭矩反馈
            int sampling_period;
            sampling_period = TIME_STEP;// wb_motor_get_torque_feedback_sampling_period(ROBOT.motor[i].ID);
            wb_motor_enable_torque_feedback(ROBOT.wheelset[i].servo_mod[j].motor.ID, sampling_period);
            //归零
            ROBOT.wheelset[i].servo_mod[j].motor.torque = 0;
            //wb_motor_set_torque(ROBOT.wheelset[i].servo_mod[j].motor.ID, 0);
            printf("get motor %s succeed: %d\n", ROBOT.wheelset[i].servo_mod[j].motor.name, ROBOT.wheelset[i].servo_mod[j].motor.ID);
        }
    }
}

void position_sensor_init() {
    ROBOT.wheelset[R].servo_mod[B].pos.name = "RB_1_POS";
    ROBOT.wheelset[R].servo_mod[F].pos.name = "RF_1_POS";
    ROBOT.wheelset[R].servo_mod[WHEEL].pos.name = "R_POS";

    ROBOT.wheelset[L].servo_mod[B].pos.name = "LB_1_POS";
    ROBOT.wheelset[L].servo_mod[F].pos.name = "LF_1_POS";
    ROBOT.wheelset[L].servo_mod[WHEEL].pos.name = "L_POS";

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            ROBOT.wheelset[i].servo_mod[j].pos.ID = wb_robot_get_device(ROBOT.wheelset[i].servo_mod[j].pos.name);
            wb_position_sensor_enable(ROBOT.wheelset[i].servo_mod[j].pos.ID, (int)TIME_STEP);

            ROBOT.wheelset[i].servo_mod[j].pos.position = 0;
            ROBOT.wheelset[i].servo_mod[j].pos.position_last = 0;
        }
    }
};

#endif