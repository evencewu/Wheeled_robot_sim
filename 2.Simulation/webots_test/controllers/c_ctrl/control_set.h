#ifndef _CONTROL_SET_
#define _CONTROL_SET_

void read_pos() {
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 3; j++) {
            ROBOT.wheelset[i].servo_mod[j].pos.position = wb_position_sensor_get_value(ROBOT.wheelset[i].servo_mod[j].pos.ID);
            ROBOT.wheelset[i].servo_mod[j].pos.position_last = ROBOT.wheelset[i].servo_mod[j].pos.position;
        }
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
}
#endif