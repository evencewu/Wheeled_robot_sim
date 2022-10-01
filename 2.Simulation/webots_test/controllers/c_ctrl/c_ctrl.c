#include <stdio.h>

#include "type_def.h"

#define TIME_STEP 10

BODY ROBOT;

#include "init.h"
#include "control_set.h"

int main(int argc, char **argv) 
{
  wb_robot_init();
  wb_keyboard_enable(TIME_STEP);
  //
  init_all();
  

  double left_speed = 1.0;
  double right_speed = 1.0;

  //read_pos();
  //read_imu();

  while (wb_robot_step(TIME_STEP) != -1) {
      
      int new_key = wb_keyboard_get_key();
      while (new_key > 0) {
          switch (new_key) {
          case 87://w
              ROBOT.W_WAY = ROBOT.WAY + 1;
              //ROBOT.W_V = 1;
              break;

          case 83://s
              ROBOT.W_WAY = ROBOT.WAY - 1;
              //ROBOT.W_V = -1;
              break;

          case 65://a
              ROBOT.W_WAY = ROBOT.WAY;
              //ROBOT.W_V = 0;
              break;

          case 68://d
              ROBOT.W_WAY = ROBOT.WAY;
              //ROBOT.W_V = 0;
              break;
          }
          new_key = wb_keyboard_get_key();

          if (left_speed > 5)left_speed = 5;
          else if (left_speed < -5)left_speed = -5;
          if (right_speed > 5)right_speed = 5;
          else if (right_speed < -5)right_speed = -5;
      }

      //
      read_pos();
      read_imu();
      //
      calculating_leg(L, ROBOT.wheelset[L].servo_mod[F].pos.position, ROBOT.wheelset[L].servo_mod[B].pos.position);
      calculating_leg(R, ROBOT.wheelset[R].servo_mod[F].pos.position, ROBOT.wheelset[R].servo_mod[B].pos.position);

      calculating_body();
      leg_force_ctrl(L);
      leg_force_ctrl(R);
      //
      lqr_ctrl();

      printf("||--------MOTOR_POS-------||---------MOTOR_W--------||\n");
      printf("|| %+f || %+f || %+f || %+f ||\n", ROBOT.wheelset[L].servo_mod[F].pos.position, ROBOT.wheelset[R].servo_mod[F].pos.position, ROBOT.wheelset[L].servo_mod[F].pos.w, ROBOT.wheelset[R].servo_mod[F].pos.w);
      printf("|| %+f || %+f || %+f || %+f ||\n", ROBOT.wheelset[L].servo_mod[B].pos.position, ROBOT.wheelset[R].servo_mod[B].pos.position, ROBOT.wheelset[L].servo_mod[B].pos.w, ROBOT.wheelset[R].servo_mod[B].pos.w);
      printf("||--------MOTOR_TOQ-------||------------------------||\n");
      printf("|| %+f || %+f ||\n", ROBOT.wheelset[L].servo_mod[F].motor.torque, ROBOT.wheelset[R].servo_mod[F].motor.torque);
      printf("|| %+f || %+f ||\n", ROBOT.wheelset[L].servo_mod[B].motor.torque, ROBOT.wheelset[R].servo_mod[B].motor.torque);
      printf("||----------LEG-XY--------||----------LEG-XY-V------||\n");
      printf("|| %+f || %+f || %+f || %+f ||\n", ROBOT.wheelset[L].endpoint_x, ROBOT.wheelset[R].endpoint_x, ROBOT.wheelset[L].endpoint_Vx, ROBOT.wheelset[R].endpoint_Vx);
      printf("|| %+f || %+f || %+f || %+f ||\n", ROBOT.wheelset[L].endpoint_y, ROBOT.wheelset[R].endpoint_y, ROBOT.wheelset[L].endpoint_Vy, ROBOT.wheelset[R].endpoint_Vy);
      printf("||-------angle-data---------------------------------||\n");
      printf("|| %+f || %+f || %+f              ||\n", ROBOT.angle_data[0], ROBOT.angle_data[1], ROBOT.angle_data[2]);
      printf("|| %+f || %+f || %+f              ||\n", ROBOT.angle_data[3], ROBOT.angle_data[4], ROBOT.angle_data[5]);
      printf("||-------liner_data---------------------------------||\n");
      printf("|| %+f || %+f || %+f              ||\n" , ROBOT.liner_data[0], ROBOT.liner_data[1], ROBOT.liner_data[2]);
      printf("|| %+f || %+f || %+f              ||\n" , ROBOT.liner_data[3], ROBOT.liner_data[4], ROBOT.liner_data[5]);
      printf("||--------odometer----------------------------------||\n");
      printf("THETA: %f \n", ROBOT.THETA);
      printf("DTHETA:%f \n", ROBOT.D_THETA); 
      printf("||-------LQR-MEMBER---------------------------------||\n");
      printf("T:%f \n", ROBOT.T);
      printf("------LEG-jacobian------\n");
      printf("|| [%+f  %+f] || [%+f  %+f] ||\n", ROBOT.wheelset[L].jacobian[0], ROBOT.wheelset[L].jacobian[1], ROBOT.wheelset[R].jacobian[0], ROBOT.wheelset[R].jacobian[1]);
      printf("|| [%+f  %+f] || [%+f  %+f] ||\n", ROBOT.wheelset[L].jacobian[2], ROBOT.wheelset[L].jacobian[3], ROBOT.wheelset[R].jacobian[2], ROBOT.wheelset[R].jacobian[3]);
      printf("------------------------\n");
      printf("|| %+f || %+f || %+f || %+f ||\n\n", ROBOT.wheelset[L].W_endpoint_Vx, ROBOT.wheelset[B].W_endpoint_Vx, ROBOT.wheelset[L].W_endpoint_Fx, ROBOT.wheelset[B].W_endpoint_Fx);
      printf("|| %+f || %+f || %+f || %+f ||\n\n", ROBOT.wheelset[L].W_endpoint_Vy, ROBOT.wheelset[B].W_endpoint_Vy, ROBOT.wheelset[L].W_endpoint_Fy, ROBOT.wheelset[B].W_endpoint_Fy);
      printf("%+f\n ||%+f\n", ROBOT.L_way, ROBOT.WAY);
      printf("------------------------\n");

      

           
      wb_motor_set_torque(ROBOT.wheelset[R].servo_mod[F].motor.ID, -ROBOT.wheelset[R].servo_mod[F].motor.torque);
      wb_motor_set_torque(ROBOT.wheelset[L].servo_mod[F].motor.ID, -ROBOT.wheelset[L].servo_mod[F].motor.torque);

      wb_motor_set_torque(ROBOT.wheelset[R].servo_mod[B].motor.ID, ROBOT.wheelset[R].servo_mod[B].motor.torque);
      wb_motor_set_torque(ROBOT.wheelset[L].servo_mod[B].motor.ID, ROBOT.wheelset[L].servo_mod[B].motor.torque);
             
      wb_motor_set_torque(ROBOT.wheelset[L].servo_mod[WHEEL].motor.ID, -ROBOT.T);
      wb_motor_set_torque(ROBOT.wheelset[R].servo_mod[WHEEL].motor.ID, ROBOT.T);
  }

  wb_robot_cleanup();

  return 0;
}
