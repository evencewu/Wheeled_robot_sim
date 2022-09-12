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
  //��ʼ��
  init_all();
  

  double left_speed = 1.0;
  double right_speed = 1.0;

  while (wb_robot_step(TIME_STEP) != -1) {
      
      int new_key = wb_keyboard_get_key();
      /*
      while (new_key > 0) {
          printf("%d\n", new_key);

          switch (new_key) {
          case 87://w
              printf("UP pressed\n");
              if (right_speed < 0)left_speed = -right_speed;
              else left_speed = right_speed;
              left_speed += 1.0;
              right_speed += 1.0;
              break;

          case 83://s
              printf("DOWN pressed\n");
              if (right_speed > 0)left_speed = -right_speed;
              else left_speed = right_speed;
              left_speed -= 1.0;
              right_speed -= 1.0;
              break;

          case 65://a
              printf("LEFT pressed\n");
              left_speed -= 0.5;
              right_speed += 0.5;
              break;

          case 68://d
              printf("RIGHT pressed\n");
              left_speed += 0.5;
              right_speed -= 0.5;
              break;
          }
          new_key = wb_keyboard_get_key();

          if (left_speed > 5)left_speed = 5;
          else if (left_speed < -5)left_speed = -5;
          if (right_speed > 5)right_speed = 5;
          else if (right_speed < -5)right_speed = -5;
      }*/

      //��ȡ����������
      read_pos();
      read_imu();
      //����
      calculating_leg(L, ROBOT.wheelset[L].servo_mod[F].pos.position, ROBOT.wheelset[L].servo_mod[B].pos.position);
      calculating_leg(R, ROBOT.wheelset[R].servo_mod[F].pos.position, ROBOT.wheelset[R].servo_mod[B].pos.position);

      calculating_body();
      leg_force_strl();
      //����
      lqr_ctrl();

      printf("--------MOTOR_POS-------\n");
      printf("%f || %f\n",ROBOT.wheelset[L].servo_mod[F].pos.position, ROBOT.wheelset[R].servo_mod[F].pos.position);
      printf("%f || %f\n", ROBOT.wheelset[L].servo_mod[B].pos.position, ROBOT.wheelset[R].servo_mod[B].pos.position);
      printf("----------LEG-XY--------\n");
      printf("%f || %f\n", ROBOT.wheelset[L].endpoint_x, ROBOT.wheelset[R].endpoint_x);
      printf("%f || %f\n", ROBOT.wheelset[L].endpoint_y, ROBOT.wheelset[R].endpoint_y);
      printf("-------angle-data-------\n");
      printf("%f || %f || %f\n", ROBOT.angle_data[0], ROBOT.angle_data[1], ROBOT.angle_data[2]);
      printf("%f || %f || %f\n", ROBOT.angle_data[3], ROBOT.angle_data[4], ROBOT.angle_data[5]);
      printf("%f || %f || %f\n", ROBOT.angle_data[6], ROBOT.angle_data[7], ROBOT.angle_data[8]);
      printf("--------odometer--------\n");
      printf("X:     %f || %f \n", ROBOT.L_way, ROBOT.R_way);
      printf("DX:    %f || %f \n", ROBOT.L_V, ROBOT.R_V);
      printf("THETA: %f \n", ROBOT.THETA);
      printf("DTHETA:%f \n", ROBOT.D_THETA);
      printf("-------LQR-MEMBER-------\n");
      printf("T:%f \n", ROBOT.T);
      printf("-----------LEG----------\n");
      printf("||[%f %f] \n", ROBOT.jacobian[0], ROBOT.jacobian[1]);
      printf("||[%f %f] \n", ROBOT.jacobian[2], ROBOT.jacobian[3]);
      printf("------------------------\n");
      
      
      wb_motor_set_position(ROBOT.wheelset[R].servo_mod[F].motor.ID, -0.5);
      wb_motor_set_position(ROBOT.wheelset[L].servo_mod[B].motor.ID, 0.5);

      wb_motor_set_position(ROBOT.wheelset[L].servo_mod[F].motor.ID, -0.5);
      wb_motor_set_position(ROBOT.wheelset[R].servo_mod[B].motor.ID, 0.5);
             
      wb_motor_set_torque(ROBOT.wheelset[L].servo_mod[WHEEL].motor.ID, -ROBOT.T);
      wb_motor_set_torque(ROBOT.wheelset[R].servo_mod[WHEEL].motor.ID, ROBOT.T);
  }

  wb_robot_cleanup();

  return 0;
}
