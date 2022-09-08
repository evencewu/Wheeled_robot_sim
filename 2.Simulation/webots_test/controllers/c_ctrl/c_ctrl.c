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
  //³õÊ¼»¯
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

      read_pos();

      calculating_leg(L, ROBOT.wheelset[L].servo_mod[F].pos.position, ROBOT.wheelset[L].servo_mod[B].pos.position);
      calculating_leg(R, ROBOT.wheelset[R].servo_mod[F].pos.position, ROBOT.wheelset[R].servo_mod[B].pos.position);

      printf("--------MOTOR_POS-------\n");
      printf("%f || %f\n",ROBOT.wheelset[L].servo_mod[F].pos.position, ROBOT.wheelset[R].servo_mod[F].pos.position);
      printf("%f || %f\n", ROBOT.wheelset[L].servo_mod[B].pos.position, ROBOT.wheelset[R].servo_mod[B].pos.position);
      printf("----------LEG-XY--------\n");
      printf("%f || %f\n", ROBOT.wheelset[L].endpoint_x, ROBOT.wheelset[R].endpoint_x);
      printf("%f || %f\n", ROBOT.wheelset[L].endpoint_y, ROBOT.wheelset[R].endpoint_y);
      printf("------------------------\n");

      wb_motor_set_position(ROBOT.wheelset[R].servo_mod[F].motor.ID, -0.5);
      wb_motor_set_position(ROBOT.wheelset[L].servo_mod[B].motor.ID, 0.5);

      wb_motor_set_position(ROBOT.wheelset[L].servo_mod[F].motor.ID, -0.5);
      wb_motor_set_position(ROBOT.wheelset[R].servo_mod[B].motor.ID, 0.5);
             
      //wb_motor_set_velocity(ROBOT.wheelset[L].servo_mod[WHEEL].motor.ID, left_speed);
          
      //wb_motor_set_velocity(ROBOT.wheelset[R].servo_mod[WHEEL].motor.ID, right_speed);
  }

  wb_robot_cleanup();

  return 0;
}
