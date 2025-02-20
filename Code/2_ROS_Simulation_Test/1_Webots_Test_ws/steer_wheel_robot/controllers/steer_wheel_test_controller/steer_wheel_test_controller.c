#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv)
{
  wb_robot_init();
  printf("Start\r\n");
  
  WbDeviceTag go_wheels[4];
  char go_wheel_name[4][20] = {"go_wheel_motor1","go_wheel_motor2","go_wheel_motor3","go_wheel_motor4"};
  for (int i = 0; i < 4; i++)
  {
    go_wheels[i] = wb_robot_get_device(go_wheel_name[i]);
    wb_motor_set_position(go_wheels[i], INFINITY);
  }
  
  WbDeviceTag steer_wheels[4];
  char steer_wheel_name[4][20] = {"steer_wheel_motor1","steer_wheel_motor2","steer_wheel_motor3","steer_wheel_motor4"};
  for (int i = 0; i < 4; i++)
  {
    steer_wheels[i] = wb_robot_get_device(steer_wheel_name[i]);
    wb_motor_set_position(steer_wheels[i], 0.785);
  }
  
  
  while (wb_robot_step(TIME_STEP) != -1)
  {
    wb_motor_set_velocity(go_wheels[0], 0.5);
    wb_motor_set_velocity(go_wheels[1], 0.5);
    wb_motor_set_velocity(go_wheels[2], 0.5);
    wb_motor_set_velocity(go_wheels[3], 0.5);
  }
  wb_robot_cleanup();
  return 0; // EXIT_SUCCESS
}