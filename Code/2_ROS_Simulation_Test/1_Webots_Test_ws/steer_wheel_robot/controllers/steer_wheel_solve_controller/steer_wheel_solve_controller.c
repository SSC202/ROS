#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <math.h>

#define TIME_STEP 32

#define CHASSIS_R       0.14142 
#define CHASSIS_WHEEL_R 0.04

float vx = 0.6; // x velocity
float vy = 0.3; // y velocity
float wc = 10;

float steer_angle1,steer_angle2,steer_angle3,steer_angle4;
float go_vel1,go_vel2,go_vel3,go_vel4;

void steer_wheel_chassis_solve(float v_x,float v_y,float w_c,float *angle1,float *angle2,float *angle3,float *angle4,
                               float *vel1,float *vel2,float *vel3,float *vel4){
  // Solve for the speed of wheel1
  float __v1x = v_x - wc * CHASSIS_R * 0.7071;
  float __v1y = v_y + wc * CHASSIS_R * 0.7071;
  float __v1  = sqrt(__v1x * __v1x + __v1y * __v1y);
  
  if(fabs(__v1x) < 1e-6)
  {
    *angle1 = 1.5708; 
    if(__v1y >= 0.0f)
    {
      *vel1 = __v1y;
    }
    else
    {
      *vel1 = - __v1y;
    }
  }
  else
  {
    float __angle1 = atan(__v1y/__v1x);
    *angle1 = __angle1;
    if(__v1x > 0.0f)
    {
       *vel1 = __v1;
    }
    else
    {
       *vel1 = - __v1;
    }
  }
  
  // Solve for the speed of wheel2
  float __v2x = v_x + wc * CHASSIS_R * 0.7071;
  float __v2y = v_y + wc * CHASSIS_R * 0.7071;
  float __v2  = sqrt(__v2x * __v2x + __v2y * __v2y);
  
  if(fabs(__v2x) < 1e-6)
  {
    *angle2 = 1.5708; 
    if(__v2y >= 0.0f)
    {
      *vel2 = __v2y;
    }
    else
    {
      *vel2 = - __v2y;
    }
  }
  else
  {
    float __angle2 = atan(__v2y/__v2x);
    *angle2 = __angle2;
    if(__v2x > 0.0f)
    {
       *vel2 = __v2;
    }
    else
    {
       *vel2 = - __v2;
    }
  }
  
  // Solve for the speed of wheel3
  float __v3x = v_x - wc * CHASSIS_R * 0.7071;
  float __v3y = v_y - wc * CHASSIS_R * 0.7071;
  float __v3  = sqrt(__v3x * __v3x + __v3y * __v3y);
  
  if(fabs(__v3x) < 1e-6)
  {
    *angle3 = 1.5708; 
    if(__v3y >= 0.0f)
    {
      *vel3 = __v3y;
    }
    else
    {
      *vel3 = - __v3y;
    }
  }
  else
  {
    float __angle3 = atan(__v3y/__v3x);
    *angle3 = __angle3;
    if(__v3x > 0.0f)
    {
       *vel3 = __v3;
    }
    else
    {
       *vel3 = - __v3;
    }
  }
  
  // Solve for the speed of wheel4
  float __v4x = v_x - wc * CHASSIS_R * 0.7071;
  float __v4y = v_y - wc * CHASSIS_R * 0.7071;
  float __v4  = sqrt(__v4x * __v4x + __v4y * __v4y);
  
  if(fabs(__v4x) < 1e-6)
  {
    *angle4 = 1.5708; 
    if(__v4y >= 0.0f)
    {
      *vel4 = __v4y;
    }
    else
    {
      *vel4 = - __v4y;
    }
  }
  else
  {
    float __angle4 = atan(__v4y/__v4x);
    *angle4 = __angle4;
    if(__v4x > 0.0f)
    {
       *vel4 = __v4;
    }
    else
    {
       *vel4 = - __v4;
    }
  }
}

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
    wb_motor_set_velocity(go_wheels[i], 0);
  }
  
  WbDeviceTag steer_wheels[4];
  char steer_wheel_name[4][20] = {"steer_wheel_motor1","steer_wheel_motor2","steer_wheel_motor3","steer_wheel_motor4"};
  for (int i = 0; i < 4; i++)
  {
    steer_wheels[i] = wb_robot_get_device(steer_wheel_name[i]);
    wb_motor_set_position(steer_wheels[i], 0);
  }
  
  
  while (wb_robot_step(TIME_STEP) != -1)
  {
    steer_wheel_chassis_solve(vx,vy,wc,
                              &steer_angle1,&steer_angle2,&steer_angle3,&steer_angle4,
                              &go_vel1,&go_vel2,&go_vel3,&go_vel4);
    wb_motor_set_position(steer_wheels[0], steer_angle3);
    wb_motor_set_position(steer_wheels[1], steer_angle1);
    wb_motor_set_position(steer_wheels[2], steer_angle4);
    wb_motor_set_position(steer_wheels[3], steer_angle2);
    
    wb_motor_set_velocity(go_wheels[0], go_vel3);
    wb_motor_set_velocity(go_wheels[1], go_vel1);
    wb_motor_set_velocity(go_wheels[2], go_vel4);
    wb_motor_set_velocity(go_wheels[3], go_vel2);
  }
  wb_robot_cleanup();
  return 0; // EXIT_SUCCESS
}
