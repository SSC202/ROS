#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();
  printf("Start\r\n");
  int i;
  bool avoid_obstacle_counter = 0;
  WbDeviceTag ds[2];
  char ds_names[2][10] = {"ds_left", "ds_right"};
  for (i = 0; i < 2; i++) {
    ds[i] = wb_robot_get_device(ds_names[i]);
    wb_distance_sensor_enable(ds[i], TIME_STEP);
  }
  WbDeviceTag wheels[4];
  char wheels_names[4][8] = {"wheel1", "wheel2", "wheel3", "wheel4"};
  for (i = 0; i < 4; i++) {
    wheels[i] = wb_robot_get_device(wheels_names[i]);
    wb_motor_set_position(wheels[i], INFINITY);
  }
  while (wb_robot_step(TIME_STEP) != -1) {
    double left_speed = 1.0;
    double right_speed = 1.0;
    if (avoid_obstacle_counter > 0) {
      avoid_obstacle_counter--;
      left_speed = 1.0;
      right_speed = -1.0;
    } else { // read sensors
      double ds_values[2];
      for (i = 0; i < 2; i++)
        ds_values[i] = wb_distance_sensor_get_value(ds[i]);
      if (ds_values[0] < 950.0 || ds_values[1] < 950.0){
        avoid_obstacle_counter = 100;
        }
    }
    
    wb_motor_set_velocity(wheels[0], left_speed);
    wb_motor_set_velocity(wheels[1], left_speed);
    wb_motor_set_velocity(wheels[2], right_speed);
    wb_motor_set_velocity(wheels[3], right_speed);
  }
  wb_robot_cleanup();
  return 0;  // EXIT_SUCCESS
}