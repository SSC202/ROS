#include <webots/robot.h>
#include <webots/supervisor.h>
#include <stdio.h>

#define TIME_STEP 32

int main(int argc, char **argv) {
  wb_robot_init();

  WbNodeRef bb8_node = wb_supervisor_node_get_from_def("BB-8");
  WbFieldRef translation_field = wb_supervisor_node_get_field(bb8_node, "translation");
  
  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
  
  wb_supervisor_field_import_mf_node_from_string(children_field, -1, "DEF BALL Ball { translation 0 1 1 }");
  WbNodeRef ball_node = wb_supervisor_node_get_from_def("BALL");
  WbFieldRef color_field = wb_supervisor_node_get_field(ball_node, "color");

  int i = 0;
  while (wb_robot_step(TIME_STEP) != -1) {
    if (i == 0) {
      const double new_value[3] = {2.5, 0, 0};
      wb_supervisor_field_set_sf_vec3f(translation_field, new_value);
    }
    if (i == 10)
      wb_supervisor_node_remove(bb8_node);
    if (i == 20)
      wb_supervisor_field_import_mf_node_from_string(children_field, -1, "Nao { translation 2.5 0 0.334 }");
    
    const double *position = wb_supervisor_node_get_position(ball_node);
    printf("Ball position: %f %f %f\n", position[0], position[1], position[2]);
    
    if (position[2] < 0.2) {
      const double red_color[3] = {1, 0, 0};
      wb_supervisor_field_set_sf_color(color_field, red_color);
    }
    
    i++;
  }

  wb_robot_cleanup();

  return 0;
}