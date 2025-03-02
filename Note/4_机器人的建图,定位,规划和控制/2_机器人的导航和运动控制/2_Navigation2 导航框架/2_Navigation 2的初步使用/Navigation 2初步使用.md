# ROS Navigation2 初步使用

## 1. Nav2 基本使用

1. 创建功能包

   ```shell
   $ ros2 pkg create fishbot_navigation2 --dependencies nav2_bringup
   ```

2. 添加地图（手动建立或者通过SLAM建立）

   ```shell
   $ cd src/fishbot_navigation2
   $ mkdir launch config map param rviz
   ```

   `map` 文件夹下添加建立的地图文件。

3. 添加配置文件

   ```shell
   $ cd src/fishbot_navigation2/param/
   $ touch fishbot_nav2.yaml
   ```

   `fishbot_nav2.yaml` 配置如下：

   ```yaml
   amcl:
     ros__parameters:
       use_sim_time: True
       alpha1: 0.2
       alpha2: 0.2
       alpha3: 0.2
       alpha4: 0.2
       alpha5: 0.2
       base_frame_id: "base_link"
       beam_skip_distance: 0.5
       beam_skip_error_threshold: 0.9
       beam_skip_threshold: 0.3
       do_beamskip: false
       global_frame_id: "map"
       lambda_short: 0.1
       laser_likelihood_max_dist: 2.0
       laser_max_range: 100.0
       laser_min_range: -1.0
       laser_model_type: "likelihood_field"
       max_beams: 60
       max_particles: 2000
       min_particles: 500
       odom_frame_id: "odom"
       pf_err: 0.05
       pf_z: 0.99
       recovery_alpha_fast: 0.0
       recovery_alpha_slow: 0.0
       resample_interval: 1
       robot_model_type: "nav2_amcl::DifferentialMotionModel"
       save_pose_rate: 0.5
       sigma_hit: 0.2
       tf_broadcast: true
       transform_tolerance: 10.0
       update_min_a: 0.2
       update_min_d: 0.25
       z_hit: 0.5
       z_max: 0.05
       z_rand: 0.5
       z_short: 0.05
       scan_topic: scan
   
   amcl_map_client:
     ros__parameters:
       use_sim_time: True
   
   amcl_rclcpp_node:
     ros__parameters:
       use_sim_time: True
   
   bt_navigator:
     ros__parameters:
       use_sim_time: True
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       # 'default_nav_through_poses_bt_xml' and 'default_nav_to_pose_bt_xml' are use defaults:
       # nav2_bt_navigator/navigate_to_pose_w_replanning_and_recovery.xml
       # nav2_bt_navigator/navigate_through_poses_w_replanning_and_recovery.xml
       # They can be set here or via a RewrittenYaml remap from a parent launch file to Nav2.
       plugin_lib_names:
       - nav2_compute_path_to_pose_action_bt_node
       - nav2_compute_path_through_poses_action_bt_node
       - nav2_smooth_path_action_bt_node
       - nav2_follow_path_action_bt_node
       - nav2_spin_action_bt_node
       - nav2_wait_action_bt_node
       - nav2_back_up_action_bt_node
       - nav2_drive_on_heading_bt_node
       - nav2_clear_costmap_service_bt_node
       - nav2_is_stuck_condition_bt_node
       - nav2_goal_reached_condition_bt_node
       - nav2_goal_updated_condition_bt_node
       - nav2_globally_updated_goal_condition_bt_node
       - nav2_is_path_valid_condition_bt_node
       - nav2_initial_pose_received_condition_bt_node
       - nav2_reinitialize_global_localization_service_bt_node
       - nav2_rate_controller_bt_node
       - nav2_distance_controller_bt_node
       - nav2_speed_controller_bt_node
       - nav2_truncate_path_action_bt_node
       - nav2_truncate_path_local_action_bt_node
       - nav2_goal_updater_node_bt_node
       - nav2_recovery_node_bt_node
       - nav2_pipeline_sequence_bt_node
       - nav2_round_robin_node_bt_node
       - nav2_transform_available_condition_bt_node
       - nav2_time_expired_condition_bt_node
       - nav2_path_expiring_timer_condition
       - nav2_distance_traveled_condition_bt_node
       - nav2_single_trigger_bt_node
       - nav2_is_battery_low_condition_bt_node
       - nav2_navigate_through_poses_action_bt_node
       - nav2_navigate_to_pose_action_bt_node
       - nav2_remove_passed_goals_action_bt_node
       - nav2_planner_selector_bt_node
       - nav2_controller_selector_bt_node
       - nav2_goal_checker_selector_bt_node
       - nav2_controller_cancel_bt_node
       - nav2_path_longer_on_approach_bt_node
       - nav2_wait_cancel_bt_node
       - nav2_spin_cancel_bt_node
       - nav2_back_up_cancel_bt_node
       - nav2_drive_on_heading_cancel_bt_node
   
   bt_navigator_rclcpp_node:
     ros__parameters:
       use_sim_time: True
   
   controller_server:
     ros__parameters:
       use_sim_time: True
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.5
       min_theta_velocity_threshold: 0.001
       failure_tolerance: 0.3
       progress_checker_plugin: "progress_checker"
       goal_checker_plugins: ["general_goal_checker"] # "precise_goal_checker"
       controller_plugins: ["FollowPath"]
   
       # Progress checker parameters
       progress_checker:
         plugin: "nav2_controller::SimpleProgressChecker"
         required_movement_radius: 0.5
         movement_time_allowance: 10.0
       # Goal checker parameters
       #precise_goal_checker:
       #  plugin: "nav2_controller::SimpleGoalChecker"
       #  xy_goal_tolerance: 0.25
       #  yaw_goal_tolerance: 0.25
       #  stateful: True
       general_goal_checker:
         stateful: True
         plugin: "nav2_controller::SimpleGoalChecker"
         xy_goal_tolerance: 0.25
         yaw_goal_tolerance: 0.25
       # DWB parameters
       FollowPath:
         plugin: "dwb_core::DWBLocalPlanner"
         debug_trajectory_details: True
         min_vel_x: 0.0
         min_vel_y: 0.0
         max_vel_x: 0.26
         max_vel_y: 0.0
         max_vel_theta: 1.0
         min_speed_xy: 0.0
         max_speed_xy: 0.26
         min_speed_theta: 0.0
         # Add high threshold velocity for turtlebot 3 issue.
         # https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/75
         acc_lim_x: 2.5
         acc_lim_y: 0.0
         acc_lim_theta: 3.2
         decel_lim_x: -2.5
         decel_lim_y: 0.0
         decel_lim_theta: -3.2
         vx_samples: 20
         vy_samples: 5
         vtheta_samples: 20
         sim_time: 1.7
         linear_granularity: 0.05
         angular_granularity: 0.025
         transform_tolerance: 0.2
         xy_goal_tolerance: 0.25
         trans_stopped_velocity: 0.25
         short_circuit_trajectory_evaluation: True
         stateful: True
         critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]
         BaseObstacle.scale: 0.02
         PathAlign.scale: 32.0
         PathAlign.forward_point_distance: 0.1
         GoalAlign.scale: 24.0
         GoalAlign.forward_point_distance: 0.1
         PathDist.scale: 32.0
         GoalDist.scale: 24.0
         RotateToGoal.scale: 32.0
         RotateToGoal.slowing_factor: 5.0
         RotateToGoal.lookahead_time: -1.0
   
   controller_server_rclcpp_node:
     ros__parameters:
       use_sim_time: True
   
   local_costmap:
     local_costmap:
       ros__parameters:
         update_frequency: 5.0
         publish_frequency: 2.0
         global_frame: odom
         robot_base_frame: base_link
         use_sim_time: True
         rolling_window: true
         width: 3
         height: 3
         resolution: 0.05
         robot_radius: 0.22
         plugins: ["voxel_layer", "inflation_layer"]
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55
         voxel_layer:
           plugin: "nav2_costmap_2d::VoxelLayer"
           enabled: True
           publish_voxel_map: True
           origin_z: 0.0
           z_resolution: 0.05
           z_voxels: 16
           max_obstacle_height: 2.0
           mark_threshold: 0
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 3.0
             raytrace_min_range: 0.0
             obstacle_max_range: 2.5
             obstacle_min_range: 0.0
         static_layer:
           map_subscribe_transient_local: True
         always_send_full_costmap: True
     local_costmap_client:
       ros__parameters:
         use_sim_time: True
     local_costmap_rclcpp_node:
       ros__parameters:
         use_sim_time: True
   
   global_costmap:
     global_costmap:
       ros__parameters:
         update_frequency: 1.0
         publish_frequency: 1.0
         global_frame: map
         robot_base_frame: base_link
         use_sim_time: True
         robot_radius: 0.22
         resolution: 0.05
         track_unknown_space: true
         plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
         obstacle_layer:
           plugin: "nav2_costmap_2d::ObstacleLayer"
           enabled: True
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 3.0
             raytrace_min_range: 0.0
             obstacle_max_range: 2.5
             obstacle_min_range: 0.0
         static_layer:
           plugin: "nav2_costmap_2d::StaticLayer"
           map_subscribe_transient_local: True
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.55
         always_send_full_costmap: True
     global_costmap_client:
       ros__parameters:
         use_sim_time: True
     global_costmap_rclcpp_node:
       ros__parameters:
         use_sim_time: True
   
   map_server:
     ros__parameters:
       use_sim_time: True
       yaml_filename: "turtlebot3_world.yaml"
   
   map_saver:
     ros__parameters:
       use_sim_time: True
       save_map_timeout: 5.0
       free_thresh_default: 0.25
       occupied_thresh_default: 0.65
       map_subscribe_transient_local: True
   
   planner_server:
     ros__parameters:
       expected_planner_frequency: 20.0
       use_sim_time: True
       planner_plugins: ["GridBased"]
       GridBased:
         plugin: "nav2_navfn_planner/NavfnPlanner"
         tolerance: 0.5
         use_astar: false
         allow_unknown: true
   
   planner_server_rclcpp_node:
     ros__parameters:
       use_sim_time: True
   
   smoother_server:
     ros__parameters:
       use_sim_time: True
       smoother_plugins: ["simple_smoother"]
       simple_smoother:
         plugin: "nav2_smoother::SimpleSmoother"
         tolerance: 1.0e-10
         max_its: 1000
         do_refinement: True
   
   behavior_server:
     ros__parameters:
       costmap_topic: local_costmap/costmap_raw
       footprint_topic: local_costmap/published_footprint
       cycle_frequency: 10.0
       behavior_plugins: ["spin", "backup", "drive_on_heading", "wait"]
       spin:
         plugin: "nav2_behaviors/Spin"
       backup:
         plugin: "nav2_behaviors/BackUp"
       drive_on_heading:
         plugin: "nav2_behaviors/DriveOnHeading"
       wait:
         plugin: "nav2_behaviors/Wait"
       global_frame: odom
       robot_base_frame: base_link
       transform_tolerance: 0.1
       use_sim_time: true
       simulate_ahead_time: 2.0
       max_rotational_vel: 1.0
       min_rotational_vel: 0.4
       rotational_acc_lim: 3.2
   
   robot_state_publisher:
     ros__parameters:
       use_sim_time: True
   
   waypoint_follower:
     ros__parameters:
       loop_rate: 20
       stop_on_failure: false
       waypoint_task_executor_plugin: "wait_at_waypoint"
       wait_at_waypoint:
         plugin: "nav2_waypoint_follower::WaitAtWaypoint"
         enabled: True
         waypoint_pause_duration: 200
   ```

4. 编写 `launch` 文件启动功能包

   ```python
   import os
   from ament_index_python.packages import get_package_share_directory
   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration
   from launch_ros.actions import Node
   
   
   def generate_launch_description():
       fishbot_navigation2_dir = get_package_share_directory('fishbot_navigation2')
       nav2_bringup_dir = get_package_share_directory('nav2_bringup')
       
       
       
       # use_sim_time 设置
       use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
       # map 文件设置
       map_yaml_path = LaunchConfiguration('map',default=os.path.join(fishbot_navigation2_dir,'maps','fishbot_map.yaml'))
       # nav2 参数设置
       nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(fishbot_navigation2_dir,'param','fishbot_nav2.yaml'))
       # rviz 参数设置
       rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')
   
       # 启动 nav2 
       nav2_bringup_launch = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
               launch_arguments={
                   'map': map_yaml_path,
                   'use_sim_time': use_sim_time,
                   'params_file': nav2_param_path}.items(),
           )
       # 启动 rviz2
       rviz_node =  Node(
               package='rviz2',
               executable='rviz2',
               name='rviz2',
               arguments=['-d', rviz_config_dir],
               parameters=[{'use_sim_time': use_sim_time}],
               output='screen')
       
       return LaunchDescription([nav2_bringup_launch,rviz_node])
   ```

5. 编写 `CMakeLists.txt`

   ```cmake
   install(
     DIRECTORY launch param maps
     DESTINATION share/${PROJECT_NAME}
   )
   ```

6. `package.xml` 添加依赖

   ```xml
   <exec_depend>nav2_bringup</exec_depend>
   ```

7. 导航测试

   启动仿真环境后启动导航。一开始需要为机器人设定差不多的初始位置，然后可以进行单点和路点导航。

## 2. 配置文件

[配置指南 — Navigation 2 1.0.0 文档](https://fishros.org/doc/nav2/configuration/index.html)

### AMCL 

- `alpha1`：来自旋转的里程计旋转估计的预期过程噪声。默认值0.2。
- `alpha2`：来自平移的里程计旋转估计的预期过程噪声。默认值0.2。
- `alpha3`：来自平移的里程计平移估计的预期过程噪声。默认值0.2。
- `alpha4`：来自旋转的里程计平移估计的预期过程噪声。默认值0.2。
- `alpha5`：仅适用于Omni模型：平移噪声。默认值0.2。
- `base_frame_id`：机器人基准坐标系。默认值`"base_footprint"`。
- `beam_skip_distance`：在似然场模型中忽略大多数粒子不同意的射线。要考虑跳过的最大距离（米）。默认值0.5。
- `beam_skip_error_threshold`：由于收敛性不佳未能将地图匹配到强制完全更新的射线的百分比。默认值0.9。
- `beam_skip_threshold`：需要跳过的射线的百分比。默认值0.3。
- `do_beamskip`：是否在似然场模型中进行射线跳过。默认值 False。
- `global_frame_id`：由定位系统发布的坐标框架的名称。默认值`"map"`。
- `lambda_short`：模型中 `z_short` 部分的指数衰减参数。默认值0.1。
- `laser_likelihood_max_dist`：地图上进行障碍物膨胀的最大距离，用于 likelihood_field 模型。默认值2.0。
- `laser_max_range` ：要考虑的最大扫描范围，-1.0将导致使用激光报告的最大范围。默认值100.0。
- `laser_min_range`：要考虑的最小扫描范围，-1.0 表示使用激光报告的最小范围。
- `laser_model_type`：激光雷达要使用的模型，可以是`beam`、`likelihood_field`或`likelihood_field_prob`。与`likelihood_field`相同，但如果启用，则包含`beamskip`功能。默认值`"likelihood_field"`。
- `set_initial_pose`：使AMCL从`initial_pose*`参数设置初始姿态，而不是等待`initial_pose`消息。默认值 False。
- `initial_pose`：机器人基座标系在全局坐标系中的初始姿态的X、Y、Z和偏航角坐标（以米和弧度表示）。默认值 `{x: 0.0, y: 0.0, z: 0.0, yaw: 0.0}`。
- `max_beams`：更新滤波器时每个扫描中要使用的均匀间隔的激光束数量。默认值 60。
- `max_particles`：粒子的最大允许数量。默认值 2000。
- `min_particles`：粒子数的最小允许值。默认值 500。
- `odom_frame_id`：用于里程计的帧。默认值 `"odom"`。
- `pf_err`：粒子滤波器种群错误。默认值 0.05。
- `pf_z`：粒子滤波器种群密度。默认值 0.99。
- `recovery_alpha_fast`：快速平均权重滤波器的指数衰减率，用于决定何时通过添加随机姿势进行恢复。一个好的值可能是 0.1。默认值 0.0。
- `recovery_alpha_slow`：慢速平均权重滤波器的指数衰减率，用于决定何时通过添加随机姿势进行恢复。一个好的值可能是 0.001。默认值 0.0。
- `resample_interval`：重新取样前所需的滤波器更新次数。默认值 1。
- `robot_model_type`：插件类的完全限定类型。选项为`nav2_amcl::DifferentialMotionModel`和`nav2_amcl::OmniMotionModel`。用户还可以提供自定义的运动模型插件类型。（Nav2 本身提供全向运动模型和差速运动模型）
- `save_pose_rate`：以每秒存储上次估计的姿态和协方差到参数服务器的最大速率（变量为`~initial_pose_*`和`~initial_cov_*`）。此保存的姿态将在后续运行中用于初始化滤波器（-1.0为禁用）。默认值 0.5。
- `sigma_hit`：用于模型中`z_hit`部分的高斯模型的标准偏差。
- `tf_broadcast`：将其设置为false可防止 AMCL 发布全局坐标系和里程计坐标系之间的变换。
- `transform_tolerance`：发布变换时用于后期日期的时间，以表明该变换在未来是有效的。
- `update_min_a`：在执行滤波器更新之前需要的旋转运动。默认值 0.2。
- `update_min_d`：在执行滤波器更新之前需要的平移运动。默认值 0.25。
- `z_hit`：模型中z_hit部分的混合权重，所有使用的z权重之和必须为1。Beam使用全部4个，似然模型使用z_hit和z_rand。默认值 0.5。
- `z_max`：模型中z_max部分的混合权重，所有使用的z权重之和必须为1。Beam使用全部4个权重，似然模型使用z_hit和z_rand。默认值 0.05。
- `z_rand`：模型中z_rand部分的混合权重，所有使用的z权重之和必须为1。Beam使用全部4个权重，似然模型使用z_hit和z_rand。默认值 0.5。
- `z_short`：z_short部分的混合权重，所有使用的z权重之和必须为1。Beam使用全部4个，似然模型使用z_hit和z_rand。默认值 0.005。
- `always_reset_initial_pose`：要求在重置时，通过话题或`initial_pose*`参数（设置参数`set_initial_pose`为`true`）向AMCL提供初始姿态。否则，默认情况下，AMCL将使用上次已知的姿态进行初始化。
- `scan_topic`：要订阅的激光扫描话题。默认值 `scan`。
- `map_topic`：要订阅的地图话题。默认值 `map`。
- `first_map_only`：允许AMCL在map_topic上接受多个地图。这在使用`map_server`中的`LoadMap`服务时特别有用。

示例：

```yaml
amcl:
  ros__parameters:
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_tolerance: 1.0
    update_min_a: 0.2
    update_min_d: 0.25
    z_hit: 0.5
    z_max: 0.05
    z_rand: 0.5
    z_short: 0.05
    scan_topic: scan
    map_topic: map
    set_initial_pose: false
    always_reset_initial_pose: false
    first_map_only: false
    initial_pose:
      x: 0.0
      y: 0.0
      z: 0.0
      yaw: 0.0
```

### bt_navigator

- `navigators`：为实现`nav2_core::BehaviorTreeNavigator`接口的导航器类型添加插件。它们通过实现具有自定义接口定义的自定义动作服务器，并使用该数据来填充和处理行为树导航请求。插件类在相同的命名空间下定义。默认值对应于`NavigateToPoseNavigator`和`NavigateThroughPosesNavigator`。
- `default_nav_to_pose_bt_xml`：用于`NavigateToPose`(单点导航)的默认行为树XML描述文件路径。
- `default_nav_through_poses_bt_xml`：用于`NavigateThroughPoses`(路点导航)的默认行为树XML描述文件路径。
- `plugin_lib_names`：行为树节点共享库的列表。
- `bt_loop_duration`：BT执行的每次迭代的持续时间（以毫秒为单位）。
- `default_server_timeout`：BT操作节点等待动作服务器确认的默认超时时间（毫秒）。如果提供了输入端口`server_timeout`，则此值将被覆盖为BT节点的超时时间。
- `transform_tolerance`：TF变换容差。
- `global_frame`：全局坐标系
- `robot_base_frame`：机器人基准坐标系
- `odom_topic`：里程计话题
- `goal_blackboard_id`：用于向``NavigateToPose``行为树提供目标的黑板变量。应与BT XML文件的端口匹配。
- `path_blackboard_id`：Blackboard变量，用于从行为树获取``NavigateThroughPoses``反馈的路径。应与BT XML文件的端口名称匹配。
- `goals_blackboard_id`：用于向`NavigateThroughPoses`行为树提供目标的黑板变量。应与BT XML文件的端口匹配。
- `use_sim_time`：使用仿真提供的时间。
- `error_code_names`：要进行比较的错误代码列表。

示例：

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: true
    global_frame: map
    robot_base_frame: base_link
    transform_tolerance: 0.1
    default_nav_to_pose_bt_xml: replace/with/path/to/bt.xml
    default_nav_through_poses_bt_xml: replace/with/path/to/bt.xml
    goal_blackboard_id: goal
    goals_blackboard_id: goals
    path_blackboard_id: path
    navigators: ['navigate_to_pose', 'navigate_through_poses']
    navigate_to_pose:
      plugin: "nav2_bt_navigator/NavigateToPoseNavigator"
    navigate_through_poses:
      plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"
    plugin_lib_names:
      - nav2_compute_path_to_pose_action_bt_node
      - nav2_follow_path_action_bt_node
      - nav2_back_up_action_bt_node
      - nav2_spin_action_bt_node
      - nav2_wait_action_bt_node
      - nav2_clear_costmap_service_bt_node
      - nav2_is_stuck_condition_bt_node
      - nav2_goal_reached_condition_bt_node
      - nav2_initial_pose_received_condition_bt_node
      - nav2_goal_updated_condition_bt_node
      - nav2_reinitialize_global_localization_service_bt_node
      - nav2_rate_controller_bt_node
      - nav2_distance_controller_bt_node
      - nav2_speed_controller_bt_node
      - nav2_recovery_node_bt_node
      - nav2_pipeline_sequence_bt_node
      - nav2_round_robin_node_bt_node
      - nav2_transform_available_condition_bt_node
      - nav2_time_expired_condition_bt_node
      - nav2_distance_traveled_condition_bt_node
      - nav2_single_trigger_bt_node
    error_code_names:
      - compute_path_error_code
      - follow_path_error_code
      # - smoother_error_code, navigate_to_pose_error_code, navigate_through_poses_error_code, etc
```

### controller_server

- `controller_frequency`：控制器运行的频率（Hz）。
- `controller_plugins`：用于处理请求和参数的控制器插件的映射名称列表。默认插件：`dwb_core::DWBLocalPlanner`（DWB 控制器）
- `progress_checker_plugins`：检查机器人进度的进度检查插件的映射名称。默认插件：`nav2_controller::SimpleProgressChecker`
- `goal_checker_plugins`：检查目标是否已达到的目标检查插件的映射名称。默认插件：`nav2_controller::SimpleGoalChecker`
- `min_x_velocity_threshold`：控制器服务器在将接收到的里程计消息发送给控制器插件之前会过滤速度部分。低于此阈值（以 m/s 为单位）的里程计值将被设置为 0.0。
- `min_y_velocity_threshold`：控制器服务器在将接收到的里程计消息发送给控制器插件之前会过滤速度部分。低于此阈值（以 m/s 为单位）的里程计值将被设置为 0.0。
- `min_theta_velocity_threshold`：控制器服务器在将接收到的里程计消息发送给控制器插件之前会过滤速度部分。低于此阈值（以 rad/s 为单位）的里程计值将被设置为 0.0。
- `failure_tolerance`：被调用的控制器插件在失败之前的最大持续时间（即插件的 `computeVelocityCommands` 函数抛出异常）允许的时间（以秒为单位），否则 `nav2_msgs::action::FollowPath` 动作将失败。将其设置为特殊值 -1.0 可以使其无限制，设置为 0 则禁用，设置为正值可设定适当的超时时间。
- `speed_limit_topic`：要订阅的速度限制主题名称。这可以由速度过滤器发布。
- `odom_topic`：里程计话题。

示例：

```yaml
controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    failure_tolerance: 0.3
    odom_topic: "odom"
    progress_checker_plugins: ["progress_checker"] # progress_checker_plugin: "progress_checker" For Humble and older
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]
    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0
    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
```

### planner_server

- `planner_plugins`：映射到参数和处理请求的插件名称列表，默认加载：`nav2_navfn_planner/NavfnPlanner`
- `expected_planner_frequency`：预期的规划器频率。如果当前频率低于预期频率，则显示警告消息。

示例：

```yaml
planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    planner_plugins: ['GridBased']
    GridBased:
      plugin: 'nav2_navfn_planner/NavfnPlanner'
```

### behavior_server

- `local_costmap_topic`：用于在局部代价地图上进行碰撞检查的原始代价地图主题。

- `global_costmap_topic`：用于在全局代价地图上进行碰撞检查的原始代价地图主题。

- `local_footprint_topic`：在本地代价地图框架中的足迹主题。

- `global_footprint_topic`：全局代价地图框架中的足迹主题。

- `cycle_frequency`：运行行为插件的频率。

- `transform_tolerance`：TF变换容差。

- `local_frame`：本地坐标系。

- `global_frame`：全局坐标系。

- `robot_base_frame`：机器人基准坐标系。

- `behavior_plugins`：要使用的插件名称列表。默认加载：`nav2_behaviors/Spin`（旋转），`nav2_behaviors/BackUp`（后退），`nav2_behaviors/DriveOnHeading`（按照航向前进），`nav2_behaviors/Wait`（等待）

  > - 旋转行为参数：
  >   - `simulate_ahead_time`：向前查找碰撞的时间。
  >   - `max_rotational_vel`：最大旋转速度。
  >   - `min_rotational_vel`：最小旋转速度。
  >   - `rotational_acc_lim`：最大旋转加速度。
  > - 后退行为参数
  >   - `simulate_ahead_time`：向前查找碰撞的时间。
  > - 按目标导航行为参数
  >   - `simulate_ahead_time`：向前查找碰撞的时间。
  > - 辅助遥控行为参数
  >   - `projection_time`：遥控时间。
  >   - `simulation_time_step`：模拟时间步长。
  >   - `cmd_vel_teleop`：用于监听远程操作消息的主题。

示例：

```yaml
behavior_server:
  ros__parameters:
    local_costmap_topic: local_costmap/costmap_raw
    local_footprint_topic: local_costmap/published_footprint
    global_costmap_topic: global_costmap/costmap_raw
    global_footprint_topic: global_costmap/published_footprint
    cycle_frequency: 10.0
    behavior_plugins: ["spin", "backup", "drive_on_heading", "wait", "assisted_teleop"]
    spin:
      plugin: "nav2_behaviors/Spin"
    backup:
      plugin: "nav2_behaviors/BackUp"
    drive_on_heading:
      plugin: "nav2_behaviors/DriveOnHeading"
    wait:
      plugin: "nav2_behaviors/Wait"
    assisted_teleop:
      plugin: "nav2_behaviors/AssistedTeleop"
    local_frame: odom
    global_frame: map
    robot_base_frame: base_link
    transform_timeout: 0.1
    simulate_ahead_time: 2.0
    max_rotational_vel: 1.0
    min_rotational_vel: 0.4
    rotational_acc_lim: 3.2
```

### Costmap_2D

- `always_send_full_costmap`：是否在每次更新时发送完整的代价地图，而不是增量更新。

- `footprint_padding`：机器人足迹的占位符大小（以米为单位）。

- `footprint`：按照字符串传递的机器人足迹轮廓点的有序集合，必须是闭合集合。

- `global_frame`：全局坐标系。

- `height`：代价地图的高度（以米为单位）。

- `width`：代价地图的宽度（以米为单位）。

- `lethal_cost_threshold`：被视为致命障碍物的占位栅格地图的最小代价。

- `map_vis_z`：地图高度（避免 rviz2 闪烁）。

- `observation_sources`：传感器源的字符串列表。

- `origin_x`：与宽度（米）相关的代价地图的X原点。

- `origin_y`：与高度（米）相关的代价地图的Y原点。

- `publish_frequency`：发布代价地图到话题的频率。

- `resolution`：代价地图的每个像素的分辨率，以米为单位。

- `robot_base_frame`：机器人基准坐标系。

- `robot_radius`：如果未提供机器人足迹坐标，则使用的机器人足迹半径。

- `rolling_window`：代价地图是否应与机器人基准坐标系一起滚动。

- `track_unknown_space`：如果为false，则将未知空间视为自由空间，否则视为未知空间。

- `transform_tolerance`：TF变换容差。

- `initial_transform_timeout`：等待从机器人基本框架到全局框架的转换可用的时间。如果超过，配置阶段将中止。

- `trinary_costmap`：如果占用栅格地图只应解释为三个值（自由、占用、未知），或者与其存储的值一起。

- `unknown_cost_value`：如果跟踪未知空间，则未知空间的代价。

- `update_frequency`：代价地图更新频率。

- `use_maximum`：在合并代价地图时是否使用最大代价或覆盖。

- `plugins`：插件名称列表。默认使用`nav2_costmap_2d::StaticLayer`，`nav2_costmap_2d::ObstacleLayer`，`nav2_costmap_2d::InflationLayer`。

  > - 静态层参数`static layer`：
  >   - `enabled`：是否启用。
  >   - `subscribe_to_updates`：在接收到第一个静态地图更新后订阅。
  >   - `map_subscribe_transient_local`：用于地图话题的QoS设置。
  >   - `transform_tolerance`：TF容差。
  >   - `map_topic`：要订阅的地图话题。
  > - 膨胀层参数`inflation layer`：
  >   - `enabled`：是否启用。
  >   - `inflation_radius`：扩展致命障碍物周围的代价地图半径。
  >   - `cost_scaling_factor`：在膨胀半径上的指数衰减因子。
  >   - `inflate_unknown`：是否将未知的单元格视为致命障碍物而进行膨胀。
  >   - `inflate_around_unknown`：是否将未知的单元格进行膨胀。
  > - 障碍物层参数`obstacle layer`：
  >   - `enabled`：是否启用。
  >   - `footprint_clearing_enabled`：清除机器人足迹下的任何占用单元格。
  >   - `max_obstacle_height`：添加到占用格网的最大高度。
  >   - `combination_method`：用于将数据添加到主成本地图的方法的枚举，默认为最大值。
  >   - `observation_sources`：数据源的命名空间。
  >     - `topic`：数据的话题。
  >     - `sensor_frame`：传感器的帧，如果消息中没有提供。如果为空，则使用消息的帧标识。
  >     - `observation_persistence`：在将消息添加到代价地图之前，将其存储在缓冲区中的时间长度（以秒为单位）。
  >     - `expected_update_rate`：从传感器获取新数据的预期速率。
  >     - `data_type`：输入数据的类型。
  >     - `min_obstacle_height`：添加到占据栅格的最小高度值。
  >     - `max_obstacle_height`：添加到占用格网的最大高度。
  >     - `inf_is_valid`：激光扫描仪返回的无穷远处障碍物是否有效。
  >     - `marking`：是否在代价地图中标记来源。
  >     - `clearing`：是否在代价地图中进行光线追踪清除。
  >     - `obstacle_max_range`：在代价地图中标记障碍物的最大范围。
  >     - `obstacle_min_range`：在代价地图中标记障碍物的最小距离。
  >     - `raytrace_max_range`：从代价地图中射线跟踪清除障碍物的最大距离。
  >     - `raytrace_min_range`：从代价地图中射线跟踪清除障碍物的最小距离。
  > - 体素层参数`voxel layer`：
  >   - `enabled`：是否启用。
  >   - `footprint_clearing_enabled`：清除机器人足迹下的任何占用单元格。
  >   - `max_obstacle_height`：添加到占用格网的最大高度。
  >   - `z_voxels`：要标记的垂直体素数，最多16个。
  >   - `origin_z`：开始标记体素的位置（米）。
  >   - `z_resolution`：垂直方向上体素的分辨率（米）。
  >   - `unknown_threshold`：在2D占据网格中，标记为未知状态的列中空心体素的最小数量。
  >   - `mark_threshold`：在2D占据网格中，标记为占用状态的列中的体素最小数量。
  >   - `combination_method`：用于将数据添加到主成本地图的方法的枚举，默认为最大值。
  >   - `publish_voxel_map`：是否为调试目的发布3D体素网格，计算开销较大。
  >   - `observation_sources`：数据源的命名空间。
  >     - `topic`：数据的话题。
  >     - `sensor_frame`：传感器的帧，如果消息中没有提供。如果为空，则使用消息的帧标识。
  >     - `observation_persistence`：在将消息添加到代价地图之前，将其存储在缓冲区中的时间长度（以秒为单位）。
  >     - `expected_update_rate`：从传感器获取新数据的预期速率。
  >     - `data_type`：输入数据的类型。
  >     - `min_obstacle_height`：添加到占据栅格的最小高度值。
  >     - `max_obstacle_height`：添加到占用格网的最大高度。
  >     - `inf_is_valid`：激光扫描仪返回的无穷远处障碍物是否有效。
  >     - `marking`：是否在代价地图中标记来源。
  >     - `clearing`：是否在代价地图中进行光线追踪清除。
  >     - `obstacle_max_range`：在代价地图中标记障碍物的最大范围。
  >     - `obstacle_min_range`：在代价地图中标记障碍物的最小距离。
  >     - `raytrace_max_range`：从代价地图中射线跟踪清除障碍物的最大距离。
  >     - `raytrace_min_range`：从代价地图中射线跟踪清除障碍物的最小距离。
  > - 范围传感器参数`range layer`：
  >   - `enabled`：是否启用。
  >   - `topics`：订阅的范围主题。
  >   - `phi`：Phi 值。
  >   - `inflate_cone`：扩大传感器覆盖的三角区域（百分比）。
  >   - `no_readings_timeout`：如果为零，此参数无效。否则，如果该层在此时间段内未接收到传感器数据，则该层将向用户发出警告，并将该层标记为非当前层。
  >   - `clear_threshold`：将概率低于此值的单元格标记为可通行。
  >   - `mark_threshold`：被标记为占用的单元格的概率上限。
  >   - `clear_on_max_reading`：是否在最大范围上清除传感器读数。
  >   - `input_sensor_type`：输入传感器类型可以是`ALL`（自动选择）、`VARIABLE`（最小范围不等于最大范围）或`FIXED`（最小范围等于最大范围）。
  > - 降噪层参数`denoise layer`：
  >   - `enabled`：是否启用。
  >   - `minimal_group_size`：应视为噪声而不应被丢弃的相邻障碍物的最小数量。如果为1或更少，将保留所有障碍物。如果为2，则会删除独立的障碍物（没有相邻单元格中的邻居）。如果为N，将删除小于N的障碍物组。
  >   - `group_connectivity_type`：障碍物的连通性类型（障碍物与其邻居之间的关系方式）。必须为4或8。4 - 相邻障碍物在水平和垂直方向上连接。8 - 相邻障碍物在水平、垂直和对角线方向上相连。

- `filters`：过滤器名称列表。

  > - 保护区过滤器：通过使用过滤器掩码信息更新相应的代价地图图层，强制机器人避开限制区域或保持在首选车道上。如果希望在使用可行规划器时为机器人的任何部分设置禁止区域，应当启用膨胀层。
  >   - `enabled`：是否启用。
  >   - `filter_info_topic`：传入的 `CostmapFilterInfo` 话题题的名称，其中包含与过滤器相关的信息。由 `Costmap Filter Info Server` 与过滤器掩码话题一起发布。
  >   - `transform_tolerance`：发布的变换的时间，用于表示该变换在未来是有效的。在过滤器掩码和当前代价地图图层位于不同的帧时使用。
  > - 速度过滤器：是一种成本地图过滤器，用于限制机器人的最大速度。机器人需要减速的区域和允许的最大速度值被编码在过滤器掩码中。地图服务器发布的过滤器掩码与成本地图过滤器信息服务器发布的过滤器信息主题配对使用。速度过滤器本身发布限制速度的消息，这些消息针对控制器，以确保机器人不会超过所需速度。
  >   - `enabled`：是否启用。
  >   - `filter_info_topic`：传入的 `CostmapFilterInfo` 话题题的名称，其中包含与过滤器相关的信息。由 `Costmap Filter Info Server` 与过滤器掩码话题一起发布。
  >   - `transform_tolerance`：发布的变换的时间，用于表示该变换在未来是有效的。在过滤器掩码和当前代价地图图层位于不同的帧时使用。
  >   - `speed_limit_topic`：发布速度限制的话题。
  > - 二值过滤器：是一个发布布尔类型主题的 Costmap 过滤器，当编码的过滤器空间值（对应机器人所在的过滤器掩码点）高于给定的阈值时，翻转二进制状态。当低于或等于阈值时，再次翻转回去。
  >   - `enabled`：是否启用。
  >   - `filter_info_topic`：传入的 `CostmapFilterInfo` 话题题的名称，其中包含与过滤器相关的信息。由 `Costmap Filter Info Server` 与过滤器掩码话题一起发布。
  >   - `transform_tolerance`：发布的变换的时间，用于表示该变换在未来是有效的。在过滤器掩码和当前代价地图图层位于不同的帧时使用。
  >   - `default_state`：二进制过滤器的默认状态。
  >   - `binary_state_topic`：以``std_msgs::msg::Bool``类型发布二进制过滤器状态的主题。
  >   - `flip_threshold`：二进制状态翻转的阈值。过滤超过此阈值的值，将将二进制状态设置为非默认状态。

示例：

```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      footprint_padding: 0.03
      update_frequency: 1.0
      publish_frequency: 1.0
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.22 # radius set and used, so no footprint points
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "voxel_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        footprint_clearing_enabled: true
        max_obstacle_height: 2.0
        combination_method: 1
        scan:
          topic: /scan
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          inf_is_valid: false
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        footprint_clearing_enabled: true
        max_obstacle_height: 2.0
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.05
        z_voxels: 16
        max_obstacle_height: 2.0
        unknown_threshold: 15
        mark_threshold: 0
        observation_sources: pointcloud
        combination_method: 1
        pointcloud:  # no frame set, uses frame from message
          topic: /intel_realsense_r200_depth/points
          max_obstacle_height: 2.0
          min_obstacle_height: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          clearing: True
          marking: True
          data_type: "PointCloud2"
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
        enabled: true
        subscribe_to_updates: true
        transform_tolerance: 0.1
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        enabled: true
        inflation_radius: 0.55
        cost_scaling_factor: 1.0
        inflate_unknown: false
        inflate_around_unknown: true
      always_send_full_costmap: True


local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
```

