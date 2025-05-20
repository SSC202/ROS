# ROS Navigation2 自定义规划器和控制器

## 1. ROS2 插件机制

`pluginlib` 是一个 C++ 类库，用于从一个 ROS 包中加载和卸载插件 `plugins`。 

> 插件 `plugins` 是一种能从运行库（例如：共享对象，动态链接库）中动态导入的类。
>
> 使用 `pluginlib` ， 就不必在程序中显式地声明或定义类库，而是可以在代码中的任何位置动态的导入外部的类，甚至不需要知道类库/头文件/类定义，插件可用于扩展/修改应用程序行为，而无需使用应用程序源代码。
>
> `plugins` 在扩展和修改应用程序的时候是非常有用的。例如代价地图层中需要添加一层新的代价地图，用于表示车辆或行人，就需要用到 `pluginlib` 导入这一层的代价地图（一个已经被导出的类，这个类称为插件）。

### 使用 `pluginlib` 创建插件

比如规划器插件，一个机器人支持 A* ，Dijkstra 等规划方法。通常会创建一个基类接口，然后让插件继承基类、实现不同控制方式。

> 1. 基类通常采用抽象类，抽象类(Abstract Class)至少包含一个纯虚函数，只能被继承、不能被实例化。
> 2. 所有同类型插件继承自基类，并且都要编写纯虚函数的具体实现方法。

以下为一个实现的例子：

1. 创建工作空间`<workspace/src>` ，根据以下指令添加功能包：

   ```shell
   $ ros2 pkg create <package> --dependencies pluginlib --license Apache-2.0
   ```

   该功能包包含依赖 `pluginlib` 。

2. 在 `<workspace/src/<package>/include/<workspace>/src>` 下，新建基类库 `<<pluginname_interface>.hpp>`：

   ```c++
   #ifndef FISHBOT_CONTROL_INTERFACE_HPP
   #define FISHBOT_CONTROL_INTERFACE_HPP
   
   namespace fishbot_control_system
   {
       class FishbotController
       {
           // 定义抽象基类，至少包含一个纯虚函数，只能被继承、不能被实例化
           // virtual关键字声明虚函数；=0语法声明纯虚函数。
           // 所有插件都要继承该抽象类，并编写基类函数的具体实现。
       public:
           virtual void start() = 0;       // 开始运动
           virtual void stop() = 0;        // 停止运动
           virtual ~FishbotController() {} // 析构函数
       };
   
   }
   
   #endif // FISHBOT_CONTROL_INTERFACE_HPP
   ```

3. 编写插件

   - 新建头文件 `<<pluginname>.hpp>`

     ```c++
     #ifndef SPIN_CONTROLLER_HPP
     #define SPIN_CONTROLLER_HPP
     #include "fishbot_control_system/fishbot_control_interface.hpp"
     
     namespace fishbot_control_system
     {
         // 继承于抽象基类
         class SpinController : public FishbotController
         {
         public:
         // override 表示派生类的成员函数，将覆盖基类的虚函数
             void start() override; 
             void stop() override;
         };
     }
     
     #endif // SPIN_CONTROLLER_HPP
     ```

   - 实现各个函数，在 `<workspace>/src/<package>/src` 下新建文件 `<pluginname>.cpp`：

     ```c++
     #include <iostream>
     #include "fishbot_control_system/spin_controller.hpp"
     
     namespace fishbot_control_system
     {
         /**
          * @brief   旋转运动启动函数
          */
         void SpinController::start()
         {
             std::cout << "SpinMotionController::start" << std::endl;
         }
     
         /**
          * @brief   旋转运动停止函数
          */
         void SpinController::stop()
         {
     
             std::cout << "SpinMotionController:stop" << std::endl;
         }
     }
     
     // 使用 PLUGINLIB_EXPORT_CLASS宏对插件导出
     // PLUGINLIB_EXPORT_CLASS 宏定义在 pluginlib/class_list_macros.hpp 中
     // 该宏有两个参数：第一个是要导出的类，第二个是导出的类的抽象基类
     #include "pluginlib/class_list_macros.hpp" 
     PLUGINLIB_EXPORT_CLASS(fishbot_control_system::SpinController, fishbot_control_system::FishbotController)
     ```

   - 编写插件描述函数，在`<workspace>/src/<package>`中新建`<<pluginname>.xml>`文件：

     ```xml
     <library path="spin_controller">
         <!-- 子标签 class，指定插件类的名字、类、抽象基类-->
         <class name="fishbot_control_system/SpinController" type="fishbot_control_system::SpinController" base_class_type="fishbot_control_system::FishbotController">
             <description>Spin Controller</description>
         </class>
     </library>
     ```

   - 修改 `CMakeslists.txt` ，生成动态库。

     ```cmake
     cmake_minimum_required(VERSION 3.8)
     project(fishbot_control_system)
     
     if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
       add_compile_options(-Wall -Wextra -Wpedantic)
     endif()
     
     # find dependencies
     find_package(ament_cmake REQUIRED)
     find_package(pluginlib REQUIRED)
     
     include_directories(include)
     # 添加库文件
     # 第一个参数是库的名字，第二个参数SHARED表示生成动态库，第三个参数是库对应的源代码文件
     add_library(spin_controller SHARED src/spin_controller.cpp)
     ament_target_dependencies(spin_controller pluginlib)
       
     install(TARGETS spin_controller
       ARCHIVE DESTINATION lib
       LIBRARY DESTINATION lib
       RUNTIME DESTINATION bin
       )
     install(DIRECTORY include/
       DESTINATION include/
       )
     
     # 导出插件描述文件
     # 第一个参数是功能包名字，第二个参数是插件描述文件名字
     pluginlib_export_plugin_description_file(fishbot_control_system spin_plugins.xml)
     
     if(BUILD_TESTING)
       find_package(ament_lint_auto REQUIRED)
       # the following line skips the linter which checks for copyrights
       # comment the line when a copyright and license is added to all source files
       set(ament_cmake_copyright_FOUND TRUE)
       # the following line skips cpplint (only works in a git repo)
       # comment the line when this package is in a git repo and when
       # a copyright and license is added to all source files
       set(ament_cmake_cpplint_FOUND TRUE)
       ament_lint_auto_find_test_dependencies()
     endif()
     
     ament_package()
     ```

   5. 使用 `colcon build` 进行构建。构建后，目录 `install/<package>/lib/` 下可以看到动态库 `lib<pluginname>.so`
   
   6. 在 `src` 下编写测试程序：
   
      ```c++
      #include "fishbot_control_system/fishbot_control_interface.hpp" // 抽象基类头文件
      #include <pluginlib/class_loader.hpp>                           // pluginlib库
      
      int main(int argc, char **argv)
      {
          // 判断命令行的参数数量是否合法
          if (argc != 2)
              return 0;
          // 通过命令行参数，选择要加载的插件，argv[0] 是可执行文件名，argv[1] 表示参数名
          // 从参数数组中，获取控制器的名字
          std::string controller_name = argv[1]; 
          
          /**
           * @brief   加载和使用插件
           */
      
          // 通过功能包名称和基类名称创建控制器加载器
          // 类加载器 ClassLoader 的对象 controller_loader：第一个参数是功能包名字；第二个参数是控制器基类的名字。
          pluginlib::ClassLoader<fishbot_control_system::MotionController>
              controller_loader("motion_control_system", "motion_control_system::MotionController");
      
      
          // 使用加载器加载指定名称的插件，返回的是指定插件类的对象的指针
          // 通过控制器名称，创建类加载器实例指针
          // 控制器名称，是在插件描述文件中定义的名称。
          auto controller = controller_loader.createSharedInstance(controller_name);
      
          // 调用插件的方法
          // 调用控制器的成员方法进行控制
          controller->start();
          controller->stop();
          return 0;
      }
      ```
   
   7. 修改 `CMakeLists.txt`：
   
      ```cmake
      add_executable(spin_test src/spin_test.cpp)
      ament_target_dependencies(spin_test pluginlib)
      install(TARGETS spin_test 
        DESTINATION lib/${PROJECT_NAME}
        )
      ```
   
   8. 运行测试
   
      ```shell
      $ ros2 run fishbot_control_system spin_test fishbot_control_system/SpinController
      ```
   
## 2. 自定义规划器

路径规划器基于给定的机器人初始位姿、目标位姿、环境地图，计算出一条可行的路径。

### 接口定义

- 机器人位姿：在 Navigation 2 中，机器人位姿使用消息接口 `geometry_msgs/msg/PoseStamped` 表示。
  
  ```
  # A Pose with reference coordinate frame and timestamp
  
  std_msgs/Header header
          builtin_interfaces/Time stamp
                  int32 sec
                  uint32 nanosec
          string frame_id
  Pose pose
          Point position
                  float64 x
                  float64 y
                  float64 z
          Quaternion orientation
                  float64 x 0
                  float64 y 0
                  float64 z 0
                  float64 w 1
  ```
  
- 路径：路径使用消息接口 `nav_msgs/msg/Path` 表示，路径为点的数组 `poses[]`。（一条路径按一定距离采样，可得到一个用于表示路径的点集合）
  
  ```
  # An array of poses that represents a Path for a robot to follow.
     
  # Indicates the frame_id of the path.
  std_msgs/Header header
          builtin_interfaces/Time stamp
                  int32 sec
                  uint32 nanosec
          string frame_id
     
  # Array of poses to follow.
  geometry_msgs/PoseStamped[] poses
          std_msgs/Header header
                  builtin_interfaces/Time stamp
                          int32 sec
                          uint32 nanosec
                  string frame_id
          Pose pose
                  Point position
                          float64 x
                          float64 y
                          float64 z
                  Quaternion orientation
                          float64 x 0
                          float64 y 0
                          float64 z 0
                          float64 w 1
  ```
  
- 环境地图：导航所使用的是占据栅格地图，其对应的消息接口是 `nav_msgs/msg/OccupancyGrid` 。
  
  ```
  # This represents a 2-D grid map
  std_msgs/Header header
          builtin_interfaces/Time stamp
                  int32 sec
                  uint32 nanosec
          string frame_id
     
  # MetaData for the map 地图map的基础信息
  MapMetaData info
          builtin_interfaces/Time map_load_time
                  int32 sec
                  uint32 nanosec
          float32 resolution
          uint32 width
          uint32 height
          geometry_msgs/Pose origin
                  Point position
                          float64 x
                          float64 y
                          float64 z
                  Quaternion orientation
                          float64 x 0
                          float64 y 0
                          float64 z 0
                          float64 w 1
     
  # The map data, in row-major order, starting with (0,0). 从地图左上角开始、从左到右按行存储的实际数据数组
  # Cell (1, 0) will be listed second, representing the next cell in the x direction.
  # Cell (0, 1) will be at the index equal to info.width, followed by (1, 1).
  # The values inside are application dependent, but frequently,
  # 0 represents unoccupied, 1 represents definitely occupied, and
  # -1 represents unknown.
  int8[] data
  ```
  
  > 在地图坐标系下，如果要判断路径上某位置$ (x,y) $是否有障碍物（即对应栅格的占据状态）：
  >
  > 1. 将$ (x, y) $坐标转换为 `data` 数组的索引 ` (row_index, col_index)`；
  > 2. 获取对应栅格的占据状态。
  > 3. `info.origin.x` 和 `info.origin.y` 是地图原点的 x 和 y 坐标。`info.resolution` 是地图分辨率。
  >
  > ```c++
  > # 将位置坐标 (x,y) 转换为 data 数组的索引 (row_index, col_index)。
  > row_index = (y - info.origin.y) / info.resolution
  > col_index = (x - info.origin.x) / info.resolution
  > 
  > # 获取栅格的占据状态
  > occupied_status = data[row_index * map_width + col_index]
  > ```
  
### 构建规划器框架

1. 新建功能包，依赖为 `pluginlib` 和 `nav2_core`。

2. 创建 `<plannername>.hpp`

   ```c++
   #ifndef FISHBOT_PLANNER_HPP
   #define FISHBOT_PLANNER_HPP
   
   #include <memory>
   #include <string>
   #include "geometry_msgs/msg/point.hpp"
   #include "geometry_msgs/msg/pose_stamped.hpp"
   #include "rclcpp/rclcpp.hpp"
   #include "nav2_core/global_planner.hpp"       // 全局路径规划器文件
   #include "nav2_costmap_2d/costmap_2d_ros.hpp" // 2D 代价地图文件
   #include "nav2_util/lifecycle_node.hpp"       // 生命周期节点文件
   #include "nav2_util/robot_utils.hpp"          // 机器人硬件文件
   #include "nav_msgs/msg/path.hpp"              // 路径话题文件
   
   namespace fishbot_planner
   {
       // 全局规划器类，继承自抽象基类 GlobalPlanner
       class FishbotPlanner : public nav2_core::GlobalPlanner
       {
       public:
           FishbotPlanner() = default;
           ~FishbotPlanner() = default;
   
           /**
            * @brief   插件配置函数
            */
           void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                          std::shared_ptr<tf2_ros::Buffer> tf,
                          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
   
           /**
            * @brief    插件清理函数
            */
           void cleanup() override;
   
           /**
            * @brief   插件激活函数
            */
           void activate() override;
   
           /**
            * @brief   插件停止函数
            */
           void deactivate() override;
   
           /**
            * @brief   路径规划函数
            */
           nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped &start,
                                          const geometry_msgs::msg::PoseStamped &goal) override;
   
       private:
           /**
            * @brief 参数存储
            */
   
           std::shared_ptr<tf2_ros::Buffer> tf_;      // 坐标变换缓存指针，可用于查询坐标关系
           nav2_util::LifecycleNode::SharedPtr node_; // 节点指针
           nav2_costmap_2d::Costmap2D *costmap_;      // 全局代价地图
           std::string global_frame_, name_;          // 全局代价地图的坐标系
           double interpolation_resolution_;          // 插值分辨率
       };
   
   }
   
   #endif
   ```

   > 插件加载完成后，先调用 `configure()` 配置，再调用 `activate()` 激活，需要路径规划时调用 `createPlan()` 获取路径，退出时先调用 `deactivate()`、再调用 `cleanup()` 清理。

3. 创建 `<plannername>.cpp`

   ```c++
   #include "nav2_util/node_utils.hpp"
   #include <cmath>
   #include <memory>
   #include <string>
   
   #include "nav2_core/exceptions.hpp"
   #include "fishbot_planner/fishbot_planner.hpp"
   
   namespace fishbot_planner
   {
       void FishbotPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
                                      std::shared_ptr<tf2_ros::Buffer> tf,
                                      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
       {
           tf_ = tf;
           node_ = parent.lock();
           name_ = name;
           // 代价地图 costmap_ 和 全局坐标系 global_frame_，直接从参数 costmap_ros 中获取
           costmap_ = costmap_ros->getCostmap();
           global_frame_ = costmap_ros->getGlobalFrameID();
           // 参数 interpolation_resolution，需要先声明、再获取
           nav2_util::declare_parameter_if_not_declared(
               node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
           node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
       }
   
       void FishbotPlanner::cleanup()
       {
           RCLCPP_INFO(node_->get_logger(), "正在清理类型为 FishbotPlanner 的插件 %s", name_.c_str());
       }
   
       void FishbotPlanner::activate()
       {
           RCLCPP_INFO(node_->get_logger(), "正在激活类型为 FishbotPlanner 的插件 %s", name_.c_str());
       }
   
       void FishbotPlanner::deactivate()
       {
           RCLCPP_INFO(node_->get_logger(), "正在停用类型为 FishbotPlanner 的插件 %s", name_.c_str());
       }
   
       nav_msgs::msg::Path FishbotPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                                      const geometry_msgs::msg::PoseStamped &goal)
       {
           nav_msgs::msg::Path global_path;
           // 自定义规划器
           return global_path;
       }
   }
   
   #include "pluginlib/class_list_macros.hpp"
   PLUGINLIB_EXPORT_CLASS(fishbot_planner::FishbotPlanner, nav2_core::GlobalPlanner)
   ```

4. 创建插件描述文件(同上一节)

5. 修改 `CMakeLists.txt`

   ```cmake
   cmake_minimum_required(VERSION 3.8)
   project(fishbot_planner)
   
   if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
     add_compile_options(-Wall -Wextra -Wpedantic)
   endif()
   
   # find dependencies
   find_package(ament_cmake REQUIRED)
   find_package(pluginlib REQUIRED)
   find_package(nav2_core REQUIRED)
   
   include_directories(include)
   # 定义库名称
   set(library_name ${PROJECT_NAME}_plugin)
   # 创建共享库
   add_library(${library_name} SHARED  src/fishbot_planner.cpp)
   # 指定库的依赖关系
   ament_target_dependencies(${library_name} nav2_core pluginlib)
   # 安装库文件到指定目录
   install(TARGETS ${library_name}
     ARCHIVE DESTINATION lib
     LIBRARY DESTINATION lib
     RUNTIME DESTINATION lib/${PROJECT_NAME})
   # 安装头文件到指定目录
   install(DIRECTORY include/
     DESTINATION include/ )
   # 导出插件描述文件
   pluginlib_export_plugin_description_file(nav2_core fishbot_planner_plugin.xml)
   
   if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)
     # the following line skips the linter which checks for copyrights
     # comment the line when a copyright and license is added to all source files
     set(ament_cmake_copyright_FOUND TRUE)
     # the following line skips cpplint (only works in a git repo)
     # comment the line when this package is in a git repo and when
     # a copyright and license is added to all source files
     set(ament_cmake_cpplint_FOUND TRUE)
     ament_lint_auto_find_test_dependencies()
   endif()
   
   ament_package()
   ```

6. 修改 `package.xml`

   ```xml
   <?xml version="1.0"?>
   <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
   <package format="3">
     <name>fishbot_planner</name>
     <version>0.0.0</version>
     <description>TODO: Package description</description>
     <maintainer email="ssc@todo.todo">ssc</maintainer>
     <license>Apache-2.0</license>
   
     <buildtool_depend>ament_cmake</buildtool_depend>
   
     <depend>pluginlib</depend>
     <depend>nav2_core</depend>
   
     <test_depend>ament_lint_auto</test_depend>
     <test_depend>ament_lint_common</test_depend>
   
     <export>
       <build_type>ament_cmake</build_type>
       <nav2_core plugin="${prefix}/fishbot_planner_plugin.xml"/>
     </export>
   </package>
   ```

7. `colcon build` 即可。

### 自定义规划器

这里仅仅实现最简单的直线规划器：当收到规划请求时， 直接生成一个从当前位置到目标位置的直线路径，同时判断路径上是否有障碍物，如果存在则直接抛出异常，表示规划失败。

```c++
    nav_msgs::msg::Path FishbotPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                                                   const geometry_msgs::msg::PoseStamped &goal)
    {
        nav_msgs::msg::Path global_path;
        // 自定义规划器 —— 直线规划器

        // 1.初始化 global_path
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // 2.检查目标点和起始点是否在全局坐标系中
        if (start.header.frame_id != global_frame_)
        {
            RCLCPP_ERROR(node_->get_logger(), "规划器仅接受来自 %s 坐标系的起始位置", global_frame_.c_str());
            return global_path;
        }
        if (goal.header.frame_id != global_frame_)
        {
            RCLCPP_INFO(node_->get_logger(), "规划器仅接受来自 %s 坐标系的目标位置", global_frame_.c_str());
            return global_path;
        }

        // 3.计算当前插值分辨率 interpolation_resolution_ 下的循环次数和步进值
        int total_number_of_loop = // 起始点 & 目标点 → 路径长度，路径长度 / 插值分辨率 → 循环次数
            std::hypot(goal.pose.position.x - start.pose.position.x, goal.pose.position.y - start.pose.position.y) / interpolation_resolution_;
        double x_increment = //  循环次数 → x和y方向每次插值的步长。
            (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
        double y_increment =
            (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

        // 4.生成路径，存入 global_path 中。
        for (int i = 0; i < total_number_of_loop; ++i)
        {
            geometry_msgs::msg::PoseStamped pose; // 生成一个点
            pose.pose.position.x = start.pose.position.x + x_increment * i;
            pose.pose.position.y = start.pose.position.y + y_increment * i;
            pose.pose.position.z = 0.0;
            pose.header.stamp = node_->now();
            pose.header.frame_id = global_frame_;
            // 将该点放到路径中
            global_path.poses.push_back(pose);
        }

        // 5.使用 costmap 检查该条路径是否经过障碍物
        for (geometry_msgs::msg::PoseStamped pose : global_path.poses)
        {
            unsigned int mx, my; // 将点的坐标转换为栅格坐标
            if (costmap_->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my))
            {                                                   // 坐标点转换成栅格坐标 (mx, my)
                unsigned char cost = costmap_->getCost(mx, my); // 获取对应栅格的代价值
                if (cost == nav2_costmap_2d::LETHAL_OBSTACLE)
                { // 如果存在致命障碍物，则抛出异常
                    RCLCPP_WARN(node_->get_logger(), "在 (%f,%f) 检测到障碍物，规划失败。", pose.pose.position.x, pose.pose.position.y);
                    throw nav2_core::PlannerException("无法创建目标规划:" + std::to_string(goal.pose.position.x) + "," + std::to_string(goal.pose.position.y));
                }
            }
        }

        // 6.若没有障碍物，收尾，将目标点作为路径的最后一个点，并返回路径
        geometry_msgs::msg::PoseStamped goal_pose = goal;
        goal_pose.header.stamp = node_->now();
        goal_pose.header.frame_id = global_frame_;
        global_path.poses.push_back(goal_pose);
        return global_path;
        return global_path;
    }
```

## 3. 自定义控制器

### 构建控制器框架

1. 新建功能包，依赖为 `pluginlib` 和 `nav2_core`。

2. 创建 `<controllername>.hpp`

   ```c++
   #ifndef FISHBOT_PLANNER
   #define FISHBOT_PLANNER
   
   #include <memory>
   #include <string>
   #include <vector>
   #include "nav2_core/controller.hpp"
   #include "nav2_util/robot_utils.hpp"
   
   namespace fishbot_controller
   {
       // 控制器类，继承自抽象基类 Controller
       class FishbotController : public nav2_core::Controller
       {
       public:
           FishbotController() = default;
           ~FishbotController() override = default;
   
           /**
            * @brief   插件配置函数
            */
           void configure(
               const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
               std::shared_ptr<tf2_ros::Buffer> tf,
               std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
   
           /**
            * @brief   插件清理函数
            */
           void cleanup() override;
   
           /**
            * @brief   插件激活函数
            */
           void activate() override;
   
           /**
            * @brief   插件停止函数
            */
           void deactivate() override;
   
           /**
            * @brief 根据当前姿态和速度计算最佳命令,假定全局路径已经设置
            * @param pose 当前机器人姿态
            * @param velocity 当前机器人速度
            * @param goal_checker 任务正在使用的当前目标检查器的指针
            * @return 机器人导航的最佳命令
            */
           geometry_msgs::msg::TwistStamped computeVelocityCommands(
               const geometry_msgs::msg::PoseStamped &pose,
               const geometry_msgs::msg::Twist &velocity,
               nav2_core::GoalChecker *goal_checker) override;
   
           /**
            * @brief 设置全局路径的方法
            * @param path 全局路径
            */
           void setPlan(const nav_msgs::msg::Path &path) override;
   
           /**
            * @brief 限制机器人的最大线速度。
            * @param speed_limit 绝对值表示的速度限制(以m/s为单位)或从最大机器人速度的百分比表示。
            * @param percentage 如果为 true, 则以百分比设置速度限制，如果为 false， 则以绝对值设置速度限制。
            */
           void setSpeedLimit(const double &speed_limit, const bool &percentage) override;
   
       private:
           std::string plugin_name_;                                    // 存储插件名称
           std::shared_ptr<tf2_ros::Buffer> tf_;                        // 存储坐标变换缓存指针，可用于查询坐标关系
           std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_; // 存储代价地图
           nav2_util::LifecycleNode::SharedPtr node_;                   // 存储节点指针
           nav2_costmap_2d::Costmap2D *costmap_;                        // 存储全局代价地图
           nav_msgs::msg::Path global_plan_;                            // 存储 setPlan 提供的全局路径
           double max_angular_speed_;                                   // 最大线速度角速度
           double max_linear_speed_;
   
           /**
            * @brief   获取路径中距离当前点最近的点
            */
           geometry_msgs::msg::PoseStamped getNearestTargetPose(
               const geometry_msgs::msg::PoseStamped &current_pose);
   
           /**
            * @brief   计算目标点方向和当前位置的角度差
            */
           double calculateAngleDifference(
               const geometry_msgs::msg::PoseStamped &current_pose,
               const geometry_msgs::msg::PoseStamped &target_pose);
       };
   }
   
   #endif
   ```

   > 1. 先调用 `setPlan()` 将路径通过参数传递给控制器；
   > 2. 传递当前机器人位置等参数给  `computeVelocityCommands()`，该方法根据控制算法返回需要下发的速度命令；
   > 3. `setSpeedLimit()`  用于设置机器人的最大速度，该方法即使不调用也不会影响控制器进行路径跟踪。

3. 创建 `<controllername>.cpp`

   ```c++
   #include "fishbot_controller/fishbot_controller.hpp"
   #include "nav2_core/exceptions.hpp"
   #include "nav2_util/geometry_utils.hpp"
   #include "nav2_util/node_utils.hpp"
   #include <algorithm>
   #include <chrono>
   #include <iostream>
   #include <memory>
   #include <string>
   #include <thread>
   
   namespace fishbot_controller
   {
       void FishbotController::configure(
           const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
           std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
           std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
       {
           node_ = parent.lock();
           costmap_ros_ = costmap_ros;
           tf_ = tf;
           plugin_name_ = name;
   
           // 设置最大线速度 max_linear_speed_ 和最大角速度 max_angular_speed_ ，需要先声明再获取参数，
           nav2_util::declare_parameter_if_not_declared(
               node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
           node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
           nav2_util::declare_parameter_if_not_declared(
               node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
           node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
       }
   
       void FishbotController::cleanup()
       {
           RCLCPP_INFO(node_->get_logger(), "清理控制器: %s 类型为 nav2_custom_controller::CustomController",
                       plugin_name_.c_str());
       }
   
       void FishbotController::activate()
       {
           RCLCPP_INFO(node_->get_logger(), "激活控制器: %s 类型为 nav2_custom_controller::CustomController",
                       plugin_name_.c_str());
       }
   
       void FishbotController::deactivate()
       {
           RCLCPP_INFO(node_->get_logger(), "停用控制器: %s 类型为 nav2_custom_controller::CustomController",
                       plugin_name_.c_str());
       }
   
       geometry_msgs::msg::TwistStamped FishbotController::computeVelocityCommands(
           const geometry_msgs::msg::PoseStamped &pose,
           const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *)
       {
           (void)pose;
           geometry_msgs::msg::TwistStamped cmd_vel;
           return cmd_vel;
       }
   
       void FishbotController::setSpeedLimit(const double &speed_limit, const bool &percentage)
       {
           (void)percentage;
           (void)speed_limit;
       }
   
       void FishbotController::setPlan(const nav_msgs::msg::Path &path)
       {
           global_plan_ = path;
       }
   
       geometry_msgs::msg::PoseStamped FishbotController::getNearestTargetPose(
           const geometry_msgs::msg::PoseStamped &current_pose)
       {
           // TODO: 获取最接近目标的点
           return current_pose;
       }
   
       double FishbotController::calculateAngleDifference(
           const geometry_msgs::msg::PoseStamped &current_pose,
           const geometry_msgs::msg::PoseStamped &target_pose)
       {
           (void)current_pose;
           (void)target_pose;
           // 计算当前姿态与目标姿态之间的角度差
           return .0;
       }
   }
   
   #include "pluginlib/class_list_macros.hpp"
   PLUGINLIB_EXPORT_CLASS(fishbot_controller::FishbotController, nav2_core::Controller)
   ```

4. 剩余操作仿照规划器即可。

### 自定义控制器

本节只采用最简单的原地旋转和直行策略。当检测到目标点方向和当前机器人朝向角度差较大时，则原地旋转到目标点方向，否则则朝目标点前进；因为要跟随路径，不能直接选择路径终点为路标点。将距离机器人当前位置最近的点的下一个点作为目标点。

```c++
#include "fishbot_controller/fishbot_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace fishbot_controller
{
    void FishbotController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        node_ = parent.lock();
        costmap_ros_ = costmap_ros;
        tf_ = tf;
        plugin_name_ = name;

        // 设置最大线速度 max_linear_speed_ 和最大角速度 max_angular_speed_ ，需要先声明再获取参数，
        nav2_util::declare_parameter_if_not_declared(
            node_, plugin_name_ + ".max_linear_speed", rclcpp::ParameterValue(0.1));
        node_->get_parameter(plugin_name_ + ".max_linear_speed", max_linear_speed_);
        nav2_util::declare_parameter_if_not_declared(
            node_, plugin_name_ + ".max_angular_speed", rclcpp::ParameterValue(1.0));
        node_->get_parameter(plugin_name_ + ".max_angular_speed", max_angular_speed_);
    }

    void FishbotController::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "清理控制器: %s 类型为 nav2_custom_controller::CustomController",
                    plugin_name_.c_str());
    }

    void FishbotController::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "激活控制器: %s 类型为 nav2_custom_controller::CustomController",
                    plugin_name_.c_str());
    }

    void FishbotController::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "停用控制器: %s 类型为 nav2_custom_controller::CustomController",
                    plugin_name_.c_str());
    }

    geometry_msgs::msg::TwistStamped FishbotController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *)
    {
        // 1.检查路径是否为空
        if (global_plan_.poses.empty())
        {
            throw nav2_core::PlannerException("收到长度为零的路径");
        }

        // 2.调用 transformPoseInTargetFrame()，将机器人当前姿态(默认里程计坐标系)转换到全局map坐标系中
        geometry_msgs::msg::PoseStamped pose_in_globalframe;
        if (!nav2_util::transformPoseInTargetFrame(pose, pose_in_globalframe, *tf_, global_plan_.header.frame_id, 0.1))
        {
            throw nav2_core::PlannerException("无法将机器人姿态转换为全局坐标系"); // 转换失败则抛出异常、终止代码
        }

        // 3.获取最近的目标点和计算角度差
        auto target_pose = getNearestTargetPose(pose_in_globalframe);
        auto angle_diff = calculateAngleDifference(pose_in_globalframe, target_pose);

        // 4.根据角度差，计算线速度和角速度
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = pose_in_globalframe.header.frame_id;
        cmd_vel.header.stamp = node_->get_clock()->now();
        // 根据角度差计算速度
        if (fabs(angle_diff) > M_PI / 10.0)
        { // 如果角度差绝对值 > π/10(即18°)，则机器人停止前进、只原地旋转，角速度设为最大角速度(max_angular_speed_)
            cmd_vel.twist.linear.x = .0;
            cmd_vel.twist.angular.z = fabs(angle_diff) / angle_diff * max_angular_speed_;
        }
        else
        { // 否则直行，线速度设为最大线速度(max_linear_speed_)，角速度为0
            cmd_vel.twist.linear.x = max_linear_speed_;
            cmd_vel.twist.angular.z = .0;
        }
        RCLCPP_INFO(node_->get_logger(), "控制器: %s 发送速度 (%f,%f)",
                    plugin_name_.c_str(), cmd_vel.twist.linear.x, cmd_vel.twist.angular.z);
        return cmd_vel;
    }

    void FishbotController::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)percentage;
        (void)speed_limit;
    }

    void FishbotController::setPlan(const nav_msgs::msg::Path &path)
    {
        global_plan_ = path;
    }

    geometry_msgs::msg::PoseStamped FishbotController::getNearestTargetPose(
        const geometry_msgs::msg::PoseStamped &current_pose)
    {
        // 1.根据当前位姿遍历路径，获取路径中距离当前点最近的点的索引，存储到 nearest_pose_index
        using nav2_util::geometry_utils::euclidean_distance;
        int nearest_pose_index = 0;
        double min_dist = euclidean_distance(current_pose, global_plan_.poses.at(0));
        for (unsigned int i = 1; i < global_plan_.poses.size(); i++)
        {
            double dist = euclidean_distance(current_pose, global_plan_.poses.at(i));
            if (dist < min_dist)
            {
                nearest_pose_index = i;
                min_dist = dist;
            }
        }

        // 2.从路径中，擦除全局路径数组中从开始到最近点的路径数据（这样最近点就变成了索引为0的点）
        global_plan_.poses.erase(std::begin(global_plan_.poses), std::begin(global_plan_.poses) + nearest_pose_index);

        // 3.如果只有一个点，则直接返回最近点；否则返回最近点的下一个点，作为目标点
        if (global_plan_.poses.size() == 1)
        {
            return global_plan_.poses.at(0);
        }
        return global_plan_.poses.at(1);
    }

    double FishbotController::calculateAngleDifference(
        const geometry_msgs::msg::PoseStamped &current_pose,
        const geometry_msgs::msg::PoseStamped &target_pose)
    {
        // 1.调用 tf2::getYaw() 获取当前机器人朝向
        float current_robot_yaw = tf2::getYaw(current_pose.pose.orientation);

        // 2.通过 std::atan2() 获取目标点相对当前点的朝向
        float target_angle = std::atan2(
            target_pose.pose.position.y - current_pose.pose.position.y,
            target_pose.pose.position.x - current_pose.pose.position.x);

        // 3.计算角度差，并转换到 -M_PI 到 M_PI 之间(限定角度差范围)
        double angle_diff = target_angle - current_robot_yaw;
        if (angle_diff < -M_PI)
        {
            angle_diff += 2.0 * M_PI;
        }
        else if (angle_diff > M_PI)
        {
            angle_diff -= 2.0 * M_PI;
        }
        return angle_diff;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fishbot_controller::FishbotController, nav2_core::Controller)
```

