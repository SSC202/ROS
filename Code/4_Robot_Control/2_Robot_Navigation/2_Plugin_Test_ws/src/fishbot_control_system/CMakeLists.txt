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

add_executable(spin_test src/spin_test.cpp)
ament_target_dependencies(spin_test pluginlib)
install(TARGETS spin_test 
  DESTINATION lib/${PROJECT_NAME}
  )

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
