#include <iostream>
#include "fishbot_control_system/spin_controller.hpp"

namespace fishbot_control_system
{
    /**
     * @brief   旋转运动启动函数
     */
    void SpinController::start()
    {
        std::cout << "start" << std::endl;
    }

    /**
     * @brief   旋转运动停止函数
     */
    void SpinController::stop()
    {
        std::cout << "stop" << std::endl;
    }
}

// 使用 PLUGINLIB_EXPORT_CLASS宏对插件导出
// PLUGINLIB_EXPORT_CLASS 宏定义在 pluginlib/class_list_macros.hpp 中
// 该宏有两个参数：第一个是要导出的类，第二个是导出的类的抽象基类
#include "pluginlib/class_list_macros.hpp" 
PLUGINLIB_EXPORT_CLASS(fishbot_control_system::SpinController, fishbot_control_system::FishbotController)
