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
    pluginlib::ClassLoader<fishbot_control_system::FishbotController>
        controller_loader("fishbot_control_system", "fishbot_control_system::FishbotController");


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
