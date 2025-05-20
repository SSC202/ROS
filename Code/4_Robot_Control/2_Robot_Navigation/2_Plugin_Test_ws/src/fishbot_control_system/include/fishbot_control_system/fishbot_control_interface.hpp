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