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