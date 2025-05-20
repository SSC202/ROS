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