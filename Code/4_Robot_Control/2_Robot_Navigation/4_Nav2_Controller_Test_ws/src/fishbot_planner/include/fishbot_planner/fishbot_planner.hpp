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
