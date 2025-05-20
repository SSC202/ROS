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
        // 自定义规划器 —— 直线规划器

        // 1.声明并初始化 global_path
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
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fishbot_planner::FishbotPlanner, nav2_core::GlobalPlanner)