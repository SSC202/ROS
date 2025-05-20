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