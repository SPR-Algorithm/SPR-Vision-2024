// Copyright 2022 Chen Jun
// Licensed under the MIT License.

//接受tracker发布的target，进行装甲板选择，计算yaw和pitch角度，发布数据
#ifndef TARGET_SOLVE__TARGET_SOLVE_NODE_HPP_
#define TARGET_SOLVE__TARGET_SOLVE_NODE_HPP_
// ROS

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

// STD
#include <memory>
#include <string>
#include <vector>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "auto_aim_interfaces/msg/gimbal.hpp"
#include "target_solve/strategy.hpp"

namespace rm_auto_aim
{

class TargetSolveNode : public rclcpp::Node
{
public:
   explicit TargetSolveNode(const rclcpp::NodeOptions & options);


    //策略选择器实例化
    std::unique_ptr<Strategy> strategy_;
    

    void targetCallback(auto_aim_interfaces::msg::Target::SharedPtr msg);

    float shoot_speed;
    float  shooter_height;

    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub2_;

    rclcpp::Publisher<auto_aim_interfaces::msg::Gimbal>::SharedPtr gimbal_pub_;

  //void get_target(auto_aim_interfaces::msg::Target::SharedPtr msg);


};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_NODE_HPP_


