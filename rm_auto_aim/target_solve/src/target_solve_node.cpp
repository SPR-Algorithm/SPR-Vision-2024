
// Copyright 2022 Chen Jun
// Licensed under the MIT License.



#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/qos.hpp>

// STD
#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include  "target_solve/strategy.hpp"
#include  "target_solve/target_solve_node.hpp"

namespace rm_auto_aim
{
TargetSolveNode::TargetSolveNode(const rclcpp::NodeOptions & options)
: Node("target_solve", options)
{
  RCLCPP_INFO(get_logger(), "Start Target_solve!");

    // Create Subscription
  target_sub2_ = this->create_subscription<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::SensorDataQoS(),
    std::bind(&TargetSolveNode::targetCallback, this, std::placeholders::_1));

  // Create Publisher
  gimbal_pub_ = this->create_publisher<auto_aim_interfaces::msg::Gimbal>("/gimbal", 10);

  strategy_ = std::make_unique<Strategy>();

  shoot_speed = this->declare_parameter("shooter.speed", 30.0);
  shooter_height=this->declare_parameter("shooter_height",0.325);//单位：m
}

void TargetSolveNode::targetCallback(auto_aim_interfaces::msg::Target::SharedPtr msg)
{
    //RCLCPP_INFO(get_logger(), "start callback:");
    Strategy::TargetData target_data;
    //数据包  
    target_data.tracking =msg->tracking;
    target_data.id = msg->id;
    target_data.armors_num = msg->armors_num;
    target_data.x = msg->position.x;
    target_data.y = msg->position.y;
    target_data.z = msg->position.z;
    target_data.yaw = msg->yaw;
    target_data.vx = msg->velocity.x;
    target_data.vy = msg->velocity.y;
    target_data.vz = msg->velocity.z;
    target_data.v_yaw = msg->v_yaw;
    target_data.r1 = msg->radius_1;
    target_data.r2 = msg->radius_2;
    target_data.dz = msg->dz;
      
      // RCLCPP_INFO(get_logger(), "tracking: %u", target_data.tracking);
      // RCLCPP_INFO(get_logger(), "armors_num: %u", target_data.armors_num);
      // RCLCPP_INFO(get_logger(), "x: %f", target_data.x);
      // RCLCPP_INFO(get_logger(), "y: %f", target_data.x);
      // RCLCPP_INFO(get_logger(), "z: %f", target_data.z);
      // RCLCPP_INFO(get_logger(), "yaw: %f",target_data.yaw);
      // RCLCPP_INFO(get_logger(), "v_x: %f",target_data.vx);
      // RCLCPP_INFO(get_logger(), "v_y: %f", target_data.vy);
      // RCLCPP_INFO(get_logger(), "v_z: %f", target_data.vz);
      // RCLCPP_INFO(get_logger(), "v_yaw: %f", target_data.v_yaw);
      // RCLCPP_INFO(get_logger(), "r1: %f", target_data.r1);
      // RCLCPP_INFO(get_logger(), "r2: %f", target_data.r2);
      // RCLCPP_INFO(get_logger(), "dz: %f", target_data.dz);

    if(target_data.tracking){

    //进入击打策略，计算最后云台yaw和pitch
    Strategy::Gimbal_data gimbal_ = strategy_->calculate_gimbal(target_data,shoot_speed,shooter_height);

    //发布云台yaw和pitch
    auto_aim_interfaces::msg::Gimbal gimbal_msg;
    gimbal_msg.gimbal_yaw = gimbal_.gimbal_yaw;
    gimbal_msg.gimbal_pitch = gimbal_.gimbal_pitch;
    gimbal_msg.fire= gimbal_.fire;
    gimbal_pub_->publish(gimbal_msg);
    //RCLCPP_INFO(get_logger(), "finish publishing:");

    }


  }
}

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_auto_aim::TargetSolveNode)
