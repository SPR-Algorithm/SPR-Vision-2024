// Copyright 2022 Chen Jun

#ifndef TARGET_SOLVE__STRATEGY_HPP_
#define TARGET_SOLVE__STRATEGY_HPP_


// ROS
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// STD
#include <memory>
#include <string>
#include <vector>


namespace rm_auto_aim
{
class Strategy
{
public:
  //定义一个储存装甲板xyz和yaw的结构体
  struct ArmorData {

    float x;
    float y; 
    float z;
    float yaw;

  };

  //定义一个vector储存所有装甲板的数据
  std::vector<ArmorData> armors_data;
  
  struct Gimbal_data {
      float gimbal_yaw;
      float gimbal_pitch;
      int fire;
  };

  struct TargetData {
    bool tracking;
    std::string id;
    int armors_num;
    float x, y, z;
    float yaw;
    float vx, vy, vz;
    float v_yaw;
    float r1, r2;
    float dz;
  };
  

  //使用牛顿迭代法求解当前子弹飞行时间
  double equation(float theta, float h, float distance,float shoot_speed);
  double derivative(float theta, float h, float distance, float shoot_speed);
  double solve_pitch(float h, float distance,float shoot_speed);



  Gimbal_data calculate_gimbal(const TargetData & target_data,float shoot_speed,float shooter_height);//接受tracker发布的target，进行装甲板选择，计算yaw和pitch角度，返回yaw和pitch角度


};

}  // namespace rm_auto_aim

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
