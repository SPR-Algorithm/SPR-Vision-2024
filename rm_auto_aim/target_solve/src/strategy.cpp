// Copyright 2022 Chen Jun

#include "target_solve/strategy.hpp"
#include <rclcpp/logger.hpp>

#include <angles/angles.h>
// STD
#include <cfloat>
#include <memory>
#include <string>
#include <vector>
#include "target_solve/strategy.hpp"

namespace rm_auto_aim
{

double Strategy::equation(float theta, float h, float distance, float  shoot_speed) {
    double cos_theta = cos(theta);
    return tan(theta) * distance - h - 9.8 * distance * distance / (2 * shoot_speed * shoot_speed * cos_theta * cos_theta);
}

double Strategy::solve_pitch(float h, float distance,float shoot_speed) {
    // printf(" h: %f",h);
    // printf("\n");
    // printf(" diatance: %f",distance);
    // printf("\n");
    // printf(" speed: %f",shoot_speed);
    // printf("\n");
    double a = -M_PI / 4; // 初始区间的下界
    double b = M_PI / 4; // 初始区间的上界，这里假设解不会超过90度
    double epsilon = 1e-3; // 定义求解的精度
    double f_a = equation(a, h, distance, shoot_speed);
    double f_b = equation(b, h, distance, shoot_speed);

    // 检查二分法的前提条件是否满足
    if (f_a * f_b > 0) {
        //std::cerr << "二分法区间选择错误，函数在区间端点应具有相反符号。" << std::endl;
        return -1; // 或者其他错误处理
    }

    double c;
    while ((b - a) / 2 > epsilon) {
        c = (a + b) / 2;
        double f_c = equation(c, h, distance, shoot_speed);

        if (f_c == 0) {
            // 找到了确切的零点
            break;
        } else if (f_a * f_c < 0) {
            // 零点在左半区间
            b = c;
            f_b = f_c;
        } else {
            // 零点在右半区间
            a = c;
            f_a = f_c;
        }
    }

    // a 和 b 的平均值作为解
    return (a + b) / 2;
    
}

//注意，之前tf完成的坐标变换z方向已经变换到了枪管，所以目标的h就是target的z坐标
Strategy::Gimbal_data Strategy::calculate_gimbal(const TargetData & target_data,float shoot_speed,float shooter_height){

    float distance =std::sqrt(target_data.x*target_data.x+target_data.y*target_data.y)-target_data.r1;
    float h=target_data.z;
    float theta = solve_pitch(h, distance,shoot_speed);
    printf(" theta: %f \n",theta);

    float dt = distance / (shoot_speed * cos(theta));
    dt=(dt)*1.5;//0.1是云台响应延迟，1.5是一个补偿系数

    printf(" dt: %f \n",dt);
    float dz =target_data.dz;

    //清空armors_data
    armors_data.clear();

    //首先计算车体中心dt后位置
    float center_x = target_data.x + target_data.vx * (dt+0.25);
    float center_y = target_data.y + target_data.vy * (dt+0.25);
    float center_z = target_data.z + dz / 2;
    float yaw = target_data.yaw + target_data.v_yaw * dt;
    printf("预测yaw预量: %f \n",float(yaw-target_data.yaw));

  if(target_data.armors_num ==4){//r1是当前装甲板的轴向半径，

    //计算四块装甲板的位置
    //target发布的
    ArmorData armor1;
    armor1.x = center_x - target_data.r1 * cos(yaw);
    armor1.y = center_y - target_data.r1 * sin(yaw);
    armor1.z = target_data.z;
    armor1.yaw = yaw;
    printf("armor1_yaw %f \n",armor1.yaw);

    //target顺时针旋转90度的
    ArmorData armor2;
    armor2.x = center_x - target_data.r2 * cos(yaw+M_PI/2);
    armor2.y = center_y - target_data.r2 * sin(yaw+M_PI/2);
    armor2.z = target_data.z + dz;//当前装甲是较低，dz>0，较高,dz<0
    armor2.yaw = yaw+M_PI/2;
    printf("armor2_yaw %f \n",armor2.yaw);

    //target逆时针旋转90度的
    ArmorData armor3;
    armor3.x = center_x - target_data.r2 * cos(yaw-M_PI/2);
    armor3.y = center_y - target_data.r2 * sin(yaw-M_PI/2);
    armor3.z = target_data.z + dz;//当前装甲是较低的，dz>0，较高的dz<0
    armor3.yaw = yaw-M_PI/2;
    printf("armor3_yaw %f \n",armor3.yaw);


    //target顺时针旋转180度的
    ArmorData armor4;
    armor4.x = center_x - target_data.r1 * cos(yaw+M_PI);
    armor4.y = center_y - target_data.r1 * sin(yaw+M_PI);
    armor4.z = target_data.z;
    armor4.yaw = yaw+M_PI;
    printf("armor4_yaw %f \n",armor4.yaw);

    //将四块装甲板的数据存入vector
    armors_data.push_back(armor1);
    armors_data.push_back(armor2);
    armors_data.push_back(armor3);
    armors_data.push_back(armor4);


  }

  else if (target_data.armors_num == 3)
  {
    // printf("armors_num %d",target_data.armors_num);
    // printf("\n");

    //计算三块装甲板的位置
    //target发布的
    ArmorData armor1;
    armor1.x = center_x - target_data.r1 * cos(yaw);
    armor1.y = center_y - target_data.r1 * sin(yaw);
    armor1.z = target_data.z;
    armor1.yaw = yaw;
    printf("armor1_yaw %f \n",armor1.yaw);

    //target顺时针旋转120度的
    ArmorData armor2;
    armor2.x = center_x - target_data.r1 * cos(yaw+M_PI*2/3);
    armor2.y = center_y - target_data.r1 * sin(yaw+M_PI*2/3);
    armor2.z = target_data.z + dz;//前哨战dz=0
    armor2.yaw = yaw+M_PI*2/3;
    printf("armor2_yaw %f \n",armor2.yaw);

    //target逆时针旋转120度的
    ArmorData armor3;
    armor3.x = center_x - target_data.r1 * cos(yaw-M_PI*2/3);
    armor3.y = center_y - target_data.r1 * sin(yaw-M_PI*2/3);
    armor3.z = target_data.z + dz;
    armor3.yaw = yaw-M_PI*2/3;
    printf("armor3_yaw %f \n",armor3.yaw);

    //将三块装甲板的数据存入vector
    armors_data.push_back(armor1);
    armors_data.push_back(armor2);
    armors_data.push_back(armor3);

  }

  else if (target_data.armors_num == 2)
  {
    // printf("armors_num %d",target_data.armors_num);
    // printf("\n");

    //计算三块装甲板的位置
    //target发布的
    ArmorData armor1;
    armor1.x = center_x - target_data.r1 * cos(yaw);
    armor1.y = center_y - target_data.r1 * sin(yaw);
    armor1.z = target_data.z;
    armor1.yaw = yaw;
    //printf("armor1_yaw %f \n",armor1.yaw);

    //target顺时针旋转180度的
    ArmorData armor2;
    armor2.x = center_x - target_data.r1 * cos(yaw+M_PI);
    armor2.y = center_y - target_data.r1 * sin(yaw+M_PI);
    armor2.z = target_data.z + dz;//当前装甲是较低的，dz>0，较高的dz<0
    armor2.yaw = yaw+M_PI;
    //printf("armor1_yaw %f \n",armor1.yaw);

    //将两块装甲板的数据存入vector
    armors_data.push_back(armor1);
    armors_data.push_back(armor2);

  }
  
  //依次判断每块装甲板的位置，选择最优的装甲板，此处区分英雄和其他兵种，英雄只考虑yaw角，追求击打准确度，其他兵种优先考虑距离，yaw角辅助判断，追求击杀效率
  
  Strategy::Gimbal_data gimbal_temp;//初始化云台角度

  //hero
  if(armors_data.size()==3){

    //表示没有可以击打的装甲板，返回一个默认值
    gimbal_temp.gimbal_yaw=atan2(center_y,center_x);
    //printf("centerz %f \n",center_z);
    gimbal_temp.gimbal_pitch=solve_pitch(center_z, center_x-target_data.r1,shoot_speed);//始终瞄准最正的一块装甲，瞄准的yaw和pitch是固定的
    gimbal_temp.fire=0;

    printf("云台yaw:%f \n",gimbal_temp.gimbal_yaw);
    printf("云台pitch:%f \n",gimbal_temp.gimbal_pitch);

    for (size_t i = 0; i < armors_data.size(); i++){
      //printf("容器大小 %lu \n",armors_data.size());
      //std::cout<<armors_data.size<<std::endl
      ArmorData armor_temp=armors_data[i];

      //判断yaw是否在可击打范围内
      if(armor_temp.yaw>-0.1 && armor_temp.yaw<0.1){

        gimbal_temp.fire=1;

      }
    }

    return gimbal_temp;

  }
  
  //others
  else{
    //首先计算odom系下车体中心的yaw角度
    //float center_yaw_odom = atan2(center_y,center_x);
    float min_diatance_ = 10.0;//10m
    size_t min_index = 0;
    //float yaw_diff = M_PI/2;

    for (size_t i = 0; i < armors_data.size(); i++){
        ArmorData armor_temp=armors_data[i];
        float armor_i_distance=std::sqrt(armor_temp.x*armor_temp.x+armor_temp.y*armor_temp.y);
        if (armor_i_distance < min_diatance_ ) {
          min_diatance_ = armor_i_distance;
          min_index = i;
        }
   }

    // for (size_t i = 0; i < armors_data.size(); i++){
    //     ArmorData armor_temp=armors_data[i];
    //     float armor_yaw_at_odom =atan2(armor_temp.y,armor_temp.x);//得到装甲板在odom系下的yaw
    //     float yaw_diff_tmp=fabs(armor_yaw_at_odom-center_yaw_odom);//得到yaw_diff
    //     if (yaw_diff_tmp < yaw_diff ) {//选择最小的yaw_diff
    //       yaw_diff = yaw_diff_tmp;
    //       min_index = i;
    //     }
    // }

    //赋值云台角度
    gimbal_temp.gimbal_yaw=atan2(armors_data[min_index].y,armors_data[min_index].x);

    printf("z轴高度 %f \n",armors_data[min_index].z);
    gimbal_temp.gimbal_pitch = solve_pitch(armors_data[min_index].z, std::sqrt(armors_data[min_index].x*armors_data[min_index].x+armors_data[min_index].y*armors_data[min_index].y),shoot_speed);

    //gimbal_temp.gimbal_pitch=gimbal_temp.gimbal_pitch-0.02;//pitch补偿
    //使用yaw角度，防止击打过于侧面的装甲板
    //printf("目标yaw朝向:%f \n",armors_data[min_index].yaw);
    
    if(std::fabs(gimbal_temp.gimbal_yaw-armors_data[min_index].yaw)<0.52){//此时云台角度已经瞄到目标预测位置，只需要判断yaw角度是否在云台+-30度范围内
   
    gimbal_temp.fire=1;
    }
    else{
      gimbal_temp.fire=0;
    }
    printf("云台yaw:%f \n",gimbal_temp.gimbal_yaw);
    printf("云台pitch:%f \n",gimbal_temp.gimbal_pitch);

  }

  return gimbal_temp;

}

}  // namespace rm_auto_aim


