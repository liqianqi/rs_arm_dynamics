/**
 * @file fk_test.cpp
 * @brief 正向运动学测试程序
 */

 #include "forward_kinematics.h"
 #include <iostream>
 
 using namespace rs_arm;
 
 double deg2rad(double deg) { return deg * PI / 180.0; }
 
 int main() {
     std::cout << "===== RS-A3 正向运动学测试 =====\n\n";
     
     RSArmFK robot;
     
     // 测试1: 零位
     std::cout << "【测试1】零位状态\n";
     robot.setJointAngles({0, 0, 0, 0, 0, 0});
     robot.printStatus();
     auto T = robot.computeFK();
     RSArmFK::printMatrix(T, "T_end");
     
     // 测试2: 设定角度
     std::cout << "\n【测试2】q1=30°, q2=45°\n";
     robot.setJointAngles({deg2rad(30), deg2rad(45), 0, 0, 0, 0});
     robot.printStatus();
     auto pos = robot.getEndEffectorPosition();
     std::cout << "末端位置: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")\n";
     
     // 测试3: 各关节坐标系
     std::cout << "\n【测试3】各关节坐标系位置\n";
     robot.setJointAngles({0, 0, 0, 0, 0, 0});
     for (int i = 0; i < NUM_JOINTS; ++i) {
         auto Ti = robot.computeFKToJoint(i);
         std::cout << "Joint " << (i+1) << ": (" 
                   << Ti[0][3] << ", " << Ti[1][3] << ", " << Ti[2][3] << ")\n";
     }
     
     std::cout << "\n===== 测试完成 =====\n";
     return 0;
 }
 
 