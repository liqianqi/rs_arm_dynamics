/**
 * @file ik_test.cpp
 * @brief RS-A3 机械臂逆运动学测试程序
 */

#include "inverse_kinematics.h"
#include <chrono>

using namespace rs_arm;

/**
 * @brief 测试1: FK -> IK 验证
 * 给定关节角度，先用 FK 计算末端位置，再用 IK 反解
 */
void test_fk_ik_roundtrip() {
    std::cout << "\n========================================\n";
    std::cout << "【测试1】FK -> IK 往返验证\n";
    std::cout << "========================================\n";
    
    RSArmIK ik;
    
    // 设置关节方向 (电机顺时针为正)
    IKJointDirections dirs = {-1, -1, -1, -1, -1, -1};
    ik.setJointDirections(dirs);
    
    // 原始关节角度
    IKJointAngles q_original = {0.5, 0.3, -0.2, 0.4, -0.1, 0.2};
    
    std::cout << "\n原始关节角度:\n";
    for (int i = 0; i < IK_NUM_JOINTS; ++i) {
        std::cout << "  q" << (i+1) << " = " << q_original[i] 
                  << " rad (" << (q_original[i] * 180.0 / IK_PI) << "°)\n";
    }
    
    // 正向运动学
    IKMatrix4x4 T_target = ik.computeFK(q_original);
    IKVector3 pos_target = {T_target[0][3], T_target[1][3], T_target[2][3]};
    
    std::cout << "\nFK 计算的末端位置:\n";
    std::cout << "  x = " << pos_target[0] << " m\n";
    std::cout << "  y = " << pos_target[1] << " m\n";
    std::cout << "  z = " << pos_target[2] << " m\n";
    
    // 逆运动学 (从零位开始)
    IKJointAngles initial_guess = {0, 0, 0, 0, 0, 0};
    
    auto start = std::chrono::high_resolution_clock::now();
    IKResult result = ik.solveIK(T_target, initial_guess);
    auto end = std::chrono::high_resolution_clock::now();
    
    double time_ms = std::chrono::duration<double, std::milli>(end - start).count();
    
    RSArmIK::printResult(result);
    std::cout << "计算耗时: " << time_ms << " ms\n";
    
    // 验证
    if (result.success) {
        IKMatrix4x4 T_verify = ik.computeFK(result.joint_angles);
        std::cout << "\n验证 - IK 解的末端位置:\n";
        std::cout << "  x = " << T_verify[0][3] << " m\n";
        std::cout << "  y = " << T_verify[1][3] << " m\n";
        std::cout << "  z = " << T_verify[2][3] << " m\n";
    }
}

/**
 * @brief 测试2: 指定目标位置
 */
void test_target_position() {
    std::cout << "\n========================================\n";
    std::cout << "【测试2】指定目标位置\n";
    std::cout << "========================================\n";
    
    RSArmIK ik;
    IKJointDirections dirs = {-1, -1, -1, -1, -1, -1};
    ik.setJointDirections(dirs);
    
    // 目标位置 (可达范围内)
    IKVector3 target_pos = {0.1, 0.05, 0.15};
    
    std::cout << "\n目标位置:\n";
    std::cout << "  x = " << target_pos[0] << " m\n";
    std::cout << "  y = " << target_pos[1] << " m\n";
    std::cout << "  z = " << target_pos[2] << " m\n";
    
    // 仅位置 IK
    IKJointAngles initial_guess = {0, 0, 0, 0, 0, 0};
    IKResult result = ik.solveIKPosition(target_pos, initial_guess);
    
    RSArmIK::printResult(result);
    
    if (result.success) {
        IKMatrix4x4 T_verify = ik.computeFK(result.joint_angles);
        std::cout << "验证位置: (" << T_verify[0][3] << ", " 
                  << T_verify[1][3] << ", " << T_verify[2][3] << ")\n";
    }
}

/**
 * @brief 测试3: 指定目标位姿
 */
void test_target_pose() {
    std::cout << "\n========================================\n";
    std::cout << "【测试3】指定目标位姿\n";
    std::cout << "========================================\n";
    
    RSArmIK ik;
    IKJointDirections dirs = {-1, -1, -1, -1, -1, -1};
    ik.setJointDirections(dirs);
    
    // 目标位置
    IKVector3 target_pos = {0.08, 0.02, 0.12};
    
    // 目标姿态 (末端朝下)
    IKMatrix3x3 target_rot = {{
        {{ 0,  0, -1}},
        {{ 1,  0,  0}},
        {{ 0, -1,  0}}
    }};
    
    std::cout << "\n目标位置: (" << target_pos[0] << ", " 
              << target_pos[1] << ", " << target_pos[2] << ")\n";
    std::cout << "目标姿态: 末端朝下\n";
    
    // 设置更宽松的收敛条件
    IKConfig config;
    config.max_iterations = 200;
    config.position_tolerance = 1e-3;
    config.orientation_tolerance = 1e-2;
    ik.setConfig(config);
    
    IKJointAngles initial_guess = {0, 0, 0, 0, 0, 0};
    IKResult result = ik.solveIK(target_pos, target_rot, initial_guess);
    
    RSArmIK::printResult(result);
}

/**
 * @brief 测试4: 从零位开始的轨迹
 */
void test_trajectory() {
    std::cout << "\n========================================\n";
    std::cout << "【测试4】轨迹跟踪测试\n";
    std::cout << "========================================\n";
    
    RSArmIK ik;
    IKJointDirections dirs = {-1, -1, -1, -1, -1, -1};
    ik.setJointDirections(dirs);
    
    // 零位的末端位置
    IKJointAngles q_zero = {0, 0, 0, 0, 0, 0};
    IKMatrix4x4 T_zero = ik.computeFK(q_zero);
    
    std::cout << "零位末端位置: (" << T_zero[0][3] << ", " 
              << T_zero[1][3] << ", " << T_zero[2][3] << ")\n";
    
    // 沿 X 轴移动轨迹
    std::cout << "\n沿 X 轴移动 (每步 0.01m):\n";
    std::cout << std::setw(6) << "Step" << std::setw(10) << "X_target"
              << std::setw(10) << "Success" << std::setw(10) << "Iters"
              << std::setw(15) << "Pos_Error" << "\n";
    std::cout << std::string(51, '-') << "\n";
    
    IKJointAngles q_current = q_zero;
    
    for (int step = 0; step < 5; ++step) {
        double x_offset = (step + 1) * 0.01;  // 0.01, 0.02, 0.03, 0.04, 0.05
        IKVector3 target_pos = {
            T_zero[0][3] + x_offset,
            T_zero[1][3],
            T_zero[2][3]
        };
        
        // 使用上一步的解作为初始猜测
        IKResult result = ik.solveIKPosition(target_pos, q_current);
        
        std::cout << std::setw(6) << (step + 1)
                  << std::setw(10) << std::fixed << std::setprecision(4) << target_pos[0]
                  << std::setw(10) << (result.success ? "OK" : "FAIL")
                  << std::setw(10) << result.iterations
                  << std::setw(15) << std::scientific << std::setprecision(2) << result.position_error
                  << "\n";
        
        if (result.success) {
            q_current = result.joint_angles;
        }
    }
}

/**
 * @brief 测试5: 工作空间边界测试
 */
void test_workspace_boundary() {
    std::cout << "\n========================================\n";
    std::cout << "【测试5】工作空间边界测试\n";
    std::cout << "========================================\n";
    
    RSArmIK ik;
    IKJointDirections dirs = {-1, -1, -1, -1, -1, -1};
    ik.setJointDirections(dirs);
    
    // 测试不同位置
    std::vector<IKVector3> test_positions = {
        {0.0, 0.0, 0.15},    // 正上方
        {0.1, 0.0, 0.1},     // 前方
        {0.0, 0.1, 0.1},     // 侧方
        {-0.05, 0.0, 0.1},   // 后方
        {0.5, 0.0, 0.0},     // 超出范围
    };
    
    std::cout << "\n测试不同目标位置:\n";
    std::cout << std::setw(20) << "Target" << std::setw(10) << "Result" 
              << std::setw(10) << "Iters" << std::setw(15) << "Pos_Error" << "\n";
    std::cout << std::string(55, '-') << "\n";
    
    for (const auto& pos : test_positions) {
        IKJointAngles initial_guess = {0, 0, 0, 0, 0, 0};
        IKResult result = ik.solveIKPosition(pos, initial_guess);
        
        std::cout << "(" << std::setw(5) << std::fixed << std::setprecision(2) << pos[0]
                  << "," << std::setw(5) << pos[1]
                  << "," << std::setw(5) << pos[2] << ")"
                  << std::setw(10) << (result.success ? "OK" : "FAIL")
                  << std::setw(10) << result.iterations
                  << std::setw(15) << std::scientific << std::setprecision(2) << result.position_error
                  << "\n";
    }
}

int main() {
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║   RS-A3 机械臂逆运动学测试程序         ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    
    test_fk_ik_roundtrip();
    test_target_position();
    test_target_pose();
    test_trajectory();
    test_workspace_boundary();
    
    std::cout << "\n========================================\n";
    std::cout << "所有测试完成!\n";
    std::cout << "========================================\n";
    
    return 0;
}

