/**
 * @file inverse_kinematics.h
 * @brief RS-A3 机械臂逆运动学 (Inverse Kinematics)
 * 
 * 实现方法：
 *   1. 数值解法 - 基于雅可比矩阵的阻尼最小二乘法 (DLS/Levenberg-Marquardt)
 *   2. 解析解法 - 基于几何关系的封闭解 (适用于满足 Pieper 条件的机械臂)
 */

#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <optional>
#include <vector>

namespace rs_arm {

constexpr int IK_NUM_JOINTS = 6;
constexpr double IK_PI = 3.14159265358979323846;

using IKMatrix4x4 = std::array<std::array<double, 4>, 4>;
using IKJointAngles = std::array<double, IK_NUM_JOINTS>;
using IKJointDirections = std::array<int, IK_NUM_JOINTS>;
using IKVector3 = std::array<double, 3>;
using IKMatrix3x3 = std::array<std::array<double, 3>, 3>;

// 雅可比矩阵: 6x6 (位置3行 + 姿态3行) x 6关节
using Jacobian = std::array<std::array<double, IK_NUM_JOINTS>, 6>;

/**
 * @brief IK 求解结果
 */
struct IKResult {
    bool success;                  // 是否成功
    IKJointAngles joint_angles;    // 关节角度
    int iterations;                // 迭代次数
    double position_error;         // 位置误差 (m)
    double orientation_error;      // 姿态误差 (rad)
    std::string message;           // 状态信息
};

/**
 * @brief IK 求解器配置
 */
struct IKConfig {
    int max_iterations = 100;      // 最大迭代次数
    double position_tolerance = 1e-4;    // 位置容差 (m)
    double orientation_tolerance = 1e-3; // 姿态容差 (rad)
    double damping_factor = 0.01;  // 阻尼因子 (DLS)
    double step_size = 1.0;        // 步长
    bool use_joint_limits = true;  // 是否使用关节限位
    
    // 关节限位 (rad)
    IKJointAngles joint_min = {-2*IK_PI, -IK_PI/2, -IK_PI, -2*IK_PI, -IK_PI/2, -2*IK_PI};
    IKJointAngles joint_max = { 2*IK_PI,  IK_PI/2,  IK_PI,  2*IK_PI,  IK_PI/2,  2*IK_PI};
};

/**
 * @brief MDH 参数
 */
struct IKMDHParams {
    double theta_offset;
    double d;
    double a;
    double alpha;
};

/**
 * @brief RS-A3 逆运动学求解器
 */
class RSArmIK {
public:
    RSArmIK();
    
    /**
     * @brief 设置关节方向系数
     * @param directions 方向系数数组 (1: 逆时针为正, -1: 顺时针为正)
     */
    void setJointDirections(const IKJointDirections& directions);
    IKJointDirections getJointDirections() const;
    
    /**
     * @brief 设置 IK 配置
     */
    void setConfig(const IKConfig& config);
    IKConfig getConfig() const;
    
    /**
     * @brief 数值法求解 IK - 给定目标位置和姿态
     * @param target_pos 目标位置 [x, y, z] (m)
     * @param target_rot 目标姿态旋转矩阵 3x3
     * @param initial_guess 初始猜测关节角度
     * @return IK 结果
     */
    IKResult solveIK(const IKVector3& target_pos, 
                     const IKMatrix3x3& target_rot,
                     const IKJointAngles& initial_guess);
    
    /**
     * @brief 数值法求解 IK - 仅位置 (忽略姿态)
     * @param target_pos 目标位置 [x, y, z] (m)
     * @param initial_guess 初始猜测关节角度
     * @return IK 结果
     */
    IKResult solveIKPosition(const IKVector3& target_pos,
                             const IKJointAngles& initial_guess);
    
    /**
     * @brief 数值法求解 IK - 给定 4x4 变换矩阵
     * @param target_T 目标变换矩阵 4x4
     * @param initial_guess 初始猜测关节角度
     * @return IK 结果
     */
    IKResult solveIK(const IKMatrix4x4& target_T,
                     const IKJointAngles& initial_guess);
    
    /**
     * @brief 计算正向运动学
     * @param q 关节角度 (电机角度，使用 joint_directions_ 转换)
     * @return 末端变换矩阵 4x4
     */
    IKMatrix4x4 computeFK(const IKJointAngles& q);
    
    /**
     * @brief 计算雅可比矩阵 (数值微分)
     * @param q 关节角度
     * @return 6x6 雅可比矩阵
     */
    Jacobian computeJacobian(const IKJointAngles& q);
    
    /**
     * @brief 从旋转矩阵提取欧拉角 (ZYX 顺序)
     */
    static IKVector3 rotationToEulerZYX(const IKMatrix3x3& R);
    
    /**
     * @brief 从欧拉角构建旋转矩阵 (ZYX 顺序)
     */
    static IKMatrix3x3 eulerZYXToRotation(const IKVector3& euler);
    
    /**
     * @brief 从旋转矩阵提取轴角表示
     */
    static std::pair<IKVector3, double> rotationToAxisAngle(const IKMatrix3x3& R);
    
    /**
     * @brief 打印 IK 结果
     */
    static void printResult(const IKResult& result);

private:
    std::array<IKMDHParams, IK_NUM_JOINTS> mdh_params_;
    IKJointDirections joint_directions_;
    IKConfig config_;
    
    // MDH 变换矩阵
    IKMatrix4x4 computeMDHTransform(int joint_index, double q_motor);
    
    // 矩阵运算
    static IKMatrix4x4 matMul(const IKMatrix4x4& A, const IKMatrix4x4& B);
    static IKMatrix4x4 eye4();
    static IKMatrix3x3 eye3();
    static IKMatrix3x3 extractRotation(const IKMatrix4x4& T);
    static IKVector3 extractPosition(const IKMatrix4x4& T);
    
    // 姿态误差计算 (使用轴角表示)
    static IKVector3 computeOrientationError(const IKMatrix3x3& R_current, 
                                              const IKMatrix3x3& R_target);
    
    // 限制关节角度到范围内
    void clampJointAngles(IKJointAngles& q);
    
    // 求解阻尼最小二乘
    IKJointAngles solveDLS(const Jacobian& J, 
                           const std::array<double, 6>& error,
                           double damping);
};

} // namespace rs_arm

#endif // INVERSE_KINEMATICS_H

