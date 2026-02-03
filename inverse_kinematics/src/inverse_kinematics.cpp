/**
 * @file inverse_kinematics.cpp
 * @brief RS-A3 机械臂逆运动学实现
 */

#include "inverse_kinematics.h"

namespace rs_arm {

RSArmIK::RSArmIK() {
    // MDH 参数 (与 FK 一致)
    // Joint 1
    mdh_params_[0] = {0.0, 0.06400010, 0.0, 0.0};
    // Joint 2
    mdh_params_[1] = {0.0, 0.02590000, -0.01714870, IK_PI/2};
    // Joint 3
    mdh_params_[2] = {2.76108628, 0.0, 0.19000000, 0.0};
    // Joint 4
    mdh_params_[3] = {-2.76108628, -0.02590014, 0.16155494, 0.0};
    // Joint 5
    mdh_params_[4] = {-IK_PI/2, -0.00000003, -0.04920000, IK_PI/2};
    // Joint 6
    mdh_params_[5] = {0.0, -0.00805000, -0.00000003, IK_PI/2};
    
    // 默认方向系数: 电机顺时针为正
    joint_directions_.fill(-1);
}

void RSArmIK::setJointDirections(const IKJointDirections& directions) {
    joint_directions_ = directions;
}

IKJointDirections RSArmIK::getJointDirections() const {
    return joint_directions_;
}

void RSArmIK::setConfig(const IKConfig& config) {
    config_ = config;
}

IKConfig RSArmIK::getConfig() const {
    return config_;
}

IKMatrix4x4 RSArmIK::computeMDHTransform(int joint_index, double q_motor) {
    const auto& p = mdh_params_[joint_index];
    int dir = joint_directions_[joint_index];
    
    // 电机角度转 DH 角度
    double q_dh = dir * q_motor;
    double theta = p.theta_offset + q_dh;
    double d = p.d;
    double a = p.a;
    double alpha = p.alpha;
    
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);
    
    IKMatrix4x4 T = {{
        {{ ct,      -st,      0,       a     }},
        {{ st*ca,    ct*ca,  -sa,    -d*sa   }},
        {{ st*sa,    ct*sa,   ca,     d*ca   }},
        {{ 0,        0,       0,       1     }}
    }};
    
    return T;
}

IKMatrix4x4 RSArmIK::computeFK(const IKJointAngles& q) {
    IKMatrix4x4 T = eye4();
    for (int i = 0; i < IK_NUM_JOINTS; ++i) {
        T = matMul(T, computeMDHTransform(i, q[i]));
    }
    return T;
}

Jacobian RSArmIK::computeJacobian(const IKJointAngles& q) {
    Jacobian J;
    const double delta = 1e-6;  // 微分步长
    
    IKMatrix4x4 T0 = computeFK(q);
    IKVector3 pos0 = extractPosition(T0);
    IKMatrix3x3 rot0 = extractRotation(T0);
    IKVector3 euler0 = rotationToEulerZYX(rot0);
    
    for (int j = 0; j < IK_NUM_JOINTS; ++j) {
        IKJointAngles q_plus = q;
        q_plus[j] += delta;
        
        IKMatrix4x4 T_plus = computeFK(q_plus);
        IKVector3 pos_plus = extractPosition(T_plus);
        IKMatrix3x3 rot_plus = extractRotation(T_plus);
        IKVector3 euler_plus = rotationToEulerZYX(rot_plus);
        
        // 位置对 q[j] 的偏导数
        J[0][j] = (pos_plus[0] - pos0[0]) / delta;
        J[1][j] = (pos_plus[1] - pos0[1]) / delta;
        J[2][j] = (pos_plus[2] - pos0[2]) / delta;
        
        // 姿态对 q[j] 的偏导数
        J[3][j] = (euler_plus[0] - euler0[0]) / delta;
        J[4][j] = (euler_plus[1] - euler0[1]) / delta;
        J[5][j] = (euler_plus[2] - euler0[2]) / delta;
    }
    
    return J;
}

IKResult RSArmIK::solveIK(const IKVector3& target_pos, 
                          const IKMatrix3x3& target_rot,
                          const IKJointAngles& initial_guess) {
    IKResult result;
    result.joint_angles = initial_guess;
    result.iterations = 0;
    result.success = false;
    
    IKJointAngles q = initial_guess;
    
    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        result.iterations = iter + 1;
        
        // 计算当前 FK
        IKMatrix4x4 T_current = computeFK(q);
        IKVector3 pos_current = extractPosition(T_current);
        IKMatrix3x3 rot_current = extractRotation(T_current);
        
        // 计算误差
        IKVector3 pos_error = {
            target_pos[0] - pos_current[0],
            target_pos[1] - pos_current[1],
            target_pos[2] - pos_current[2]
        };
        
        IKVector3 rot_error = computeOrientationError(rot_current, target_rot);
        
        // 误差范数
        result.position_error = std::sqrt(
            pos_error[0]*pos_error[0] + 
            pos_error[1]*pos_error[1] + 
            pos_error[2]*pos_error[2]
        );
        result.orientation_error = std::sqrt(
            rot_error[0]*rot_error[0] + 
            rot_error[1]*rot_error[1] + 
            rot_error[2]*rot_error[2]
        );
        
        // 检查收敛
        if (result.position_error < config_.position_tolerance &&
            result.orientation_error < config_.orientation_tolerance) {
            result.success = true;
            result.joint_angles = q;
            result.message = "收敛成功";
            return result;
        }
        
        // 构建误差向量
        std::array<double, 6> error = {
            pos_error[0], pos_error[1], pos_error[2],
            rot_error[0], rot_error[1], rot_error[2]
        };
        
        // 计算雅可比
        Jacobian J = computeJacobian(q);
        
        // 阻尼最小二乘求解
        IKJointAngles dq = solveDLS(J, error, config_.damping_factor);
        
        // 更新关节角度
        for (int i = 0; i < IK_NUM_JOINTS; ++i) {
            q[i] += config_.step_size * dq[i];
        }
        
        // 限制关节角度
        if (config_.use_joint_limits) {
            clampJointAngles(q);
        }
    }
    
    result.joint_angles = q;
    result.message = "达到最大迭代次数";
    return result;
}

IKResult RSArmIK::solveIKPosition(const IKVector3& target_pos,
                                  const IKJointAngles& initial_guess) {
    IKResult result;
    result.joint_angles = initial_guess;
    result.iterations = 0;
    result.success = false;
    result.orientation_error = 0;
    
    IKJointAngles q = initial_guess;
    
    for (int iter = 0; iter < config_.max_iterations; ++iter) {
        result.iterations = iter + 1;
        
        // 计算当前 FK
        IKMatrix4x4 T_current = computeFK(q);
        IKVector3 pos_current = extractPosition(T_current);
        
        // 计算位置误差
        IKVector3 pos_error = {
            target_pos[0] - pos_current[0],
            target_pos[1] - pos_current[1],
            target_pos[2] - pos_current[2]
        };
        
        result.position_error = std::sqrt(
            pos_error[0]*pos_error[0] + 
            pos_error[1]*pos_error[1] + 
            pos_error[2]*pos_error[2]
        );
        
        // 检查收敛
        if (result.position_error < config_.position_tolerance) {
            result.success = true;
            result.joint_angles = q;
            result.message = "收敛成功 (仅位置)";
            return result;
        }
        
        // 计算雅可比 (仅位置部分 3x6)
        Jacobian J = computeJacobian(q);
        
        // 使用仅位置的 3x6 雅可比求解
        // J^T * (J * J^T + λ²I)^(-1) * error
        // 简化：使用梯度下降
        IKJointAngles dq;
        dq.fill(0);
        
        for (int j = 0; j < IK_NUM_JOINTS; ++j) {
            for (int i = 0; i < 3; ++i) {
                dq[j] += J[i][j] * pos_error[i];
            }
        }
        
        // 归一化步长
        double norm = 0;
        for (int j = 0; j < IK_NUM_JOINTS; ++j) {
            norm += dq[j] * dq[j];
        }
        norm = std::sqrt(norm);
        if (norm > 1e-6) {
            double scale = std::min(0.1, result.position_error) / norm;
            for (int j = 0; j < IK_NUM_JOINTS; ++j) {
                q[j] += scale * dq[j];
            }
        }
        
        if (config_.use_joint_limits) {
            clampJointAngles(q);
        }
    }
    
    result.joint_angles = q;
    result.message = "达到最大迭代次数";
    return result;
}

IKResult RSArmIK::solveIK(const IKMatrix4x4& target_T,
                          const IKJointAngles& initial_guess) {
    IKVector3 pos = extractPosition(target_T);
    IKMatrix3x3 rot = extractRotation(target_T);
    return solveIK(pos, rot, initial_guess);
}

IKJointAngles RSArmIK::solveDLS(const Jacobian& J, 
                                const std::array<double, 6>& error,
                                double damping) {
    // 阻尼最小二乘: dq = J^T * (J * J^T + λ²I)^(-1) * error
    // 简化实现：dq = J^T * error / (||J||² + λ²)
    
    IKJointAngles dq;
    dq.fill(0);
    
    // J^T * error
    for (int j = 0; j < IK_NUM_JOINTS; ++j) {
        for (int i = 0; i < 6; ++i) {
            dq[j] += J[i][j] * error[i];
        }
    }
    
    // 计算 ||J||² (Frobenius 范数的平方)
    double J_norm_sq = 0;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < IK_NUM_JOINTS; ++j) {
            J_norm_sq += J[i][j] * J[i][j];
        }
    }
    
    // 阻尼
    double scale = 1.0 / (J_norm_sq / 6.0 + damping * damping);
    for (int j = 0; j < IK_NUM_JOINTS; ++j) {
        dq[j] *= scale;
    }
    
    return dq;
}

void RSArmIK::clampJointAngles(IKJointAngles& q) {
    for (int i = 0; i < IK_NUM_JOINTS; ++i) {
        if (q[i] < config_.joint_min[i]) q[i] = config_.joint_min[i];
        if (q[i] > config_.joint_max[i]) q[i] = config_.joint_max[i];
    }
}

IKVector3 RSArmIK::computeOrientationError(const IKMatrix3x3& R_current, 
                                            const IKMatrix3x3& R_target) {
    // 计算旋转误差: R_error = R_target * R_current^T
    // 然后提取轴角表示
    
    // R_current^T
    IKMatrix3x3 R_current_T;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_current_T[i][j] = R_current[j][i];
        }
    }
    
    // R_error = R_target * R_current^T
    IKMatrix3x3 R_error;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R_error[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                R_error[i][j] += R_target[i][k] * R_current_T[k][j];
            }
        }
    }
    
    // 从旋转矩阵提取轴角
    auto [axis, angle] = rotationToAxisAngle(R_error);
    
    // 误差向量 = axis * angle
    return {axis[0] * angle, axis[1] * angle, axis[2] * angle};
}

std::pair<IKVector3, double> RSArmIK::rotationToAxisAngle(const IKMatrix3x3& R) {
    // 从旋转矩阵提取轴角
    double trace = R[0][0] + R[1][1] + R[2][2];
    double angle = std::acos(std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0)));
    
    IKVector3 axis = {0, 0, 1};
    
    if (std::abs(angle) < 1e-6) {
        // 无旋转
        return {axis, 0};
    } else if (std::abs(angle - IK_PI) < 1e-6) {
        // 180度旋转
        // 找最大对角元素
        if (R[0][0] >= R[1][1] && R[0][0] >= R[2][2]) {
            axis[0] = std::sqrt((R[0][0] + 1) / 2);
            axis[1] = R[0][1] / (2 * axis[0]);
            axis[2] = R[0][2] / (2 * axis[0]);
        } else if (R[1][1] >= R[2][2]) {
            axis[1] = std::sqrt((R[1][1] + 1) / 2);
            axis[0] = R[0][1] / (2 * axis[1]);
            axis[2] = R[1][2] / (2 * axis[1]);
        } else {
            axis[2] = std::sqrt((R[2][2] + 1) / 2);
            axis[0] = R[0][2] / (2 * axis[2]);
            axis[1] = R[1][2] / (2 * axis[2]);
        }
    } else {
        double s = 2 * std::sin(angle);
        axis[0] = (R[2][1] - R[1][2]) / s;
        axis[1] = (R[0][2] - R[2][0]) / s;
        axis[2] = (R[1][0] - R[0][1]) / s;
    }
    
    return {axis, angle};
}

IKVector3 RSArmIK::rotationToEulerZYX(const IKMatrix3x3& R) {
    // ZYX 欧拉角 (yaw, pitch, roll)
    IKVector3 euler;
    
    double sy = std::sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0]);
    
    if (sy > 1e-6) {
        euler[0] = std::atan2(R[1][0], R[0][0]);  // yaw (Z)
        euler[1] = std::atan2(-R[2][0], sy);       // pitch (Y)
        euler[2] = std::atan2(R[2][1], R[2][2]);   // roll (X)
    } else {
        euler[0] = std::atan2(-R[0][1], R[1][1]);
        euler[1] = std::atan2(-R[2][0], sy);
        euler[2] = 0;
    }
    
    return euler;
}

IKMatrix3x3 RSArmIK::eulerZYXToRotation(const IKVector3& euler) {
    double cz = std::cos(euler[0]);
    double sz = std::sin(euler[0]);
    double cy = std::cos(euler[1]);
    double sy = std::sin(euler[1]);
    double cx = std::cos(euler[2]);
    double sx = std::sin(euler[2]);
    
    IKMatrix3x3 R = {{
        {{ cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx }},
        {{ sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx }},
        {{ -sy,   cy*sx,            cy*cx            }}
    }};
    
    return R;
}

IKMatrix4x4 RSArmIK::matMul(const IKMatrix4x4& A, const IKMatrix4x4& B) {
    IKMatrix4x4 C;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            C[i][j] = 0;
            for (int k = 0; k < 4; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return C;
}

IKMatrix4x4 RSArmIK::eye4() {
    return {{
        {{1, 0, 0, 0}},
        {{0, 1, 0, 0}},
        {{0, 0, 1, 0}},
        {{0, 0, 0, 1}}
    }};
}

IKMatrix3x3 RSArmIK::eye3() {
    return {{
        {{1, 0, 0}},
        {{0, 1, 0}},
        {{0, 0, 1}}
    }};
}

IKMatrix3x3 RSArmIK::extractRotation(const IKMatrix4x4& T) {
    IKMatrix3x3 R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R[i][j] = T[i][j];
        }
    }
    return R;
}

IKVector3 RSArmIK::extractPosition(const IKMatrix4x4& T) {
    return {T[0][3], T[1][3], T[2][3]};
}

void RSArmIK::printResult(const IKResult& result) {
    std::cout << "\n===== IK 求解结果 =====\n";
    std::cout << "状态: " << (result.success ? "成功 ✓" : "失败 ✗") << "\n";
    std::cout << "信息: " << result.message << "\n";
    std::cout << "迭代次数: " << result.iterations << "\n";
    std::cout << "位置误差: " << std::scientific << std::setprecision(4) 
              << result.position_error << " m\n";
    std::cout << "姿态误差: " << result.orientation_error << " rad\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "关节角度 (rad):\n";
    for (int i = 0; i < IK_NUM_JOINTS; ++i) {
        std::cout << "  q" << (i+1) << " = " << std::setw(8) << result.joint_angles[i]
                  << " rad (" << std::setw(8) << (result.joint_angles[i] * 180.0 / IK_PI) << "°)\n";
    }
    std::cout << "======================\n";
}

} // namespace rs_arm

