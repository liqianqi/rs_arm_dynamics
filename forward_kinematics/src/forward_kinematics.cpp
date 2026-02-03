/**
 * @file forward_kinematics.cpp
 * @brief RS-A3 机械臂正向运动学实现
 */

#include "forward_kinematics.h"

namespace rs_arm {

RSArmFK::RSArmFK() : fk_computed_(false) {
    // MDH 参数
    mdh_params_[0] = {0.0, 0.064, 0.0, 0.0};
    mdh_params_[1] = {0.0, 0.0259, -0.0171487, PI / 2.0};
    mdh_params_[2] = {2.76108628, 0.0, 0.19, 0.0};
    mdh_params_[3] = {-2.76108628, -0.02590014, 0.16155494, 0.0};
    mdh_params_[4] = {-PI / 2.0, 0.0, -0.0492, PI / 2.0};
    mdh_params_[5] = {0.0, -0.00805, 0.0, PI / 2.0};
    
    joint_directions_.fill(-1);  // 电机顺时针为正
    joint_angles_.fill(0.0);
    
    for (auto& T : T_matrices_) T = eye();
    T_end_ = eye();
}

void RSArmFK::setJointDirections(const JointDirections& directions) {
    joint_directions_ = directions;
    fk_computed_ = false;
}

JointDirections RSArmFK::getJointDirections() const { return joint_directions_; }

void RSArmFK::setJointAngles(const JointAngles& q) {
    joint_angles_ = q;
    fk_computed_ = false;
}

JointAngles RSArmFK::getJointAngles() const { return joint_angles_; }

void RSArmFK::updateJointAngle(int joint_index, double angle) {
    if (joint_index >= 0 && joint_index < NUM_JOINTS) {
        joint_angles_[joint_index] = angle;
        fk_computed_ = false;
    }
}

Matrix4x4 RSArmFK::computeMDHTransform(int joint_index, double q) {
    const auto& params = mdh_params_[joint_index];
    double q_dh = joint_directions_[joint_index] * q;
    double theta = params.theta_offset + q_dh;
    double d = params.d, a = params.a, alpha = params.alpha;
    
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);
    
    Matrix4x4 T;
    T[0][0] = ct;      T[0][1] = -st;     T[0][2] = 0;    T[0][3] = a;
    T[1][0] = st*ca;   T[1][1] = ct*ca;   T[1][2] = -sa;  T[1][3] = -d*sa;
    T[2][0] = st*sa;   T[2][1] = ct*sa;   T[2][2] = ca;   T[2][3] = d*ca;
    T[3][0] = 0;       T[3][1] = 0;       T[3][2] = 0;    T[3][3] = 1;
    return T;
}

Matrix4x4 RSArmFK::computeFK() {
    T_end_ = eye();
    for (int i = 0; i < NUM_JOINTS; ++i) {
        T_matrices_[i] = computeMDHTransform(i, joint_angles_[i]);
        T_end_ = matMul(T_end_, T_matrices_[i]);
    }
    fk_computed_ = true;
    return T_end_;
}

Matrix4x4 RSArmFK::computeFKToJoint(int joint_index) {
    if (joint_index < 0 || joint_index >= NUM_JOINTS) return eye();
    Matrix4x4 T = eye();
    for (int i = 0; i <= joint_index; ++i) {
        T = matMul(T, computeMDHTransform(i, joint_angles_[i]));
    }
    return T;
}

std::array<double, 3> RSArmFK::getEndEffectorPosition() {
    if (!fk_computed_) computeFK();
    return {T_end_[0][3], T_end_[1][3], T_end_[2][3]};
}

std::array<std::array<double, 3>, 3> RSArmFK::getEndEffectorOrientation() {
    if (!fk_computed_) computeFK();
    std::array<std::array<double, 3>, 3> R;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            R[i][j] = T_end_[i][j];
    return R;
}

void RSArmFK::printStatus() const {
    std::cout << "\n===== RS-A3 机械臂状态 =====\n";
    std::cout << "关节方向: [";
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::cout << (joint_directions_[i] > 0 ? "+1" : "-1");
        if (i < NUM_JOINTS - 1) std::cout << ", ";
    }
    std::cout << "]\n关节角度:\n";
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::cout << "  q" << (i+1) << " = " << std::fixed << std::setprecision(4)
                  << joint_angles_[i] << " rad (" << (joint_angles_[i]*180.0/PI) << "°)\n";
    }
}

void RSArmFK::printMatrix(const Matrix4x4& T, const std::string& name) {
    std::cout << name << " =\n";
    for (int i = 0; i < 4; ++i) {
        std::cout << "  [";
        for (int j = 0; j < 4; ++j) {
            std::cout << std::fixed << std::setprecision(6) << std::setw(12) << T[i][j];
            if (j < 3) std::cout << ", ";
        }
        std::cout << "]\n";
    }
    std::cout << "位置: (" << T[0][3] << ", " << T[1][3] << ", " << T[2][3] << ")\n";
}

Matrix4x4 RSArmFK::matMul(const Matrix4x4& A, const Matrix4x4& B) {
    Matrix4x4 C;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            C[i][j] = 0;
            for (int k = 0; k < 4; ++k) C[i][j] += A[i][k] * B[k][j];
        }
    return C;
}

Matrix4x4 RSArmFK::eye() {
    Matrix4x4 I;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            I[i][j] = (i == j) ? 1.0 : 0.0;
    return I;
}

} // namespace rs_arm

