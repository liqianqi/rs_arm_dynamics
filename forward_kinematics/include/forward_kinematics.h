/**
 * @file forward_kinematics.h
 * @brief RS-A3 机械臂正向运动学 (Forward Kinematics) - 基于 Modified DH 参数
 */

#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <array>
#include <cmath>
#include <iostream>
#include <iomanip>

namespace rs_arm {

constexpr int NUM_JOINTS = 6;
constexpr double PI = 3.14159265358979323846;

using Matrix4x4 = std::array<std::array<double, 4>, 4>;
using JointAngles = std::array<double, NUM_JOINTS>;
using JointDirections = std::array<int, NUM_JOINTS>;

struct MDHParams {
    double theta_offset;
    double d;
    double a;
    double alpha;
};

class RSArmFK {
public:
    RSArmFK();
    
    void setJointAngles(const JointAngles& q);
    JointAngles getJointAngles() const;
    void updateJointAngle(int joint_index, double angle);
    void setJointDirections(const JointDirections& directions);
    JointDirections getJointDirections() const;
    
    Matrix4x4 computeFK();
    Matrix4x4 computeFKToJoint(int joint_index);
    std::array<double, 3> getEndEffectorPosition();
    std::array<std::array<double, 3>, 3> getEndEffectorOrientation();
    
    void printStatus() const;
    static void printMatrix(const Matrix4x4& T, const std::string& name = "T");

private:
    std::array<MDHParams, NUM_JOINTS> mdh_params_;
    JointDirections joint_directions_;
    JointAngles joint_angles_;
    std::array<Matrix4x4, NUM_JOINTS> T_matrices_;
    Matrix4x4 T_end_;
    bool fk_computed_;
    
    Matrix4x4 computeMDHTransform(int joint_index, double q);
    static Matrix4x4 matMul(const Matrix4x4& A, const Matrix4x4& B);
    static Matrix4x4 eye();
};

} // namespace rs_arm

#endif // FORWARD_KINEMATICS_H

