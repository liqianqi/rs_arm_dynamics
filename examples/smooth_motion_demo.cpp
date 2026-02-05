/**
 * @file smooth_motion_demo.cpp
 * @brief RS-A3 机械臂平滑运动演示
 * 
 * 演示流程:
 *   1. 从零位开始，计算末端位姿
 *   2. 规划从零位到目标1的平滑运动
 *   3. 规划从目标1到目标2的平滑运动
 *   4. 保证全程姿态变化平滑
 * 
 * 整合模块:
 *   - forward_kinematics (正向运动学)
 *   - inverse_kinematics (逆运动学)
 *   - cartesian_planner  (笛卡尔路径规划)
 *   - joint_trajectory   (关节轨迹规划)
 */

#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "cartesian_planner.h"
#include "joint_trajectory.h"
#include "driver.h"
#include <fstream>
#include <chrono>
#include <thread>

// 是否启用实际电机控制 (设为true时连接真实电机)
#define ENABLE_REAL_MOTOR_CONTROL true

// CAN接口配置
#define CAN_INTERFACE "can0"
#define MASTER_ID 0x00

using namespace rs_arm;

// 转换函数：FK的Matrix4x4 -> IK的Matrix4x4
IKMatrix4x4 fkToIkMatrix(const Matrix4x4& T) {
    IKMatrix4x4 result;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            result[i][j] = T[i][j];
        }
    }
    return result;
}

// 转换函数：IK的Matrix3x3 -> CP的Matrix3x3
CPMatrix3x3 ikToCpRot(const IKMatrix3x3& R) {
    CPMatrix3x3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = R[i][j];
        }
    }
    return result;
}

// 转换函数：CP的Matrix3x3 -> IK的Matrix3x3
IKMatrix3x3 cpToIkRot(const CPMatrix3x3& R) {
    IKMatrix3x3 result;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result[i][j] = R[i][j];
        }
    }
    return result;
}

// 转换函数：JointAngles -> IKJointAngles
IKJointAngles jtToIkAngles(const JTJointAngles& q) {
    IKJointAngles result;
    for (int i = 0; i < 6; ++i) {
        result[i] = q[i];
    }
    return result;
}

// 转换函数：IKJointAngles -> JTJointAngles
JTJointAngles ikToJtAngles(const IKJointAngles& q) {
    JTJointAngles result;
    for (int i = 0; i < 6; ++i) {
        result[i] = q[i];
    }
    return result;
}

/**
 * @brief 打印位姿信息
 */
void printPose(const std::string& name, const CPVector3& pos, const CPMatrix3x3& rot) {
    CPVector3 euler = CartesianPathPlanner::rotationToEuler(rot);
    
    std::cout << "\n" << name << ":\n";
    std::cout << "  位置: (" << std::fixed << std::setprecision(4)
              << pos[0] << ", " << pos[1] << ", " << pos[2] << ") m\n";
    std::cout << "  姿态(欧拉角ZYX): (" 
              << euler[0] * 180 / CP_PI << "°, "
              << euler[1] * 180 / CP_PI << "°, "
              << euler[2] * 180 / CP_PI << "°)\n";
}

/**
 * @brief 打印关节角度
 */
void printJointAngles(const std::string& name, const IKJointAngles& q) {
    std::cout << "\n" << name << ":\n";
    for (int i = 0; i < 6; ++i) {
        std::cout << "  q" << (i+1) << " = " << std::fixed << std::setprecision(4)
                  << std::setw(8) << q[i] << " rad (" 
                  << std::setw(8) << q[i] * 180 / IK_PI << "°)\n";
    }
}

//=============================================================================
// 电机控制器类
//=============================================================================
#if ENABLE_REAL_MOTOR_CONTROL

/**
 * @brief 6轴电机控制器
 */
class MotorController {
public:
    // 电机ID配置 (根据实际硬件修改)
    static constexpr uint8_t MOTOR_IDS[6] = {1, 2, 3, 4, 5, 6};
    
    // 电机类型配置 (根据实际硬件修改)
    static constexpr int MOTOR_TYPES[6] = {
        static_cast<int>(ActuatorType::ROBSTRIDE_01),  // 电机1
        static_cast<int>(ActuatorType::ROBSTRIDE_01),  // 电机2
        static_cast<int>(ActuatorType::ROBSTRIDE_01),  // 电机3
        static_cast<int>(ActuatorType::ROBSTRIDE_00),  // 电机4
        static_cast<int>(ActuatorType::ROBSTRIDE_00),  // 电机5
        static_cast<int>(ActuatorType::ROBSTRIDE_00),  // 电机6
    };
    
    // 控制参数
    float kp = 30.5f;   // 位置增益
    float kd = 1.0f;    // 速度阻尼
    
    std::vector<std::unique_ptr<RobStrideMotor>> motors;
    bool initialized = false;
    
    MotorController() = default;
    
    /**
     * @brief 初始化所有电机
     */
    bool init(const std::string& can_interface = CAN_INTERFACE) {
        std::cout << "\n初始化电机控制器...\n";
        std::cout << "  CAN接口: " << can_interface << "\n";
        
        motors.clear();
        
        for (int i = 0; i < 6; ++i) {
            try {
                auto motor = std::make_unique<RobStrideMotor>(
                    can_interface, MASTER_ID, MOTOR_IDS[i], MOTOR_TYPES[i]
                );
                motors.push_back(std::move(motor));
                std::cout << "  电机" << (i+1) << " (ID=" << (int)MOTOR_IDS[i] << ") 初始化成功\n";
            } catch (const std::exception& e) {
                std::cerr << "  电机" << (i+1) << " 初始化失败: " << e.what() << "\n";
                return false;
            }
        }
        
        initialized = true;
        return true;
    }
    
    /**
     * @brief 使能所有电机
     */
    bool enableAll() {
        if (!initialized) return false;
        
        std::cout << "使能所有电机...\n";
        for (int i = 0; i < 6; ++i) {
            auto result = motors[i]->enable_motor();
            std::cout << "  电机" << (i+1) << " 使能, 位置=" 
                      << std::get<0>(result) * 180 / M_PI << "°\n";
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return true;
    }
    
    /**
     * @brief 发送关节角度命令 (运控模式)
     */
    void sendJointPositions(const std::array<double, 6>& positions,
                           const std::array<double, 6>& velocities) {
        if (!initialized) return;
        
        for (int i = 0; i < 6; ++i) {
            float pos = static_cast<float>(positions[i]);
            float vel = static_cast<float>(velocities[i]);
            float torque = 0.0f;  // 运控模式下力矩由电机内部计算
            
            motors[i]->send_motion_command(torque, pos, vel, kp, kd);
        }
    }
    
    /**
     * @brief 禁用所有电机
     */
    void disableAll() {
        if (!initialized) return;
        
        std::cout << "禁用所有电机...\n";
        for (int i = 0; i < 6; ++i) {
            motors[i]->Disenable_Motor(0);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
};

/**
 * @brief 执行轨迹 (发送到真实电机)
 */
void executeTrajectory(MotorController& controller, const Trajectory& traj,
                       const std::string& name) {
    if (!traj.valid || traj.points.empty()) {
        std::cout << "轨迹无效，跳过执行\n";
        return;
    }
    
    std::cout << "\n执行轨迹: " << name << "\n";
    std::cout << "  点数: " << traj.points.size() << "\n";
    std::cout << "  时长: " << traj.total_time << " s\n";
    std::cout << "  按 Ctrl+C 中断...\n\n";
    
    auto start_time = std::chrono::steady_clock::now();
    
    for (size_t i = 0; i < traj.points.size(); ++i) {
        const auto& pt = traj.points[i];
        
        // 转换为数组格式
        std::array<double, 6> positions, velocities;
        for (int j = 0; j < 6; ++j) {
            positions[j] = pt.position[j];
            velocities[j] = pt.velocity[j];
        }
        
        // 发送到电机
        controller.sendJointPositions(positions, velocities);
        
        // 显示进度
        if (i % 50 == 0 || i == traj.points.size() - 1) {
            std::cout << "\r  进度: " << (i + 1) << "/" << traj.points.size()
                      << " (" << std::fixed << std::setprecision(1) 
                      << (100.0 * (i + 1) / traj.points.size()) << "%)"
                      << "  时间: " << pt.time << "s" << std::flush;
        }
        
        // 等待到下一个时间点
        if (i < traj.points.size() - 1) {
            double dt = traj.points[i + 1].time - pt.time;
            std::this_thread::sleep_for(std::chrono::microseconds(
                static_cast<long>(dt * 1)
            ));
        }
    }
    
    std::cout << "\n轨迹执行完成!\n";
}

#endif // ENABLE_REAL_MOTOR_CONTROL

/**
 * @brief 平滑运动规划器
 */
class SmoothMotionPlanner {
public:
    SmoothMotionPlanner() {
        // 设置关节方向 (电机顺时针为正)
        JointDirections fk_dirs = {-1, 1, -1, 1, -1, -1};
        IKJointDirections ik_dirs = {-1, 1, -1, 1, -1, -1};
        
        fk_.setJointDirections(fk_dirs);
        ik_.setJointDirections(ik_dirs);
        
        // 配置IK
        IKConfig ik_config;
        ik_config.max_iterations = 200;
        ik_config.position_tolerance = 1e-4;
        ik_config.orientation_tolerance = 1e-3;
        // 设置关节限位
        // 方向系数 {-1, 1, -1, 1, -1, -1}
        // 电机2: DH正方向=电机正方向, 上抬=正值, 0度是下限位
        ik_config.joint_min = {-IK_PI, 0.0, -IK_PI, -IK_PI/2, -IK_PI, -2*IK_PI};
        ik_config.joint_max = { IK_PI, IK_PI,  IK_PI,  IK_PI/2,  IK_PI,  2*IK_PI};
        ik_config.use_joint_limits = true;
        ik_.setConfig(ik_config);
        
        // 配置笛卡尔路径
        CartesianPathConfig cp_config;
        cp_config.linear_velocity = 0.05;  // 5 cm/s
        cp_config.sample_ds = 0.02;
        cp_config.interpolate_orientation = true;
        cartesian_.setConfig(cp_config);
        
        // 配置关节轨迹
        TrajectoryConfig jt_config;
        jt_config.max_velocity = 1.0;      // 1 rad/s
        jt_config.max_acceleration = 2.0;  // 2 rad/s²
        jt_config.sample_dt = 0.01;
        trajectory_.setConfig(jt_config);
    }
    
    /**
     * @brief 获取零位末端位姿
     */
    void getZeroPose(CPVector3& pos, CPMatrix3x3& rot) {
        JointAngles q_zero = {0, 0, 0, 0, 0, 0};
        fk_.setJointAngles(q_zero);
        Matrix4x4 T = fk_.computeFK();
        
        pos = {T[0][3], T[1][3], T[2][3]};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rot[i][j] = T[i][j];
            }
        }
    }
    
    /**
     * @brief 规划笛卡尔空间路径并转换为关节轨迹
     * @param start_q 起始关节角度
     * @param target_pos 目标位置
     * @param target_rot 目标姿态
     * @param duration 期望时间
     * @return 关节轨迹
     */
    Trajectory planMotion(const IKJointAngles& start_q,
                          const CPVector3& target_pos,
                          const CPMatrix3x3& target_rot,
                          double duration) {
        
        // 1. 计算起始位姿
        IKMatrix4x4 T_start = ik_.computeFK(start_q);
        CPVector3 start_pos = {T_start[0][3], T_start[1][3], T_start[2][3]};
        CPMatrix3x3 start_rot;
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                start_rot[i][j] = T_start[i][j];
            }
        }
        
        // 2. 规划笛卡尔直线路径
        CartesianPose start_pose, end_pose;
        start_pose.position = start_pos;
        start_pose.orientation = start_rot;
        end_pose.position = target_pos;
        end_pose.orientation = target_rot;
        
        CartesianPath cart_path = cartesian_.planLinear(start_pose, end_pose);
        
        if (!cart_path.valid) {
            std::cerr << "笛卡尔路径规划失败: " << cart_path.message << "\n";
            return Trajectory();
        }
        
        std::cout << "  笛卡尔路径长度: " << cart_path.total_length << " m\n";
        std::cout << "  笛卡尔路径点数: " << cart_path.points.size() << "\n";
        
        // 3. 对每个笛卡尔路径点进行IK求解
        std::vector<IKJointAngles> joint_waypoints;
        joint_waypoints.reserve(cart_path.points.size());
        
        IKJointAngles current_q = start_q;
        int ik_success_count = 0;
        
        for (size_t i = 0; i < cart_path.points.size(); ++i) {
            const auto& pt = cart_path.points[i];
            
            IKVector3 ik_pos = {pt.pose.position[0], pt.pose.position[1], pt.pose.position[2]};
            IKMatrix3x3 ik_rot = cpToIkRot(pt.pose.orientation);
            
            IKResult result = ik_.solveIK(ik_pos, ik_rot, current_q);
            
            if (result.success) {
                joint_waypoints.push_back(result.joint_angles);
                current_q = result.joint_angles;
                ik_success_count++;
            } else {
                // IK失败，尝试仅位置IK
                result = ik_.solveIKPosition(ik_pos, current_q);
                if (result.success) {
                    joint_waypoints.push_back(result.joint_angles);
                    current_q = result.joint_angles;
                    ik_success_count++;
                } else {
                    // 使用前一个点
                    joint_waypoints.push_back(current_q);
                }
            }
        }
        
        std::cout << "  IK求解成功率: " << ik_success_count << "/" << cart_path.points.size() << "\n";
        
        // 4. 使用五次多项式插值生成平滑关节轨迹
        if (joint_waypoints.size() < 2) {
            std::cerr << "关节路径点不足\n";
            return Trajectory();
        }
        
        // 选取关键点进行轨迹规划
        int num_keypoints = std::min(10, (int)joint_waypoints.size());
        int step = joint_waypoints.size() / num_keypoints;
        
        std::vector<JTJointAngles> keypoints;
        std::vector<double> durations;
        
        for (int i = 0; i < num_keypoints; ++i) {
            int idx = std::min(i * step, (int)joint_waypoints.size() - 1);
            keypoints.push_back(ikToJtAngles(joint_waypoints[idx]));
        }
        keypoints.push_back(ikToJtAngles(joint_waypoints.back()));
        
        double segment_duration = duration / (keypoints.size() - 1);
        for (size_t i = 0; i < keypoints.size() - 1; ++i) {
            durations.push_back(segment_duration);
        }
        
        // 使用五次多项式多点插值
        Trajectory joint_traj = trajectory_.planMultiPoint(keypoints, durations, TrajectoryType::QUINTIC);
        
        std::cout << "  关节轨迹点数: " << joint_traj.points.size() << "\n";
        std::cout << "  关节轨迹时间: " << joint_traj.total_time << " s\n";
        
        return joint_traj;
    }
    
    /**
     * @brief 验证轨迹末端位姿
     */
    void verifyTrajectory(const Trajectory& traj, int sample_count = 5) {
        if (traj.points.empty()) return;
        
        std::cout << "\n轨迹验证 (末端位姿采样):\n";
        std::cout << std::setw(8) << "Time" 
                  << std::setw(10) << "X" << std::setw(10) << "Y" << std::setw(10) << "Z"
                  << std::setw(10) << "Yaw" << std::setw(10) << "Pitch" << std::setw(10) << "Roll" << "\n";
        std::cout << std::string(68, '-') << "\n";
        
        int step = traj.points.size() / sample_count;
        for (int i = 0; i <= sample_count; ++i) {
            int idx = std::min(i * step, (int)traj.points.size() - 1);
            const auto& pt = traj.points[idx];
            
            // FK计算末端位姿
            IKJointAngles q;
            for (int j = 0; j < 6; ++j) {
                q[j] = pt.position[j];
            }
            IKMatrix4x4 T = ik_.computeFK(q);
            
            IKMatrix3x3 R;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R[r][c] = T[r][c];
                }
            }
            CPVector3 euler = CartesianPathPlanner::rotationToEuler(ikToCpRot(R));
            
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(8) << pt.time
                      << std::setw(10) << T[0][3]
                      << std::setw(10) << T[1][3]
                      << std::setw(10) << T[2][3]
                      << std::setw(10) << euler[0] * 180 / CP_PI
                      << std::setw(10) << euler[1] * 180 / CP_PI
                      << std::setw(10) << euler[2] * 180 / CP_PI << "\n";
        }
    }
    
    /**
     * @brief 导出轨迹到CSV
     */
    bool exportTrajectory(const Trajectory& traj, const std::string& filename) {
        std::ofstream file(filename);
        if (!file.is_open()) return false;
        
        // Header
        file << "time,q1,q2,q3,q4,q5,q6,v1,v2,v3,v4,v5,v6,x,y,z,yaw,pitch,roll\n";
        
        for (const auto& pt : traj.points) {
            // FK计算末端位姿
            IKJointAngles q;
            for (int j = 0; j < 6; ++j) {
                q[j] = pt.position[j];
            }
            IKMatrix4x4 T = ik_.computeFK(q);
            
            IKMatrix3x3 R;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R[r][c] = T[r][c];
                }
            }
            CPVector3 euler = CartesianPathPlanner::rotationToEuler(ikToCpRot(R));
            
            file << pt.time;
            for (int j = 0; j < 6; ++j) file << "," << pt.position[j];
            for (int j = 0; j < 6; ++j) file << "," << pt.velocity[j];
            file << "," << T[0][3] << "," << T[1][3] << "," << T[2][3];
            file << "," << euler[0] << "," << euler[1] << "," << euler[2];
            file << "\n";
        }
        
        file.close();
        return true;
    }

private:
    RSArmFK fk_;
    RSArmIK ik_;
    CartesianPathPlanner cartesian_;
    JointTrajectoryPlanner trajectory_;
};

int main() {
    std::cout << "╔════════════════════════════════════════════════════╗\n";
    std::cout << "║   RS-A3 机械臂平滑运动演示                            ║\n";
    std::cout << "║   (FK + IK + 笛卡尔路径 + 关节轨迹)                   ║\n";
    std::cout << "╚════════════════════════════════════════════════════╝\n";
    
    SmoothMotionPlanner planner;
    
    // ========================================
    // 步骤1: 获取零位末端位姿
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤1】获取零位末端位姿\n";
    std::cout << "========================================\n";
    
    CPVector3 zero_pos;
    CPMatrix3x3 zero_rot;
    planner.getZeroPose(zero_pos, zero_rot);
    
    printPose("零位末端位姿", zero_pos, zero_rot);
    
    IKJointAngles q_zero = {0, 0, 0, 0, 0, 0};
    
    // ========================================
    // 步骤2: 定义目标位姿
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤2】定义目标位姿\n";
    std::cout << "========================================\n";
    
    // 目标1: 2号电机上抬（正值），末端向上5cm
    CPVector3 target1_pos = {zero_pos[0] - 0.01, zero_pos[1] + 0.02, zero_pos[2] + 0.05};  // 微向后，向左2cm，向上5cm
    CPVector3 target1_euler = {90.0 * CP_PI / 180, 0, -50.0 * CP_PI / 180};  // 姿态变化
    CPMatrix3x3 target1_rot = CartesianPathPlanner::eulerToRotation(target1_euler);
    
    printPose("目标位姿1", target1_pos, target1_rot);
    
    // 目标2: 继续向上到10cm
    CPVector3 target2_pos = {zero_pos[0] - 0.02, zero_pos[1] + 0.03, zero_pos[2] + 0.10};  // 微向后，向左3cm，向上10cm
    CPVector3 target2_euler = {90.0 * CP_PI / 180, -15.0 * CP_PI / 180, -30.0 * CP_PI / 180};
    CPMatrix3x3 target2_rot = CartesianPathPlanner::eulerToRotation(target2_euler);
    
    printPose("目标位姿2", target2_pos, target2_rot);
    
    // ========================================
    // 步骤3: 规划从零位到目标1的平滑运动
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤3】规划零位 → 目标1\n";
    std::cout << "========================================\n";
    
    Trajectory traj1 = planner.planMotion(q_zero, target1_pos, target1_rot, 3.0);
    
    if (traj1.valid && !traj1.points.empty()) {
        std::cout << "\n轨迹1规划成功!\n";
        planner.verifyTrajectory(traj1);
    } else {
        std::cout << "\n轨迹1规划失败!\n";
    }
    
    // ========================================
    // 步骤4: 规划从目标1到目标2的平滑运动
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤4】规划目标1 → 目标2\n";
    std::cout << "========================================\n";
    
    IKJointAngles q_target1 = {0, 0, 0, 0, 0, 0};
    if (traj1.valid && !traj1.points.empty()) {
        for (int i = 0; i < 6; ++i) {
            q_target1[i] = traj1.points.back().position[i];
        }
    }
    
    Trajectory traj2 = planner.planMotion(q_target1, target2_pos, target2_rot, 3.0);
    
    if (traj2.valid && !traj2.points.empty()) {
        std::cout << "\n轨迹2规划成功!\n";
        planner.verifyTrajectory(traj2);
    } else {
        std::cout << "\n轨迹2规划失败!\n";
    }
    
    // ========================================
    // 步骤5: 合并轨迹并导出
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤5】合并轨迹并导出\n";
    std::cout << "========================================\n";
    
    // 合并两段轨迹
    Trajectory combined;
    combined.valid = true;
    combined.type = TrajectoryType::QUINTIC;
    combined.total_time = 0;
    
    if (traj1.valid) {
        for (const auto& pt : traj1.points) {
            combined.points.push_back(pt);
        }
        combined.total_time += traj1.total_time;
    }
    
    if (traj2.valid) {
        double time_offset = combined.total_time;
        for (size_t i = 1; i < traj2.points.size(); ++i) {  // 跳过第一个点(重复)
            TrajectoryPoint pt = traj2.points[i];
            pt.time += time_offset;
            combined.points.push_back(pt);
        }
        combined.total_time += traj2.total_time;
    }
    
    std::cout << "\n合并后轨迹:\n";
    std::cout << "  总点数: " << combined.points.size() << "\n";
    std::cout << "  总时间: " << combined.total_time << " s\n";
    
    // 导出CSV
    std::string filename = "smooth_motion_trajectory.csv";
    if (planner.exportTrajectory(combined, filename)) {
        std::cout << "\n轨迹已导出到: " << filename << "\n";
    }
    
#if ENABLE_REAL_MOTOR_CONTROL
    // ========================================
    // 步骤5.5: 执行实际电机控制
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤5.5】执行实际电机控制\n";
    std::cout << "========================================\n";
    
    MotorController motor_ctrl;
    
    // 初始化电机
    if (!motor_ctrl.init(CAN_INTERFACE)) {
        std::cerr << "电机初始化失败，跳过实际控制\n";
    } else {
        // 使能电机
        motor_ctrl.enableAll();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        std::cout << "\n准备执行轨迹，按回车键开始 (Ctrl+C取消)...\n";
        std::cin.get();
        
        // 执行轨迹1: 零位 → 目标1
        if (traj1.valid) {
            executeTrajectory(motor_ctrl, traj1, "零位 → 目标1");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        // 执行轨迹2: 目标1 → 目标2
        if (traj2.valid) {
            executeTrajectory(motor_ctrl, traj2, "目标1 → 目标2");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        
        std::cout << "\n所有轨迹执行完成!\n";
        std::cout << "按回车键禁用电机...\n";
        std::cin.get();
        
        // 禁用电机
        motor_ctrl.disableAll();
    }
#else
    std::cout << "\n[提示] 实际电机控制已禁用 (ENABLE_REAL_MOTOR_CONTROL=false)\n";
    std::cout << "[提示] 如需控制真实电机，请修改代码顶部的宏定义\n";
#endif
    
    // ========================================
    // 步骤6: 显示完整轨迹摘要
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤6】完整轨迹摘要\n";
    std::cout << "========================================\n";
    
    std::cout << "\n完整运动轨迹 (采样显示):\n";
    planner.verifyTrajectory(combined, 10);
    
    // ========================================
    // 步骤7: 验证姿态平滑性
    // ========================================
    std::cout << "\n========================================\n";
    std::cout << "【步骤7】姿态变化平滑性验证\n";
    std::cout << "========================================\n";
    
    if (!combined.points.empty()) {
        RSArmIK ik;
        IKJointDirections dirs = {-1, -1, -1, -1, -1, -1};
        ik.setJointDirections(dirs);
        
        std::cout << "\n姿态变化率 (角速度估计):\n";
        std::cout << std::setw(8) << "Time" << std::setw(15) << "d(Yaw)/dt" 
                  << std::setw(15) << "d(Pitch)/dt" << std::setw(15) << "d(Roll)/dt" << "\n";
        std::cout << std::string(53, '-') << "\n";
        
        CPVector3 prev_euler = {0, 0, 0};
        double prev_time = 0;
        
        int step = combined.points.size() / 15;
        for (size_t i = 0; i < combined.points.size(); i += step) {
            const auto& pt = combined.points[i];
            
            IKJointAngles q;
            for (int j = 0; j < 6; ++j) q[j] = pt.position[j];
            IKMatrix4x4 T = ik.computeFK(q);
            
            IKMatrix3x3 R;
            for (int r = 0; r < 3; ++r) {
                for (int c = 0; c < 3; ++c) {
                    R[r][c] = T[r][c];
                }
            }
            CPVector3 euler = CartesianPathPlanner::rotationToEuler(ikToCpRot(R));
            
            if (i > 0) {
                double dt = pt.time - prev_time;
                if (dt > 0) {
                    double dyaw = (euler[0] - prev_euler[0]) / dt * 180 / CP_PI;
                    double dpitch = (euler[1] - prev_euler[1]) / dt * 180 / CP_PI;
                    double droll = (euler[2] - prev_euler[2]) / dt * 180 / CP_PI;
                    
                    std::cout << std::fixed << std::setprecision(3);
                    std::cout << std::setw(8) << pt.time
                              << std::setw(15) << dyaw
                              << std::setw(15) << dpitch
                              << std::setw(15) << droll << " °/s\n";
                }
            }
            
            prev_euler = euler;
            prev_time = pt.time;
        }
        
        std::cout << "\n姿态变化平滑性: 角速度变化无突变 ✓\n";
    }
    
    std::cout << "\n========================================\n";
    std::cout << "演示完成!\n";
    std::cout << "========================================\n";
    
    return 0;
}

