/**
 * @file joint_trajectory.h
 * @brief 关节空间轨迹规划
 * 
 * 功能：
 *   1. 五次多项式插值 (Quintic Polynomial)
 *   2. 三次多项式插值 (Cubic Polynomial)
 *   3. 梯形速度曲线 (Trapezoidal Velocity Profile)
 *   4. S曲线速度规划 (S-Curve Velocity Profile)
 *   5. 多点轨迹插值
 */

#ifndef JOINT_TRAJECTORY_H
#define JOINT_TRAJECTORY_H

#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <algorithm>

namespace rs_arm {

constexpr int JT_NUM_JOINTS = 6;
constexpr double JT_PI = 3.14159265358979323846;

using JTJointAngles = std::array<double, JT_NUM_JOINTS>;
using JTJointVelocities = std::array<double, JT_NUM_JOINTS>;
using JTJointAccelerations = std::array<double, JT_NUM_JOINTS>;

/**
 * @brief 单个轨迹点
 */
struct TrajectoryPoint {
    double time;                    // 时间 (s)
    JTJointAngles position;         // 位置 (rad)
    JTJointVelocities velocity;     // 速度 (rad/s)
    JTJointAccelerations acceleration; // 加速度 (rad/s²)
};

/**
 * @brief 轨迹类型
 */
enum class TrajectoryType {
    CUBIC,      // 三次多项式
    QUINTIC,    // 五次多项式
    TRAPEZOIDAL,// 梯形速度
    SCURVE      // S曲线
};

/**
 * @brief 轨迹规划配置
 */
struct TrajectoryConfig {
    double max_velocity = 2.0;       // 最大速度 (rad/s)
    double max_acceleration = 5.0;   // 最大加速度 (rad/s²)
    double max_jerk = 20.0;          // 最大加加速度 (rad/s³) - S曲线用
    double sample_dt = 0.01;         // 采样时间间隔 (s)
};

/**
 * @brief 完整轨迹
 */
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double total_time;
    TrajectoryType type;
    bool valid;
    std::string message;
};

/**
 * @brief 关节轨迹规划器
 */
class JointTrajectoryPlanner {
public:
    JointTrajectoryPlanner();
    
    /**
     * @brief 设置配置
     */
    void setConfig(const TrajectoryConfig& config);
    TrajectoryConfig getConfig() const;
    
    /**
     * @brief 点到点轨迹 - 五次多项式
     * @param start 起始位置
     * @param end 终止位置
     * @param duration 持续时间 (s)
     * @param start_vel 起始速度 (默认0)
     * @param end_vel 终止速度 (默认0)
     * @return 轨迹
     */
    Trajectory planQuintic(const JTJointAngles& start, 
                           const JTJointAngles& end,
                           double duration,
                           const JTJointVelocities& start_vel = {},
                           const JTJointVelocities& end_vel = {});
    
    /**
     * @brief 点到点轨迹 - 三次多项式
     */
    Trajectory planCubic(const JTJointAngles& start, 
                         const JTJointAngles& end,
                         double duration,
                         const JTJointVelocities& start_vel = {},
                         const JTJointVelocities& end_vel = {});
    
    /**
     * @brief 点到点轨迹 - 梯形速度曲线
     * @param start 起始位置
     * @param end 终止位置
     * @return 轨迹 (时间自动计算)
     */
    Trajectory planTrapezoidal(const JTJointAngles& start, 
                               const JTJointAngles& end);
    
    /**
     * @brief 点到点轨迹 - S曲线
     */
    Trajectory planSCurve(const JTJointAngles& start, 
                          const JTJointAngles& end);
    
    /**
     * @brief 多点轨迹插值
     * @param waypoints 路径点列表
     * @param durations 每段时间
     * @param type 插值类型
     * @return 完整轨迹
     */
    Trajectory planMultiPoint(const std::vector<JTJointAngles>& waypoints,
                              const std::vector<double>& durations,
                              TrajectoryType type = TrajectoryType::QUINTIC);
    
    /**
     * @brief 在轨迹上插值获取某时刻的状态
     */
    TrajectoryPoint interpolate(const Trajectory& traj, double t);
    
    /**
     * @brief 打印轨迹信息
     */
    static void printTrajectory(const Trajectory& traj, int max_points = 10);
    
    /**
     * @brief 导出轨迹到 CSV
     */
    static bool exportToCSV(const Trajectory& traj, const std::string& filename);

private:
    TrajectoryConfig config_;
    
    // 五次多项式系数计算
    std::array<double, 6> computeQuinticCoeffs(double q0, double qf, double v0, double vf, 
                                                double a0, double af, double T);
    
    // 三次多项式系数计算
    std::array<double, 4> computeCubicCoeffs(double q0, double qf, double v0, double vf, double T);
    
    // 梯形速度规划单关节
    void planTrapezoidalSingle(double q0, double qf, double v_max, double a_max,
                               double& t_acc, double& t_const, double& t_dec, double& total_time);
    
    // S曲线规划单关节
    void planSCurveSingle(double q0, double qf, double v_max, double a_max, double j_max,
                          std::vector<double>& times, std::vector<double>& phases);
};

} // namespace rs_arm

#endif // JOINT_TRAJECTORY_H

