/**
 * @file trajectory_test.cpp
 * @brief 关节空间轨迹规划测试程序
 */

#include "joint_trajectory.h"
#include <chrono>

using namespace rs_arm;

/**
 * @brief 测试1: 五次多项式插值
 */
void test_quintic() {
    std::cout << "\n========================================\n";
    std::cout << "【测试1】五次多项式轨迹\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {0.5, 0.3, -0.2, 0.4, -0.1, 0.2};
    double duration = 2.0;
    
    std::cout << "\n起点: [";
    for (int j = 0; j < JT_NUM_JOINTS; ++j) std::cout << start[j] << (j < 5 ? ", " : "");
    std::cout << "] rad\n";
    
    std::cout << "终点: [";
    for (int j = 0; j < JT_NUM_JOINTS; ++j) std::cout << end[j] << (j < 5 ? ", " : "");
    std::cout << "] rad\n";
    
    std::cout << "持续时间: " << duration << " s\n";
    
    auto start_time = std::chrono::high_resolution_clock::now();
    Trajectory traj = planner.planQuintic(start, end, duration);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double time_us = std::chrono::duration<double, std::micro>(end_time - start_time).count();
    std::cout << "\n规划耗时: " << time_us << " μs\n";
    
    JointTrajectoryPlanner::printTrajectory(traj, 10);
    
    // 验证边界条件
    std::cout << "\n边界条件验证:\n";
    std::cout << "t=0: 位置=[";
    for (int j = 0; j < 3; ++j) std::cout << std::fixed << std::setprecision(4) << traj.points.front().position[j] << (j < 2 ? ", " : "");
    std::cout << "], 速度=[";
    for (int j = 0; j < 3; ++j) std::cout << traj.points.front().velocity[j] << (j < 2 ? ", " : "");
    std::cout << "]\n";
    
    std::cout << "t=T: 位置=[";
    for (int j = 0; j < 3; ++j) std::cout << traj.points.back().position[j] << (j < 2 ? ", " : "");
    std::cout << "], 速度=[";
    for (int j = 0; j < 3; ++j) std::cout << traj.points.back().velocity[j] << (j < 2 ? ", " : "");
    std::cout << "]\n";
}

/**
 * @brief 测试2: 三次多项式插值
 */
void test_cubic() {
    std::cout << "\n========================================\n";
    std::cout << "【测试2】三次多项式轨迹\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {1.0, -0.5, 0.3, 0, 0, 0};
    double duration = 1.5;
    
    Trajectory traj = planner.planCubic(start, end, duration);
    JointTrajectoryPlanner::printTrajectory(traj, 8);
}

/**
 * @brief 测试3: 梯形速度曲线
 */
void test_trapezoidal() {
    std::cout << "\n========================================\n";
    std::cout << "【测试3】梯形速度曲线\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    // 设置速度和加速度限制
    TrajectoryConfig config;
    config.max_velocity = 1.5;      // rad/s
    config.max_acceleration = 3.0;  // rad/s²
    config.sample_dt = 0.02;
    planner.setConfig(config);
    
    std::cout << "\n配置:\n";
    std::cout << "  最大速度: " << config.max_velocity << " rad/s\n";
    std::cout << "  最大加速度: " << config.max_acceleration << " rad/s²\n";
    
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {1.57, 0.5, -0.3, 0.8, 0, 0};  // 约90°
    
    Trajectory traj = planner.planTrapezoidal(start, end);
    JointTrajectoryPlanner::printTrajectory(traj, 10);
    
    // 检查速度限制
    double max_v = 0;
    for (const auto& pt : traj.points) {
        for (int j = 0; j < JT_NUM_JOINTS; ++j) {
            max_v = std::max(max_v, std::abs(pt.velocity[j]));
        }
    }
    std::cout << "\n实际最大速度: " << max_v << " rad/s\n";
}

/**
 * @brief 测试4: S曲线
 */
void test_scurve() {
    std::cout << "\n========================================\n";
    std::cout << "【测试4】S曲线轨迹\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {0.8, 0.4, -0.2, 0.3, 0.1, 0};
    
    Trajectory traj = planner.planSCurve(start, end);
    JointTrajectoryPlanner::printTrajectory(traj, 10);
    
    // 检查加速度平滑性
    std::cout << "\n加速度平滑性检查 (关节1的加速度):\n";
    std::cout << "时间(s)    加速度(rad/s²)\n";
    for (size_t i = 0; i < traj.points.size(); i += traj.points.size() / 8) {
        std::cout << std::fixed << std::setprecision(3) 
                  << std::setw(8) << traj.points[i].time
                  << std::setw(15) << traj.points[i].acceleration[0] << "\n";
    }
}

/**
 * @brief 测试5: 多点轨迹
 */
void test_multipoint() {
    std::cout << "\n========================================\n";
    std::cout << "【测试5】多点轨迹插值\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    // 定义3个路径点
    std::vector<JTJointAngles> waypoints = {
        {0, 0, 0, 0, 0, 0},           // 起点
        {0.5, 0.3, -0.2, 0, 0, 0},    // 中间点1
        {0.3, 0.6, 0.1, 0.2, 0, 0},   // 中间点2
        {0, 0, 0, 0, 0, 0}            // 终点（回到起点）
    };
    
    std::vector<double> durations = {1.0, 1.5, 1.0};  // 每段时间
    
    std::cout << "\n路径点数: " << waypoints.size() << "\n";
    std::cout << "段数: " << durations.size() << "\n";
    std::cout << "每段时间: [";
    for (size_t i = 0; i < durations.size(); ++i) {
        std::cout << durations[i] << (i < durations.size()-1 ? ", " : "");
    }
    std::cout << "] s\n";
    
    Trajectory traj = planner.planMultiPoint(waypoints, durations, TrajectoryType::QUINTIC);
    JointTrajectoryPlanner::printTrajectory(traj, 12);
}

/**
 * @brief 测试6: 插值功能
 */
void test_interpolation() {
    std::cout << "\n========================================\n";
    std::cout << "【测试6】轨迹插值\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {1.0, 0.5, -0.3, 0, 0, 0};
    double duration = 2.0;
    
    Trajectory traj = planner.planQuintic(start, end, duration);
    
    std::cout << "\n在任意时刻插值:\n";
    std::cout << std::setw(8) << "时间" << std::setw(12) << "位置q1" << std::setw(12) << "速度v1" << "\n";
    std::cout << std::string(32, '-') << "\n";
    
    std::vector<double> times = {0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0};
    for (double t : times) {
        TrajectoryPoint pt = planner.interpolate(traj, t);
        std::cout << std::fixed << std::setprecision(3);
        std::cout << std::setw(8) << pt.time 
                  << std::setw(12) << pt.position[0]
                  << std::setw(12) << pt.velocity[0] << "\n";
    }
}

/**
 * @brief 测试7: 导出CSV
 */
void test_export_csv() {
    std::cout << "\n========================================\n";
    std::cout << "【测试7】导出CSV\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {1.0, 0.5, -0.3, 0.2, -0.1, 0.1};
    double duration = 2.0;
    
    Trajectory traj = planner.planQuintic(start, end, duration);
    
    std::string filename = "trajectory_output.csv";
    if (JointTrajectoryPlanner::exportToCSV(traj, filename)) {
        std::cout << "\n轨迹已导出到: " << filename << "\n";
        std::cout << "共 " << traj.points.size() << " 个数据点\n";
    } else {
        std::cout << "\n导出失败!\n";
    }
}

/**
 * @brief 测试8: 比较不同插值方法
 */
void test_comparison() {
    std::cout << "\n========================================\n";
    std::cout << "【测试8】插值方法比较\n";
    std::cout << "========================================\n";
    
    JointTrajectoryPlanner planner;
    
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {1.0, 0, 0, 0, 0, 0};  // 只移动第一个关节
    double duration = 1.0;
    
    Trajectory traj_cubic = planner.planCubic(start, end, duration);
    Trajectory traj_quintic = planner.planQuintic(start, end, duration);
    Trajectory traj_scurve = planner.planSCurve(start, end);
    
    std::cout << "\n关节1在 t=0.5s 时的状态:\n";
    std::cout << std::setw(15) << "方法" << std::setw(12) << "位置" 
              << std::setw(12) << "速度" << std::setw(12) << "加速度" << "\n";
    std::cout << std::string(51, '-') << "\n";
    
    TrajectoryPoint pt;
    
    pt = planner.interpolate(traj_cubic, 0.5);
    std::cout << std::setw(15) << "三次多项式" << std::fixed << std::setprecision(4)
              << std::setw(12) << pt.position[0]
              << std::setw(12) << pt.velocity[0]
              << std::setw(12) << pt.acceleration[0] << "\n";
    
    pt = planner.interpolate(traj_quintic, 0.5);
    std::cout << std::setw(15) << "五次多项式"
              << std::setw(12) << pt.position[0]
              << std::setw(12) << pt.velocity[0]
              << std::setw(12) << pt.acceleration[0] << "\n";
    
    pt = planner.interpolate(traj_scurve, traj_scurve.total_time / 2);
    std::cout << std::setw(15) << "S曲线"
              << std::setw(12) << pt.position[0]
              << std::setw(12) << pt.velocity[0]
              << std::setw(12) << pt.acceleration[0] << "\n";
    
    std::cout << "\n速度曲线特征:\n";
    std::cout << "  - 三次多项式: 线性速度变化\n";
    std::cout << "  - 五次多项式: 抛物线速度变化，边界加速度为0\n";
    std::cout << "  - S曲线: 正弦速度变化，最平滑\n";
}

int main() {
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║   关节空间轨迹规划测试程序             ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    
    test_quintic();
    test_cubic();
    test_trapezoidal();
    test_scurve();
    test_multipoint();
    test_interpolation();
    test_export_csv();
    test_comparison();
    
    std::cout << "\n========================================\n";
    std::cout << "所有测试完成!\n";
    std::cout << "========================================\n";
    
    return 0;
}

