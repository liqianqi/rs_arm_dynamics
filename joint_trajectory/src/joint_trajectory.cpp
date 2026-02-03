/**
 * @file joint_trajectory.cpp
 * @brief 关节空间轨迹规划实现
 */

#include "joint_trajectory.h"
#include <fstream>
#include <cstring>

namespace rs_arm {

JointTrajectoryPlanner::JointTrajectoryPlanner() {
    // 默认配置
}

void JointTrajectoryPlanner::setConfig(const TrajectoryConfig& config) {
    config_ = config;
}

TrajectoryConfig JointTrajectoryPlanner::getConfig() const {
    return config_;
}

std::array<double, 6> JointTrajectoryPlanner::computeQuinticCoeffs(
    double q0, double qf, double v0, double vf, double a0, double af, double T) {
    // 五次多项式: q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    // 边界条件: q(0)=q0, q(T)=qf, v(0)=v0, v(T)=vf, a(0)=a0, a(T)=af
    
    double T2 = T * T;
    double T3 = T2 * T;
    double T4 = T3 * T;
    double T5 = T4 * T;
    
    std::array<double, 6> coeffs;
    coeffs[0] = q0;
    coeffs[1] = v0;
    coeffs[2] = a0 / 2.0;
    coeffs[3] = (20*(qf - q0) - (8*vf + 12*v0)*T - (3*a0 - af)*T2) / (2*T3);
    coeffs[4] = (-30*(qf - q0) + (14*vf + 16*v0)*T + (3*a0 - 2*af)*T2) / (2*T4);
    coeffs[5] = (12*(qf - q0) - 6*(vf + v0)*T + (af - a0)*T2) / (2*T5);
    
    return coeffs;
}

std::array<double, 4> JointTrajectoryPlanner::computeCubicCoeffs(
    double q0, double qf, double v0, double vf, double T) {
    // 三次多项式: q(t) = a0 + a1*t + a2*t² + a3*t³
    
    double T2 = T * T;
    double T3 = T2 * T;
    
    std::array<double, 4> coeffs;
    coeffs[0] = q0;
    coeffs[1] = v0;
    coeffs[2] = (3*(qf - q0) - (2*v0 + vf)*T) / T2;
    coeffs[3] = (-2*(qf - q0) + (v0 + vf)*T) / T3;
    
    return coeffs;
}

Trajectory JointTrajectoryPlanner::planQuintic(
    const JTJointAngles& start, 
    const JTJointAngles& end,
    double duration,
    const JTJointVelocities& start_vel,
    const JTJointVelocities& end_vel) {
    
    Trajectory traj;
    traj.type = TrajectoryType::QUINTIC;
    traj.total_time = duration;
    traj.valid = true;
    traj.message = "五次多项式轨迹";
    
    if (duration <= 0) {
        traj.valid = false;
        traj.message = "持续时间必须大于0";
        return traj;
    }
    
    // 计算每个关节的系数
    std::array<std::array<double, 6>, JT_NUM_JOINTS> all_coeffs;
    for (int j = 0; j < JT_NUM_JOINTS; ++j) {
        all_coeffs[j] = computeQuinticCoeffs(
            start[j], end[j], start_vel[j], end_vel[j], 0.0, 0.0, duration);
    }
    
    // 生成轨迹点
    int num_points = static_cast<int>(duration / config_.sample_dt) + 1;
    traj.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double t = i * config_.sample_dt;
        if (t > duration) t = duration;
        
        TrajectoryPoint pt;
        pt.time = t;
        
        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t3 * t;
        double t5 = t4 * t;
        
        for (int j = 0; j < JT_NUM_JOINTS; ++j) {
            const auto& c = all_coeffs[j];
            pt.position[j] = c[0] + c[1]*t + c[2]*t2 + c[3]*t3 + c[4]*t4 + c[5]*t5;
            pt.velocity[j] = c[1] + 2*c[2]*t + 3*c[3]*t2 + 4*c[4]*t3 + 5*c[5]*t4;
            pt.acceleration[j] = 2*c[2] + 6*c[3]*t + 12*c[4]*t2 + 20*c[5]*t3;
        }
        
        traj.points.push_back(pt);
    }
    
    return traj;
}

Trajectory JointTrajectoryPlanner::planCubic(
    const JTJointAngles& start, 
    const JTJointAngles& end,
    double duration,
    const JTJointVelocities& start_vel,
    const JTJointVelocities& end_vel) {
    
    Trajectory traj;
    traj.type = TrajectoryType::CUBIC;
    traj.total_time = duration;
    traj.valid = true;
    traj.message = "三次多项式轨迹";
    
    if (duration <= 0) {
        traj.valid = false;
        traj.message = "持续时间必须大于0";
        return traj;
    }
    
    // 计算每个关节的系数
    std::array<std::array<double, 4>, JT_NUM_JOINTS> all_coeffs;
    for (int j = 0; j < JT_NUM_JOINTS; ++j) {
        all_coeffs[j] = computeCubicCoeffs(
            start[j], end[j], start_vel[j], end_vel[j], duration);
    }
    
    // 生成轨迹点
    int num_points = static_cast<int>(duration / config_.sample_dt) + 1;
    traj.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double t = i * config_.sample_dt;
        if (t > duration) t = duration;
        
        TrajectoryPoint pt;
        pt.time = t;
        
        double t2 = t * t;
        double t3 = t2 * t;
        
        for (int j = 0; j < JT_NUM_JOINTS; ++j) {
            const auto& c = all_coeffs[j];
            pt.position[j] = c[0] + c[1]*t + c[2]*t2 + c[3]*t3;
            pt.velocity[j] = c[1] + 2*c[2]*t + 3*c[3]*t2;
            pt.acceleration[j] = 2*c[2] + 6*c[3]*t;
        }
        
        traj.points.push_back(pt);
    }
    
    return traj;
}

void JointTrajectoryPlanner::planTrapezoidalSingle(
    double q0, double qf, double v_max, double a_max,
    double& t_acc, double& t_const, double& t_dec, double& total_time) {
    
    double dq = std::abs(qf - q0);
    
    // 检查是否能达到最大速度
    double t_acc_full = v_max / a_max;
    double d_acc = 0.5 * a_max * t_acc_full * t_acc_full;
    
    if (2 * d_acc >= dq) {
        // 三角形速度曲线（达不到最大速度）
        t_acc = std::sqrt(dq / a_max);
        t_const = 0;
        t_dec = t_acc;
    } else {
        // 梯形速度曲线
        t_acc = t_acc_full;
        t_const = (dq - 2 * d_acc) / v_max;
        t_dec = t_acc_full;
    }
    
    total_time = t_acc + t_const + t_dec;
}

Trajectory JointTrajectoryPlanner::planTrapezoidal(
    const JTJointAngles& start, 
    const JTJointAngles& end) {
    
    Trajectory traj;
    traj.type = TrajectoryType::TRAPEZOIDAL;
    traj.valid = true;
    traj.message = "梯形速度轨迹";
    
    // 计算每个关节的时间参数，取最长的
    double max_total_time = 0;
    std::array<double, JT_NUM_JOINTS> t_accs, t_consts, t_decs;
    
    for (int j = 0; j < JT_NUM_JOINTS; ++j) {
        double total_time;
        planTrapezoidalSingle(start[j], end[j], config_.max_velocity, config_.max_acceleration,
                              t_accs[j], t_consts[j], t_decs[j], total_time);
        max_total_time = std::max(max_total_time, total_time);
    }
    
    // 重新规划，使所有关节同步
    for (int j = 0; j < JT_NUM_JOINTS; ++j) {
        double dq = std::abs(end[j] - start[j]);
        if (dq < 1e-9) {
            t_accs[j] = t_consts[j] = t_decs[j] = 0;
            continue;
        }
        
        // 调整速度使时间同步
        double v_sync = dq / (max_total_time - config_.max_velocity / config_.max_acceleration);
        if (v_sync > config_.max_velocity) v_sync = config_.max_velocity;
        
        planTrapezoidalSingle(start[j], end[j], v_sync, config_.max_acceleration,
                              t_accs[j], t_consts[j], t_decs[j], max_total_time);
    }
    
    traj.total_time = max_total_time;
    
    // 生成轨迹点
    int num_points = static_cast<int>(max_total_time / config_.sample_dt) + 1;
    traj.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double t = i * config_.sample_dt;
        if (t > max_total_time) t = max_total_time;
        
        TrajectoryPoint pt;
        pt.time = t;
        
        for (int j = 0; j < JT_NUM_JOINTS; ++j) {
            double dq = end[j] - start[j];
            double sign = (dq >= 0) ? 1.0 : -1.0;
            double a = sign * config_.max_acceleration;
            
            double t_acc = t_accs[j];
            double t_const = t_consts[j];
            double t_dec = t_decs[j];
            
            if (std::abs(dq) < 1e-9) {
                pt.position[j] = start[j];
                pt.velocity[j] = 0;
                pt.acceleration[j] = 0;
            } else if (t <= t_acc) {
                // 加速阶段
                pt.position[j] = start[j] + 0.5 * a * t * t;
                pt.velocity[j] = a * t;
                pt.acceleration[j] = a;
            } else if (t <= t_acc + t_const) {
                // 匀速阶段
                double v = a * t_acc;
                double dt = t - t_acc;
                pt.position[j] = start[j] + 0.5 * a * t_acc * t_acc + v * dt;
                pt.velocity[j] = v;
                pt.acceleration[j] = 0;
            } else {
                // 减速阶段
                double v = a * t_acc;
                double dt = t - t_acc - t_const;
                pt.position[j] = start[j] + 0.5 * a * t_acc * t_acc + v * t_const 
                                + v * dt - 0.5 * a * dt * dt;
                pt.velocity[j] = v - a * dt;
                pt.acceleration[j] = -a;
            }
        }
        
        traj.points.push_back(pt);
    }
    
    return traj;
}

Trajectory JointTrajectoryPlanner::planSCurve(
    const JTJointAngles& start, 
    const JTJointAngles& end) {
    
    Trajectory traj;
    traj.type = TrajectoryType::SCURVE;
    traj.valid = true;
    traj.message = "S曲线轨迹";
    
    // S曲线有7个阶段：加加速、匀加速、减加速、匀速、加减速、匀减速、减减速
    // 简化实现：使用正弦函数平滑加速度
    
    // 先用梯形规划获得时间
    double max_total_time = 0;
    for (int j = 0; j < JT_NUM_JOINTS; ++j) {
        double dq = std::abs(end[j] - start[j]);
        if (dq < 1e-9) continue;
        
        double t_acc, t_const, t_dec, total_time;
        planTrapezoidalSingle(start[j], end[j], config_.max_velocity, config_.max_acceleration,
                              t_acc, t_const, t_dec, total_time);
        max_total_time = std::max(max_total_time, total_time);
    }
    
    if (max_total_time < 1e-9) {
        max_total_time = config_.sample_dt;
    }
    
    traj.total_time = max_total_time;
    
    // 生成轨迹点 (使用余弦平滑)
    int num_points = static_cast<int>(max_total_time / config_.sample_dt) + 1;
    traj.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double t = i * config_.sample_dt;
        if (t > max_total_time) t = max_total_time;
        
        TrajectoryPoint pt;
        pt.time = t;
        
        // 使用余弦插值 (S曲线的简化形式)
        double s = (1 - std::cos(JT_PI * t / max_total_time)) / 2;  // 0 -> 1
        double ds = JT_PI / (2 * max_total_time) * std::sin(JT_PI * t / max_total_time);
        double dds = JT_PI * JT_PI / (2 * max_total_time * max_total_time) * 
                     std::cos(JT_PI * t / max_total_time);
        
        for (int j = 0; j < JT_NUM_JOINTS; ++j) {
            double dq = end[j] - start[j];
            pt.position[j] = start[j] + s * dq;
            pt.velocity[j] = ds * dq;
            pt.acceleration[j] = dds * dq;
        }
        
        traj.points.push_back(pt);
    }
    
    return traj;
}

Trajectory JointTrajectoryPlanner::planMultiPoint(
    const std::vector<JTJointAngles>& waypoints,
    const std::vector<double>& durations,
    TrajectoryType type) {
    
    Trajectory traj;
    traj.valid = false;
    
    if (waypoints.size() < 2) {
        traj.message = "至少需要2个路径点";
        return traj;
    }
    
    if (durations.size() != waypoints.size() - 1) {
        traj.message = "时间段数量必须等于路径点数量-1";
        return traj;
    }
    
    traj.valid = true;
    traj.type = type;
    traj.total_time = 0;
    traj.message = "多点轨迹";
    
    double current_time = 0;
    
    for (size_t seg = 0; seg < waypoints.size() - 1; ++seg) {
        Trajectory seg_traj;
        
        // 计算段内速度（平滑过渡）
        JTJointVelocities start_vel = {}, end_vel = {};
        
        if (seg > 0 && seg < waypoints.size() - 2) {
            // 中间点：使用平均速度
            for (int j = 0; j < JT_NUM_JOINTS; ++j) {
                double v_prev = (waypoints[seg][j] - waypoints[seg-1][j]) / durations[seg-1];
                double v_next = (waypoints[seg+1][j] - waypoints[seg][j]) / durations[seg];
                start_vel[j] = (v_prev + v_next) / 2;
            }
        }
        if (seg < waypoints.size() - 2) {
            for (int j = 0; j < JT_NUM_JOINTS; ++j) {
                double v_curr = (waypoints[seg+1][j] - waypoints[seg][j]) / durations[seg];
                double v_next = (waypoints[seg+2][j] - waypoints[seg+1][j]) / durations[seg+1];
                end_vel[j] = (v_curr + v_next) / 2;
            }
        }
        
        switch (type) {
            case TrajectoryType::QUINTIC:
                seg_traj = planQuintic(waypoints[seg], waypoints[seg+1], durations[seg],
                                       start_vel, end_vel);
                break;
            case TrajectoryType::CUBIC:
                seg_traj = planCubic(waypoints[seg], waypoints[seg+1], durations[seg],
                                     start_vel, end_vel);
                break;
            default:
                seg_traj = planQuintic(waypoints[seg], waypoints[seg+1], durations[seg],
                                       start_vel, end_vel);
        }
        
        // 合并轨迹
        for (auto& pt : seg_traj.points) {
            if (seg > 0 && pt.time == 0) continue;  // 避免重复第一个点
            pt.time += current_time;
            traj.points.push_back(pt);
        }
        
        current_time += durations[seg];
    }
    
    traj.total_time = current_time;
    return traj;
}

TrajectoryPoint JointTrajectoryPlanner::interpolate(const Trajectory& traj, double t) {
    TrajectoryPoint pt;
    pt.time = t;
    
    if (traj.points.empty()) {
        pt.position.fill(0);
        pt.velocity.fill(0);
        pt.acceleration.fill(0);
        return pt;
    }
    
    // 找到时间区间
    if (t <= traj.points.front().time) {
        return traj.points.front();
    }
    if (t >= traj.points.back().time) {
        return traj.points.back();
    }
    
    // 二分查找
    size_t low = 0, high = traj.points.size() - 1;
    while (low < high - 1) {
        size_t mid = (low + high) / 2;
        if (traj.points[mid].time <= t) {
            low = mid;
        } else {
            high = mid;
        }
    }
    
    // 线性插值
    const auto& p0 = traj.points[low];
    const auto& p1 = traj.points[high];
    double alpha = (t - p0.time) / (p1.time - p0.time);
    
    pt.time = t;
    for (int j = 0; j < JT_NUM_JOINTS; ++j) {
        pt.position[j] = p0.position[j] + alpha * (p1.position[j] - p0.position[j]);
        pt.velocity[j] = p0.velocity[j] + alpha * (p1.velocity[j] - p0.velocity[j]);
        pt.acceleration[j] = p0.acceleration[j] + alpha * (p1.acceleration[j] - p0.acceleration[j]);
    }
    
    return pt;
}

void JointTrajectoryPlanner::printTrajectory(const Trajectory& traj, int max_points) {
    std::cout << "\n===== 轨迹信息 =====\n";
    std::cout << "类型: ";
    switch (traj.type) {
        case TrajectoryType::CUBIC: std::cout << "三次多项式"; break;
        case TrajectoryType::QUINTIC: std::cout << "五次多项式"; break;
        case TrajectoryType::TRAPEZOIDAL: std::cout << "梯形速度"; break;
        case TrajectoryType::SCURVE: std::cout << "S曲线"; break;
    }
    std::cout << "\n";
    std::cout << "状态: " << (traj.valid ? "有效" : "无效") << "\n";
    std::cout << "信息: " << traj.message << "\n";
    std::cout << "总时间: " << traj.total_time << " s\n";
    std::cout << "点数: " << traj.points.size() << "\n";
    
    if (!traj.points.empty()) {
        std::cout << "\n轨迹点 (显示前 " << std::min(max_points, (int)traj.points.size()) << " 个):\n";
        std::cout << std::setw(8) << "Time" 
                  << std::setw(10) << "q1" << std::setw(10) << "q2" << std::setw(10) << "q3"
                  << std::setw(10) << "v1" << std::setw(10) << "v2" << std::setw(10) << "v3" << "\n";
        std::cout << std::string(68, '-') << "\n";
        
        int step = std::max(1, (int)traj.points.size() / max_points);
        for (size_t i = 0; i < traj.points.size() && (int)i < max_points * step; i += step) {
            const auto& pt = traj.points[i];
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(8) << pt.time;
            for (int j = 0; j < 3; ++j) {
                std::cout << std::setw(10) << pt.position[j];
            }
            for (int j = 0; j < 3; ++j) {
                std::cout << std::setw(10) << pt.velocity[j];
            }
            std::cout << "\n";
        }
    }
    std::cout << "====================\n";
}

bool JointTrajectoryPlanner::exportToCSV(const Trajectory& traj, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }
    
    // Header
    file << "time";
    for (int j = 0; j < JT_NUM_JOINTS; ++j) {
        file << ",q" << (j+1) << ",v" << (j+1) << ",a" << (j+1);
    }
    file << "\n";
    
    // Data
    for (const auto& pt : traj.points) {
        file << pt.time;
        for (int j = 0; j < JT_NUM_JOINTS; ++j) {
            file << "," << pt.position[j] << "," << pt.velocity[j] << "," << pt.acceleration[j];
        }
        file << "\n";
    }
    
    file.close();
    return true;
}

} // namespace rs_arm

