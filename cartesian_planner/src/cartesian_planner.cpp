/**
 * @file cartesian_planner.cpp
 * @brief 笛卡尔空间路径规划实现
 */

#include "cartesian_planner.h"

namespace rs_arm {

// ==================== CartesianPose ====================

CartesianPose CartesianPose::fromEuler(const CPVector3& pos, const CPVector3& euler_zyx) {
    CartesianPose pose;
    pose.position = pos;
    pose.orientation = CartesianPathPlanner::eulerToRotation(euler_zyx);
    return pose;
}

CPMatrix4x4 CartesianPose::toMatrix() const {
    CPMatrix4x4 T;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            T[i][j] = orientation[i][j];
        }
        T[i][3] = position[i];
    }
    T[3][0] = T[3][1] = T[3][2] = 0;
    T[3][3] = 1;
    return T;
}

CartesianPose CartesianPose::fromMatrix(const CPMatrix4x4& T) {
    CartesianPose pose;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            pose.orientation[i][j] = T[i][j];
        }
        pose.position[i] = T[i][3];
    }
    return pose;
}

// ==================== CartesianPathPlanner ====================

CartesianPathPlanner::CartesianPathPlanner() {
}

void CartesianPathPlanner::setConfig(const CartesianPathConfig& config) {
    config_ = config;
}

CartesianPathConfig CartesianPathPlanner::getConfig() const {
    return config_;
}

CPVector3 CartesianPathPlanner::cross(const CPVector3& a, const CPVector3& b) {
    return {
        a[1]*b[2] - a[2]*b[1],
        a[2]*b[0] - a[0]*b[2],
        a[0]*b[1] - a[1]*b[0]
    };
}

double CartesianPathPlanner::dot(const CPVector3& a, const CPVector3& b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

double CartesianPathPlanner::norm(const CPVector3& v) {
    return std::sqrt(dot(v, v));
}

CPVector3 CartesianPathPlanner::normalize(const CPVector3& v) {
    double n = norm(v);
    if (n < 1e-9) return {0, 0, 0};
    return {v[0]/n, v[1]/n, v[2]/n};
}

CPVector3 CartesianPathPlanner::subtract(const CPVector3& a, const CPVector3& b) {
    return {a[0]-b[0], a[1]-b[1], a[2]-b[2]};
}

CPVector3 CartesianPathPlanner::add(const CPVector3& a, const CPVector3& b) {
    return {a[0]+b[0], a[1]+b[1], a[2]+b[2]};
}

CPVector3 CartesianPathPlanner::scale(const CPVector3& v, double s) {
    return {v[0]*s, v[1]*s, v[2]*s};
}

CPMatrix3x3 CartesianPathPlanner::eye3() {
    return {{
        {{1, 0, 0}},
        {{0, 1, 0}},
        {{0, 0, 1}}
    }};
}

CPMatrix3x3 CartesianPathPlanner::eulerToRotation(const CPVector3& euler_zyx) {
    double cz = std::cos(euler_zyx[0]);
    double sz = std::sin(euler_zyx[0]);
    double cy = std::cos(euler_zyx[1]);
    double sy = std::sin(euler_zyx[1]);
    double cx = std::cos(euler_zyx[2]);
    double sx = std::sin(euler_zyx[2]);
    
    return {{
        {{ cz*cy, cz*sy*sx - sz*cx, cz*sy*cx + sz*sx }},
        {{ sz*cy, sz*sy*sx + cz*cx, sz*sy*cx - cz*sx }},
        {{ -sy,   cy*sx,            cy*cx            }}
    }};
}

CPVector3 CartesianPathPlanner::rotationToEuler(const CPMatrix3x3& R) {
    CPVector3 euler;
    double sy = std::sqrt(R[0][0]*R[0][0] + R[1][0]*R[1][0]);
    
    if (sy > 1e-6) {
        euler[0] = std::atan2(R[1][0], R[0][0]);
        euler[1] = std::atan2(-R[2][0], sy);
        euler[2] = std::atan2(R[2][1], R[2][2]);
    } else {
        euler[0] = std::atan2(-R[0][1], R[1][1]);
        euler[1] = std::atan2(-R[2][0], sy);
        euler[2] = 0;
    }
    
    return euler;
}

CPMatrix3x3 CartesianPathPlanner::slerp(const CPMatrix3x3& R1, const CPMatrix3x3& R2, double t) {
    // 简化的姿态插值（使用欧拉角线性插值）
    // 更精确的实现应使用四元数SLERP
    
    CPVector3 e1 = rotationToEuler(R1);
    CPVector3 e2 = rotationToEuler(R2);
    
    CPVector3 e_interp = {
        e1[0] + t * (e2[0] - e1[0]),
        e1[1] + t * (e2[1] - e1[1]),
        e1[2] + t * (e2[2] - e1[2])
    };
    
    return eulerToRotation(e_interp);
}

CartesianPath CartesianPathPlanner::planLinear(
    const CartesianPose& start, 
    const CartesianPose& end) {
    
    CartesianPath path;
    path.type = PathType::LINEAR;
    path.valid = true;
    path.message = "直线路径";
    
    // 计算路径长度
    CPVector3 diff = subtract(end.position, start.position);
    path.total_length = norm(diff);
    
    if (path.total_length < 1e-9) {
        path.total_length = 0;
        path.total_time = 0;
        CartesianPathPoint pt;
        pt.s = 0;
        pt.pose = start;
        pt.linear_velocity = {0, 0, 0};
        pt.angular_velocity = {0, 0, 0};
        path.points.push_back(pt);
        return path;
    }
    
    // 计算总时间
    path.total_time = path.total_length / config_.linear_velocity;
    
    // 方向向量
    CPVector3 direction = normalize(diff);
    
    // 生成路径点
    int num_points = static_cast<int>(1.0 / config_.sample_ds) + 1;
    path.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double s = i * config_.sample_ds;
        if (s > 1.0) s = 1.0;
        
        CartesianPathPoint pt;
        pt.s = s;
        
        // 位置插值
        pt.pose.position = add(start.position, scale(diff, s));
        
        // 姿态插值
        if (config_.interpolate_orientation) {
            pt.pose.orientation = slerp(start.orientation, end.orientation, s);
        } else {
            pt.pose.orientation = start.orientation;
        }
        
        // 速度
        pt.linear_velocity = scale(direction, config_.linear_velocity);
        pt.angular_velocity = {0, 0, 0};  // 简化
        
        path.points.push_back(pt);
    }
    
    return path;
}

CartesianPath CartesianPathPlanner::planLinearPosition(
    const CPVector3& start_pos, 
    const CPVector3& end_pos,
    const CPMatrix3x3& orientation) {
    
    CartesianPose start, end;
    start.position = start_pos;
    start.orientation = orientation;
    end.position = end_pos;
    end.orientation = orientation;
    
    return planLinear(start, end);
}

std::optional<CircularArcParams> CartesianPathPlanner::computeCircularParams(
    const CPVector3& p1, const CPVector3& p2, const CPVector3& p3) {
    
    // 通过三点计算圆弧参数
    CPVector3 v1 = subtract(p2, p1);
    CPVector3 v2 = subtract(p3, p2);
    
    // 法向量
    CPVector3 n = cross(v1, v2);
    double n_norm = norm(n);
    
    if (n_norm < 1e-9) {
        // 三点共线
        return std::nullopt;
    }
    
    n = scale(n, 1.0 / n_norm);
    
    // 中点
    CPVector3 m1 = scale(add(p1, p2), 0.5);
    CPVector3 m2 = scale(add(p2, p3), 0.5);
    
    // 中垂面法向量
    CPVector3 n1 = normalize(v1);
    CPVector3 n2 = normalize(v2);
    
    // 中垂线方向
    CPVector3 d1 = cross(n1, n);
    CPVector3 d2 = cross(n2, n);
    
    // 求圆心（两条中垂线的交点）
    // m1 + t1*d1 = m2 + t2*d2
    // 使用最小二乘求解
    
    CPVector3 dm = subtract(m2, m1);
    double d1d1 = dot(d1, d1);
    double d1d2 = dot(d1, d2);
    double d2d2 = dot(d2, d2);
    double d1dm = dot(d1, dm);
    double d2dm = dot(d2, dm);
    
    double denom = d1d1 * d2d2 - d1d2 * d1d2;
    if (std::abs(denom) < 1e-9) {
        return std::nullopt;
    }
    
    double t1 = (d2d2 * d1dm - d1d2 * d2dm) / denom;
    
    CircularArcParams params;
    params.center = add(m1, scale(d1, t1));
    params.normal = n;
    params.radius = norm(subtract(p1, params.center));
    
    // 计算角度
    CPVector3 r1 = subtract(p1, params.center);
    CPVector3 r3 = subtract(p3, params.center);
    
    // 起始角度（相对于圆心坐标系）
    params.start_angle = 0;
    
    // 终止角度
    double cos_angle = dot(r1, r3) / (norm(r1) * norm(r3));
    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
    params.end_angle = std::acos(cos_angle);
    
    // 判断方向
    CPVector3 cross_r = cross(r1, r3);
    if (dot(cross_r, n) < 0) {
        params.end_angle = -params.end_angle;
    }
    
    return params;
}

CartesianPath CartesianPathPlanner::planCircular(
    const CPVector3& start, 
    const CPVector3& via,
    const CPVector3& end,
    const CPMatrix3x3& orientation) {
    
    CartesianPath path;
    path.type = PathType::CIRCULAR;
    path.valid = true;
    path.message = "圆弧路径";
    
    auto params_opt = computeCircularParams(start, via, end);
    if (!params_opt) {
        path.valid = false;
        path.message = "无法计算圆弧参数（三点可能共线）";
        return path;
    }
    
    auto params = *params_opt;
    
    // 路径长度
    path.total_length = std::abs(params.end_angle) * params.radius;
    path.total_time = path.total_length / config_.linear_velocity;
    
    // 建立圆弧坐标系
    CPVector3 x_axis = normalize(subtract(start, params.center));
    CPVector3 z_axis = params.normal;
    CPVector3 y_axis = cross(z_axis, x_axis);
    
    // 生成路径点
    int num_points = static_cast<int>(1.0 / config_.sample_ds) + 1;
    path.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double s = i * config_.sample_ds;
        if (s > 1.0) s = 1.0;
        
        double angle = params.start_angle + s * (params.end_angle - params.start_angle);
        
        CartesianPathPoint pt;
        pt.s = s;
        
        // 圆弧上的位置
        CPVector3 local_pos = {
            params.radius * std::cos(angle),
            params.radius * std::sin(angle),
            0
        };
        
        pt.pose.position = {
            params.center[0] + x_axis[0]*local_pos[0] + y_axis[0]*local_pos[1],
            params.center[1] + x_axis[1]*local_pos[0] + y_axis[1]*local_pos[1],
            params.center[2] + x_axis[2]*local_pos[0] + y_axis[2]*local_pos[1]
        };
        
        pt.pose.orientation = orientation;
        
        // 切向速度
        double omega = params.end_angle / path.total_time;
        CPVector3 tangent = {
            -x_axis[0]*std::sin(angle) + y_axis[0]*std::cos(angle),
            -x_axis[1]*std::sin(angle) + y_axis[1]*std::cos(angle),
            -x_axis[2]*std::sin(angle) + y_axis[2]*std::cos(angle)
        };
        pt.linear_velocity = scale(tangent, params.radius * std::abs(omega));
        pt.angular_velocity = {0, 0, 0};
        
        path.points.push_back(pt);
    }
    
    return path;
}

CartesianPath CartesianPathPlanner::planCircularByAngle(
    const CPVector3& center,
    const CPVector3& start,
    double angle,
    const CPVector3& normal,
    const CPMatrix3x3& orientation) {
    
    CartesianPath path;
    path.type = PathType::CIRCULAR;
    path.valid = true;
    path.message = "圆弧路径（按角度）";
    
    CPVector3 n = normalize(normal);
    double radius = norm(subtract(start, center));
    
    if (radius < 1e-9) {
        path.valid = false;
        path.message = "半径过小";
        return path;
    }
    
    path.total_length = std::abs(angle) * radius;
    path.total_time = path.total_length / config_.linear_velocity;
    
    // 坐标系
    CPVector3 x_axis = normalize(subtract(start, center));
    CPVector3 y_axis = cross(n, x_axis);
    
    int num_points = static_cast<int>(1.0 / config_.sample_ds) + 1;
    path.points.reserve(num_points);
    
    for (int i = 0; i < num_points; ++i) {
        double s = i * config_.sample_ds;
        if (s > 1.0) s = 1.0;
        
        double theta = s * angle;
        
        CartesianPathPoint pt;
        pt.s = s;
        
        pt.pose.position = {
            center[0] + radius * (x_axis[0]*std::cos(theta) + y_axis[0]*std::sin(theta)),
            center[1] + radius * (x_axis[1]*std::cos(theta) + y_axis[1]*std::sin(theta)),
            center[2] + radius * (x_axis[2]*std::cos(theta) + y_axis[2]*std::sin(theta))
        };
        
        pt.pose.orientation = orientation;
        
        CPVector3 tangent = {
            -x_axis[0]*std::sin(theta) + y_axis[0]*std::cos(theta),
            -x_axis[1]*std::sin(theta) + y_axis[1]*std::cos(theta),
            -x_axis[2]*std::sin(theta) + y_axis[2]*std::cos(theta)
        };
        pt.linear_velocity = scale(tangent, config_.linear_velocity);
        pt.angular_velocity = {0, 0, 0};
        
        path.points.push_back(pt);
    }
    
    return path;
}

std::vector<CPVector3> CartesianPathPlanner::computeCubicSpline(
    const std::vector<CPVector3>& points, int num_samples) {
    
    std::vector<CPVector3> result;
    if (points.size() < 2) return result;
    
    int n = points.size() - 1;
    
    // 对每个坐标分量分别计算样条
    for (int seg = 0; seg < n; ++seg) {
        int samples_per_seg = num_samples / n;
        if (seg == n - 1) {
            samples_per_seg = num_samples - seg * (num_samples / n);
        }
        
        for (int i = 0; i < samples_per_seg; ++i) {
            double t = static_cast<double>(i) / samples_per_seg;
            
            // Catmull-Rom 样条
            CPVector3 p0 = (seg > 0) ? points[seg - 1] : points[seg];
            CPVector3 p1 = points[seg];
            CPVector3 p2 = points[seg + 1];
            CPVector3 p3 = (seg < n - 1) ? points[seg + 2] : points[seg + 1];
            
            double t2 = t * t;
            double t3 = t2 * t;
            
            CPVector3 pt;
            for (int d = 0; d < 3; ++d) {
                pt[d] = 0.5 * ((2*p1[d]) + 
                              (-p0[d] + p2[d]) * t +
                              (2*p0[d] - 5*p1[d] + 4*p2[d] - p3[d]) * t2 +
                              (-p0[d] + 3*p1[d] - 3*p2[d] + p3[d]) * t3);
            }
            
            result.push_back(pt);
        }
    }
    
    result.push_back(points.back());
    return result;
}

CartesianPath CartesianPathPlanner::planSpline(
    const std::vector<CPVector3>& waypoints,
    const std::vector<CPMatrix3x3>& orientations) {
    
    CartesianPath path;
    path.type = PathType::SPLINE;
    path.valid = true;
    path.message = "样条路径";
    
    if (waypoints.size() < 2) {
        path.valid = false;
        path.message = "至少需要2个路径点";
        return path;
    }
    
    int num_samples = static_cast<int>(1.0 / config_.sample_ds) + 1;
    std::vector<CPVector3> spline_points = computeCubicSpline(waypoints, num_samples);
    
    // 计算路径长度
    path.total_length = 0;
    for (size_t i = 1; i < spline_points.size(); ++i) {
        path.total_length += norm(subtract(spline_points[i], spline_points[i-1]));
    }
    
    path.total_time = path.total_length / config_.linear_velocity;
    
    // 默认姿态
    CPMatrix3x3 default_orientation = orientations.empty() ? eye3() : orientations[0];
    
    path.points.reserve(spline_points.size());
    double cumulative_length = 0;
    
    for (size_t i = 0; i < spline_points.size(); ++i) {
        CartesianPathPoint pt;
        pt.s = (path.total_length > 1e-9) ? cumulative_length / path.total_length : 0;
        pt.pose.position = spline_points[i];
        
        // 姿态
        if (!orientations.empty() && orientations.size() == waypoints.size()) {
            // 在路径点之间插值姿态
            int seg = static_cast<int>(pt.s * (waypoints.size() - 1));
            if (seg >= static_cast<int>(waypoints.size()) - 1) seg = waypoints.size() - 2;
            double local_s = pt.s * (waypoints.size() - 1) - seg;
            pt.pose.orientation = slerp(orientations[seg], orientations[seg + 1], local_s);
        } else {
            pt.pose.orientation = default_orientation;
        }
        
        // 速度
        if (i > 0) {
            CPVector3 dir = subtract(spline_points[i], spline_points[i-1]);
            pt.linear_velocity = scale(normalize(dir), config_.linear_velocity);
            cumulative_length += norm(dir);
        } else {
            pt.linear_velocity = {0, 0, 0};
        }
        pt.angular_velocity = {0, 0, 0};
        
        path.points.push_back(pt);
    }
    
    return path;
}

CartesianPath CartesianPathPlanner::combinePaths(const std::vector<CartesianPath>& paths) {
    CartesianPath combined;
    combined.type = PathType::LINEAR;  // 混合类型
    combined.valid = true;
    combined.message = "组合路径";
    combined.total_length = 0;
    combined.total_time = 0;
    
    double s_offset = 0;
    
    for (size_t p = 0; p < paths.size(); ++p) {
        const auto& path = paths[p];
        if (!path.valid) continue;
        
        double s_scale = path.total_length;
        combined.total_length += path.total_length;
        combined.total_time += path.total_time;
        
        for (size_t i = 0; i < path.points.size(); ++i) {
            if (p > 0 && i == 0) continue;  // 避免重复点
            
            CartesianPathPoint pt = path.points[i];
            pt.s = (s_offset + pt.s * s_scale) / (combined.total_length > 0 ? combined.total_length : 1);
            combined.points.push_back(pt);
        }
        
        s_offset += s_scale;
    }
    
    // 重新归一化 s
    if (combined.total_length > 0) {
        for (auto& pt : combined.points) {
            pt.s = pt.s;  // 已在循环中计算
        }
    }
    
    return combined;
}

CartesianPathPoint CartesianPathPlanner::interpolate(const CartesianPath& path, double s) {
    CartesianPathPoint pt;
    pt.s = s;
    
    if (path.points.empty()) {
        pt.pose.position = {0, 0, 0};
        pt.pose.orientation = eye3();
        pt.linear_velocity = {0, 0, 0};
        pt.angular_velocity = {0, 0, 0};
        return pt;
    }
    
    if (s <= 0) return path.points.front();
    if (s >= 1) return path.points.back();
    
    // 二分查找
    size_t low = 0, high = path.points.size() - 1;
    while (low < high - 1) {
        size_t mid = (low + high) / 2;
        if (path.points[mid].s <= s) {
            low = mid;
        } else {
            high = mid;
        }
    }
    
    const auto& p0 = path.points[low];
    const auto& p1 = path.points[high];
    double alpha = (p1.s > p0.s) ? (s - p0.s) / (p1.s - p0.s) : 0;
    
    pt.s = s;
    for (int d = 0; d < 3; ++d) {
        pt.pose.position[d] = p0.pose.position[d] + alpha * (p1.pose.position[d] - p0.pose.position[d]);
        pt.linear_velocity[d] = p0.linear_velocity[d] + alpha * (p1.linear_velocity[d] - p0.linear_velocity[d]);
        pt.angular_velocity[d] = p0.angular_velocity[d] + alpha * (p1.angular_velocity[d] - p0.angular_velocity[d]);
    }
    pt.pose.orientation = slerp(p0.pose.orientation, p1.pose.orientation, alpha);
    
    return pt;
}

double CartesianPathPlanner::computePathLength(const CartesianPath& path) {
    double length = 0;
    for (size_t i = 1; i < path.points.size(); ++i) {
        length += norm(subtract(path.points[i].pose.position, 
                                path.points[i-1].pose.position));
    }
    return length;
}

void CartesianPathPlanner::printPath(const CartesianPath& path, int max_points) {
    std::cout << "\n===== 笛卡尔路径信息 =====\n";
    std::cout << "类型: ";
    switch (path.type) {
        case PathType::LINEAR: std::cout << "直线"; break;
        case PathType::CIRCULAR: std::cout << "圆弧"; break;
        case PathType::SPLINE: std::cout << "样条"; break;
    }
    std::cout << "\n";
    std::cout << "状态: " << (path.valid ? "有效" : "无效") << "\n";
    std::cout << "信息: " << path.message << "\n";
    std::cout << "路径长度: " << std::fixed << std::setprecision(4) << path.total_length << " m\n";
    std::cout << "总时间: " << path.total_time << " s\n";
    std::cout << "点数: " << path.points.size() << "\n";
    
    if (!path.points.empty()) {
        std::cout << "\n路径点:\n";
        std::cout << std::setw(6) << "s" 
                  << std::setw(10) << "X" << std::setw(10) << "Y" << std::setw(10) << "Z"
                  << std::setw(10) << "Vx" << std::setw(10) << "Vy" << std::setw(10) << "Vz" << "\n";
        std::cout << std::string(66, '-') << "\n";
        
        int step = std::max(1, (int)path.points.size() / max_points);
        for (size_t i = 0; i < path.points.size() && (int)i < max_points * step; i += step) {
            const auto& pt = path.points[i];
            std::cout << std::fixed << std::setprecision(3);
            std::cout << std::setw(6) << pt.s;
            for (int d = 0; d < 3; ++d) std::cout << std::setw(10) << pt.pose.position[d];
            for (int d = 0; d < 3; ++d) std::cout << std::setw(10) << pt.linear_velocity[d];
            std::cout << "\n";
        }
    }
    std::cout << "==========================\n";
}

} // namespace rs_arm

