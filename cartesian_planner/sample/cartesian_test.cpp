/**
 * @file cartesian_test.cpp
 * @brief 笛卡尔空间路径规划测试程序
 */

#include "cartesian_planner.h"

using namespace rs_arm;

/**
 * @brief 测试1: 直线路径
 */
void test_linear_path() {
    std::cout << "\n========================================\n";
    std::cout << "【测试1】直线路径\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    // 配置
    CartesianPathConfig config;
    config.linear_velocity = 0.1;  // 10 cm/s
    config.sample_ds = 0.05;
    planner.setConfig(config);
    
    // 起点和终点
    CartesianPose start = CartesianPose::fromEuler({0.1, 0, 0.15}, {0, 0, 0});
    CartesianPose end = CartesianPose::fromEuler({0.2, 0.1, 0.1}, {0, 0, 0.5});
    
    std::cout << "\n起点: (" << start.position[0] << ", " 
              << start.position[1] << ", " << start.position[2] << ")\n";
    std::cout << "终点: (" << end.position[0] << ", " 
              << end.position[1] << ", " << end.position[2] << ")\n";
    
    CartesianPath path = planner.planLinear(start, end);
    CartesianPathPlanner::printPath(path, 10);
}

/**
 * @brief 测试2: 直线路径(仅位置)
 */
void test_linear_position() {
    std::cout << "\n========================================\n";
    std::cout << "【测试2】直线路径(固定姿态)\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    CPVector3 start_pos = {0.1, 0, 0.15};
    CPVector3 end_pos = {0.15, 0.05, 0.12};
    CPMatrix3x3 fixed_orientation = CartesianPathPlanner::eye3();
    
    std::cout << "\n起点: (" << start_pos[0] << ", " << start_pos[1] << ", " << start_pos[2] << ")\n";
    std::cout << "终点: (" << end_pos[0] << ", " << end_pos[1] << ", " << end_pos[2] << ")\n";
    
    CartesianPath path = planner.planLinearPosition(start_pos, end_pos, fixed_orientation);
    CartesianPathPlanner::printPath(path, 8);
}

/**
 * @brief 测试3: 圆弧路径(三点定义)
 */
void test_circular_three_points() {
    std::cout << "\n========================================\n";
    std::cout << "【测试3】圆弧路径(三点定义)\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    // 三个点定义圆弧
    CPVector3 start = {0.15, 0, 0.1};
    CPVector3 via = {0.15, 0.05, 0.15};
    CPVector3 end = {0.15, 0.1, 0.1};
    
    CPMatrix3x3 orientation = CartesianPathPlanner::eye3();
    
    std::cout << "\n起点: (" << start[0] << ", " << start[1] << ", " << start[2] << ")\n";
    std::cout << "经过点: (" << via[0] << ", " << via[1] << ", " << via[2] << ")\n";
    std::cout << "终点: (" << end[0] << ", " << end[1] << ", " << end[2] << ")\n";
    
    CartesianPath path = planner.planCircular(start, via, end, orientation);
    CartesianPathPlanner::printPath(path, 10);
}

/**
 * @brief 测试4: 圆弧路径(指定角度)
 */
void test_circular_by_angle() {
    std::cout << "\n========================================\n";
    std::cout << "【测试4】圆弧路径(指定角度)\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    CPVector3 center = {0.1, 0, 0.1};
    CPVector3 start = {0.15, 0, 0.1};  // 半径 0.05
    double angle = CP_PI / 2;  // 90度
    CPVector3 normal = {0, 0, 1};  // Z轴为法向量，在XY平面内画弧
    
    CPMatrix3x3 orientation = CartesianPathPlanner::eye3();
    
    std::cout << "\n圆心: (" << center[0] << ", " << center[1] << ", " << center[2] << ")\n";
    std::cout << "起点: (" << start[0] << ", " << start[1] << ", " << start[2] << ")\n";
    std::cout << "旋转角度: " << angle * 180 / CP_PI << "°\n";
    std::cout << "法向量: (" << normal[0] << ", " << normal[1] << ", " << normal[2] << ")\n";
    
    CartesianPath path = planner.planCircularByAngle(center, start, angle, normal, orientation);
    CartesianPathPlanner::printPath(path, 10);
    
    // 验证终点
    if (!path.points.empty()) {
        auto& end_pt = path.points.back();
        std::cout << "\n终点位置: (" << end_pt.pose.position[0] << ", " 
                  << end_pt.pose.position[1] << ", " << end_pt.pose.position[2] << ")\n";
        std::cout << "预期终点: (0.1, 0.05, 0.1)\n";  // 逆时针90度
    }
}

/**
 * @brief 测试5: 样条路径
 */
void test_spline_path() {
    std::cout << "\n========================================\n";
    std::cout << "【测试5】样条路径\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    // 多个路径点
    std::vector<CPVector3> waypoints = {
        {0.1, 0, 0.15},
        {0.12, 0.03, 0.14},
        {0.15, 0.05, 0.12},
        {0.18, 0.03, 0.11},
        {0.2, 0, 0.1}
    };
    
    std::cout << "\n路径点数: " << waypoints.size() << "\n";
    for (size_t i = 0; i < waypoints.size(); ++i) {
        std::cout << "  P" << i << ": (" << waypoints[i][0] << ", " 
                  << waypoints[i][1] << ", " << waypoints[i][2] << ")\n";
    }
    
    CartesianPath path = planner.planSpline(waypoints);
    CartesianPathPlanner::printPath(path, 10);
}

/**
 * @brief 测试6: 组合路径
 */
void test_combined_path() {
    std::cout << "\n========================================\n";
    std::cout << "【测试6】组合路径(直线+圆弧+直线)\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    CPMatrix3x3 orientation = CartesianPathPlanner::eye3();
    
    // 路径1: 直线
    CPVector3 p1 = {0.1, 0, 0.15};
    CPVector3 p2 = {0.15, 0, 0.15};
    CartesianPath path1 = planner.planLinearPosition(p1, p2, orientation);
    
    // 路径2: 圆弧(90度转弯)
    CPVector3 center = {0.15, 0.05, 0.15};
    CPVector3 arc_start = {0.15, 0, 0.15};
    CartesianPath path2 = planner.planCircularByAngle(center, arc_start, CP_PI/2, {0, 0, 1}, orientation);
    
    // 路径3: 直线
    CPVector3 p3 = {0.2, 0.05, 0.15};
    CPVector3 p4 = {0.2, 0.1, 0.15};
    CartesianPath path3 = planner.planLinearPosition(p3, p4, orientation);
    
    std::cout << "\n路径段:\n";
    std::cout << "  1. 直线: (" << p1[0] << "," << p1[1] << "," << p1[2] << ") -> ("
              << p2[0] << "," << p2[1] << "," << p2[2] << ")\n";
    std::cout << "  2. 圆弧: 90° 转弯\n";
    std::cout << "  3. 直线: (" << p3[0] << "," << p3[1] << "," << p3[2] << ") -> ("
              << p4[0] << "," << p4[1] << "," << p4[2] << ")\n";
    
    CartesianPath combined = planner.combinePaths({path1, path2, path3});
    CartesianPathPlanner::printPath(combined, 12);
}

/**
 * @brief 测试7: 插值功能
 */
void test_interpolation() {
    std::cout << "\n========================================\n";
    std::cout << "【测试7】路径插值\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    CartesianPose start = CartesianPose::fromEuler({0.1, 0, 0.15}, {0, 0, 0});
    CartesianPose end = CartesianPose::fromEuler({0.2, 0.1, 0.1}, {0, 0, CP_PI/4});
    
    CartesianPath path = planner.planLinear(start, end);
    
    std::cout << "\n在任意路径参数 s 处插值:\n";
    std::cout << std::setw(6) << "s" << std::setw(10) << "X" 
              << std::setw(10) << "Y" << std::setw(10) << "Z" << "\n";
    std::cout << std::string(36, '-') << "\n";
    
    std::vector<double> s_values = {0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0};
    for (double s : s_values) {
        CartesianPathPoint pt = planner.interpolate(path, s);
        std::cout << std::fixed << std::setprecision(3);
        std::cout << std::setw(6) << pt.s
                  << std::setw(10) << pt.pose.position[0]
                  << std::setw(10) << pt.pose.position[1]
                  << std::setw(10) << pt.pose.position[2] << "\n";
    }
}

/**
 * @brief 测试8: 圆形轨迹
 */
void test_full_circle() {
    std::cout << "\n========================================\n";
    std::cout << "【测试8】完整圆形轨迹\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    CartesianPathConfig config;
    config.sample_ds = 0.02;
    planner.setConfig(config);
    
    CPVector3 center = {0.15, 0, 0.12};
    CPVector3 start = {0.2, 0, 0.12};  // 半径 0.05
    double angle = 2 * CP_PI;  // 360度
    CPVector3 normal = {0, 0, 1};
    CPMatrix3x3 orientation = CartesianPathPlanner::eye3();
    
    std::cout << "\n圆心: (" << center[0] << ", " << center[1] << ", " << center[2] << ")\n";
    std::cout << "半径: 0.05 m\n";
    std::cout << "角度: 360°\n";
    
    CartesianPath path = planner.planCircularByAngle(center, start, angle, normal, orientation);
    
    std::cout << "\n路径信息:\n";
    std::cout << "  点数: " << path.points.size() << "\n";
    std::cout << "  长度: " << path.total_length << " m (理论值: " << 2*CP_PI*0.05 << " m)\n";
    std::cout << "  时间: " << path.total_time << " s\n";
    
    // 验证起点终点重合
    if (path.points.size() >= 2) {
        auto& first = path.points.front().pose.position;
        auto& last = path.points.back().pose.position;
        double dist = std::sqrt(
            std::pow(last[0]-first[0], 2) + 
            std::pow(last[1]-first[1], 2) + 
            std::pow(last[2]-first[2], 2)
        );
        std::cout << "  起终点距离: " << dist << " m (应接近0)\n";
    }
}

/**
 * @brief 测试9: 姿态插值
 */
void test_orientation_interpolation() {
    std::cout << "\n========================================\n";
    std::cout << "【测试9】姿态插值\n";
    std::cout << "========================================\n";
    
    CartesianPathPlanner planner;
    
    CartesianPathConfig config;
    config.interpolate_orientation = true;
    config.sample_ds = 0.1;
    planner.setConfig(config);
    
    // 起点姿态：无旋转
    // 终点姿态：绕Z轴旋转90度
    CartesianPose start = CartesianPose::fromEuler({0.1, 0, 0.15}, {0, 0, 0});
    CartesianPose end = CartesianPose::fromEuler({0.2, 0, 0.15}, {CP_PI/2, 0, 0});  // Z旋转90°
    
    std::cout << "\n起点姿态 (欧拉角): (0, 0, 0)\n";
    std::cout << "终点姿态 (欧拉角): (90°, 0, 0)\n";
    
    CartesianPath path = planner.planLinear(start, end);
    
    std::cout << "\n姿态变化 (欧拉角):\n";
    std::cout << std::setw(6) << "s" << std::setw(12) << "Yaw(°)" 
              << std::setw(12) << "Pitch(°)" << std::setw(12) << "Roll(°)" << "\n";
    std::cout << std::string(42, '-') << "\n";
    
    for (size_t i = 0; i < path.points.size(); i += std::max(1ul, path.points.size()/8)) {
        const auto& pt = path.points[i];
        CPVector3 euler = CartesianPathPlanner::rotationToEuler(pt.pose.orientation);
        std::cout << std::fixed << std::setprecision(2);
        std::cout << std::setw(6) << pt.s
                  << std::setw(12) << euler[0] * 180 / CP_PI
                  << std::setw(12) << euler[1] * 180 / CP_PI
                  << std::setw(12) << euler[2] * 180 / CP_PI << "\n";
    }
}

int main() {
    std::cout << "╔════════════════════════════════════════╗\n";
    std::cout << "║   笛卡尔空间路径规划测试程序                ║\n";
    std::cout << "╚════════════════════════════════════════╝\n";
    
    test_linear_path();
    test_linear_position();
    test_circular_three_points();
    test_circular_by_angle();
    test_spline_path();
    test_combined_path();
    test_interpolation();
    test_full_circle();
    test_orientation_interpolation();
    
    std::cout << "\n========================================\n";
    std::cout << "所有测试完成!\n";
    std::cout << "========================================\n";
    
    return 0;
}

