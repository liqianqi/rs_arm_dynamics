/**
 * @file cartesian_planner.h
 * @brief 笛卡尔空间路径规划
 * 
 * 功能：
 *   1. 直线路径 (Linear Path)
 *   2. 圆弧路径 (Circular Path)
 *   3. 样条路径 (Spline Path)
 *   4. 姿态插值 (Orientation Interpolation)
 */

#ifndef CARTESIAN_PLANNER_H
#define CARTESIAN_PLANNER_H

#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <optional>

namespace rs_arm {

constexpr double CP_PI = 3.14159265358979323846;

using CPVector3 = std::array<double, 3>;
using CPMatrix3x3 = std::array<std::array<double, 3>, 3>;
using CPMatrix4x4 = std::array<std::array<double, 4>, 4>;

/**
 * @brief 笛卡尔空间位姿
 */
struct CartesianPose {
    CPVector3 position;      // 位置 (m)
    CPMatrix3x3 orientation; // 姿态 (旋转矩阵)
    
    // 从位置和欧拉角构造
    static CartesianPose fromEuler(const CPVector3& pos, const CPVector3& euler_zyx);
    
    // 转换为4x4变换矩阵
    CPMatrix4x4 toMatrix() const;
    
    // 从4x4变换矩阵构造
    static CartesianPose fromMatrix(const CPMatrix4x4& T);
};

/**
 * @brief 笛卡尔空间路径点
 */
struct CartesianPathPoint {
    double s;                   // 路径参数 [0, 1]
    CartesianPose pose;         // 位姿
    CPVector3 linear_velocity;  // 线速度 (m/s)
    CPVector3 angular_velocity; // 角速度 (rad/s)
};

/**
 * @brief 路径类型
 */
enum class PathType {
    LINEAR,     // 直线
    CIRCULAR,   // 圆弧
    SPLINE      // 样条
};

/**
 * @brief 笛卡尔路径配置
 */
struct CartesianPathConfig {
    double linear_velocity = 0.1;     // 线速度 (m/s)
    double linear_acceleration = 0.5; // 线加速度 (m/s²)
    double angular_velocity = 1.0;    // 角速度 (rad/s)
    double sample_ds = 0.01;          // 路径参数采样步长
    bool interpolate_orientation = true; // 是否插值姿态
};

/**
 * @brief 笛卡尔路径
 */
struct CartesianPath {
    std::vector<CartesianPathPoint> points;
    double total_length;     // 路径长度 (m)
    double total_time;       // 总时间 (s)
    PathType type;
    bool valid;
    std::string message;
};

/**
 * @brief 圆弧参数
 */
struct CircularArcParams {
    CPVector3 center;        // 圆心
    CPVector3 normal;        // 法向量 (圆弧平面)
    double radius;           // 半径
    double start_angle;      // 起始角度
    double end_angle;        // 终止角度
};

/**
 * @brief 笛卡尔路径规划器
 */
class CartesianPathPlanner {
public:
    CartesianPathPlanner();
    
    /**
     * @brief 设置配置
     */
    void setConfig(const CartesianPathConfig& config);
    CartesianPathConfig getConfig() const;
    
    /**
     * @brief 规划直线路径
     * @param start 起始位姿
     * @param end 终止位姿
     * @return 笛卡尔路径
     */
    CartesianPath planLinear(const CartesianPose& start, const CartesianPose& end);
    
    /**
     * @brief 规划直线路径 (仅位置)
     * @param start_pos 起始位置
     * @param end_pos 终止位置
     * @param orientation 固定姿态
     */
    CartesianPath planLinearPosition(const CPVector3& start_pos, 
                                     const CPVector3& end_pos,
                                     const CPMatrix3x3& orientation);
    
    /**
     * @brief 规划圆弧路径 (通过三点)
     * @param start 起点
     * @param via 经过点
     * @param end 终点
     * @param orientation 固定姿态 (或起始姿态)
     */
    CartesianPath planCircular(const CPVector3& start, 
                               const CPVector3& via,
                               const CPVector3& end,
                               const CPMatrix3x3& orientation);
    
    /**
     * @brief 规划圆弧路径 (指定圆心和角度)
     * @param center 圆心
     * @param start 起点
     * @param angle 旋转角度 (rad)
     * @param normal 旋转轴 (圆弧平面法向量)
     * @param orientation 固定姿态
     */
    CartesianPath planCircularByAngle(const CPVector3& center,
                                      const CPVector3& start,
                                      double angle,
                                      const CPVector3& normal,
                                      const CPMatrix3x3& orientation);
    
    /**
     * @brief 规划样条路径
     * @param waypoints 路径点
     * @param orientations 各点姿态 (可选，为空则使用第一个点的姿态)
     */
    CartesianPath planSpline(const std::vector<CPVector3>& waypoints,
                             const std::vector<CPMatrix3x3>& orientations = {});
    
    /**
     * @brief 组合多段路径
     */
    CartesianPath combinePaths(const std::vector<CartesianPath>& paths);
    
    /**
     * @brief 在路径上插值
     * @param path 路径
     * @param s 路径参数 [0, 1]
     */
    CartesianPathPoint interpolate(const CartesianPath& path, double s);
    
    /**
     * @brief 计算路径长度
     */
    double computePathLength(const CartesianPath& path);
    
    /**
     * @brief 打印路径信息
     */
    static void printPath(const CartesianPath& path, int max_points = 10);
    
    // 工具函数
    static CPMatrix3x3 eulerToRotation(const CPVector3& euler_zyx);
    static CPVector3 rotationToEuler(const CPMatrix3x3& R);
    static CPMatrix3x3 slerp(const CPMatrix3x3& R1, const CPMatrix3x3& R2, double t);
    static CPMatrix3x3 eye3();
    
private:
    CartesianPathConfig config_;
    
    // 计算圆弧参数
    std::optional<CircularArcParams> computeCircularParams(
        const CPVector3& p1, const CPVector3& p2, const CPVector3& p3);
    
    // 三次样条插值
    std::vector<CPVector3> computeCubicSpline(const std::vector<CPVector3>& points, int num_samples);
    
    // 向量运算
    static CPVector3 cross(const CPVector3& a, const CPVector3& b);
    static double dot(const CPVector3& a, const CPVector3& b);
    static double norm(const CPVector3& v);
    static CPVector3 normalize(const CPVector3& v);
    static CPVector3 subtract(const CPVector3& a, const CPVector3& b);
    static CPVector3 add(const CPVector3& a, const CPVector3& b);
    static CPVector3 scale(const CPVector3& v, double s);
};

} // namespace rs_arm

#endif // CARTESIAN_PLANNER_H

