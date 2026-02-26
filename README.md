# RS-A3 机械臂动力学库 (C++)

基于 Modified DH 参数的 RS-A3 六轴机械臂运动学与控制库，包含正/逆运动学、轨迹规划和电机驱动模块。

---

## 目录

- [项目结构](#项目结构)
- [可用模块](#可用模块)
- [快速开始](#快速开始)
- [构建方法](#构建方法)
- [模块详细说明](#模块详细说明)
  - [正向运动学 (FK)](#1-正向运动学-forward_kinematics)
  - [逆向运动学 (IK)](#2-逆向运动学-inverse_kinematics)
  - [关节轨迹规划](#3-关节轨迹规划-joint_trajectory)
  - [笛卡尔路径规划](#4-笛卡尔路径规划-cartesian_planner)
  - [电机驱动](#5-电机驱动-driver)
- [综合示例](#综合示例)
- [MDH 参数](#mdh-参数)
- [电机方向配置](#电机方向配置)
- [链接库方法](#链接库方法)
- [常见问题](#常见问题)

---

## 项目结构

```
rs_arm_dynamics/
├── CMakeLists.txt              # 主构建文件
├── README.md                   # 说明文档
│
├── forward_kinematics/         # 正向运动学模块
│   ├── include/forward_kinematics.h
│   ├── src/forward_kinematics.cpp
│   ├── sample/fk_test.cpp
│   └── CMakeLists.txt
│
├── inverse_kinematics/         # 逆向运动学模块
│   ├── include/inverse_kinematics.h
│   ├── src/inverse_kinematics.cpp
│   ├── sample/ik_test.cpp
│   └── CMakeLists.txt
│
├── joint_trajectory/           # 关节轨迹规划模块
│   ├── include/joint_trajectory.h
│   ├── src/joint_trajectory.cpp
│   ├── sample/trajectory_test.cpp
│   └── CMakeLists.txt
│
├── cartesian_planner/          # 笛卡尔路径规划模块
│   ├── include/cartesian_planner.h
│   ├── src/cartesian_planner.cpp
│   ├── sample/cartesian_test.cpp
│   └── CMakeLists.txt
│
├── driver/                     # 电机驱动模块
│   ├── include/driver.h
│   ├── src/driver.cpp
│   ├── sample/driver_test.cpp
│   └── CMakeLists.txt
│
├── examples/                   # 综合示例
│   └── smooth_motion_demo.cpp  # 平滑运动演示
│
└── build/                      # 构建输出 (自动生成)
    ├── lib/                    # 静态库
    │   ├── libfk_lib.a
    │   ├── libik_lib.a
    │   ├── libtrajectory_lib.a
    │   ├── libcartesian_lib.a
    │   └── libdriver_lib.a
    └── bin/                    # 可执行文件
        ├── fk_test
        ├── ik_test
        ├── trajectory_test
        ├── cartesian_test
        ├── driver_test
        └── smooth_motion_demo
```

---

## 可用模块

| 模块 | 头文件 | 库文件 | 状态 | 说明 |
|------|--------|--------|------|------|
| 正向运动学 | `forward_kinematics.h` | `libfk_lib.a` | ✅ 完成 | 关节角 → 末端位姿 |
| 逆向运动学 | `inverse_kinematics.h` | `libik_lib.a` | ✅ 完成 | 末端位姿 → 关节角 |
| 关节轨迹规划 | `joint_trajectory.h` | `libtrajectory_lib.a` | ✅ 完成 | 平滑关节运动曲线 |
| 笛卡尔路径规划 | `cartesian_planner.h` | `libcartesian_lib.a` | ✅ 完成 | 直线/圆弧/样条路径 |
| 电机驱动 | `driver.h` | `libdriver_lib.a` | ✅ 完成 | RobStride CAN 控制 |

---

## 快速开始

```bash
# 1. 构建项目
cd rs_arm_dynamics
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 2. 运行测试
./bin/fk_test            # 测试正向运动学
./bin/ik_test            # 测试逆向运动学
./bin/trajectory_test    # 测试关节轨迹
./bin/cartesian_test     # 测试笛卡尔路径
./bin/smooth_motion_demo # 运行综合演示
```

---

## 构建方法

### 使用 CMake (推荐)

```bash
cd rs_arm_dynamics
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 构建产物位置
# 库文件: build/lib/
# 可执行文件: build/bin/
```

### 构建选项

```bash
# Debug 构建
cmake -DCMAKE_BUILD_TYPE=Debug ..

# Release 构建 (默认)
cmake -DCMAKE_BUILD_TYPE=Release ..
```

---

## 模块详细说明

### 1. 正向运动学 (forward_kinematics)

计算给定关节角度下机械臂末端的位置和姿态。

#### 头文件

```cpp
#include "forward_kinematics.h"
```

#### 主要类型

```cpp
namespace rs_arm {

constexpr int NUM_JOINTS = 6;

using Matrix4x4 = std::array<std::array<double, 4>, 4>;
using JointAngles = std::array<double, NUM_JOINTS>;       // 关节角度 (rad)
using JointDirections = std::array<int, NUM_JOINTS>;      // 方向系数 (1/-1)

struct MDHParams {
    double theta_offset;  // 关节角偏移
    double d;             // 连杆偏移
    double a;             // 连杆长度
    double alpha;         // 连杆扭角
};

}
```

#### RSArmFK 类 API

```cpp
class RSArmFK {
public:
    RSArmFK();
    
    // 设置/获取关节角度
    void setJointAngles(const JointAngles& q);
    JointAngles getJointAngles() const;
    void updateJointAngle(int joint_index, double angle);
    
    // 设置/获取关节方向
    void setJointDirections(const JointDirections& directions);
    JointDirections getJointDirections() const;
    
    // 正向运动学计算
    Matrix4x4 computeFK();                        // 计算末端变换矩阵
    Matrix4x4 computeFKToJoint(int joint_index);  // 计算到指定关节
    
    // 获取末端状态
    std::array<double, 3> getEndEffectorPosition();              // 位置 [x,y,z]
    std::array<std::array<double, 3>, 3> getEndEffectorOrientation(); // 旋转矩阵
    
    // 调试
    void printStatus() const;
    static void printMatrix(const Matrix4x4& T, const std::string& name = "T");
};
```

#### 使用示例

```cpp
#include "forward_kinematics.h"
using namespace rs_arm;

int main() {
    RSArmFK fk;
    
    // 设置关节方向 (电机顺时针为正)
    JointDirections dirs = {-1, -1, -1, -1, -1, -1};
    fk.setJointDirections(dirs);
    
    // 设置关节角度 (弧度)
    JointAngles q = {0.0, 0.5, -0.3, 0.0, 0.2, 0.0};
    fk.setJointAngles(q);
    
    // 计算正向运动学
    Matrix4x4 T = fk.computeFK();
    
    // 获取末端位置
    auto pos = fk.getEndEffectorPosition();
    std::cout << "末端位置: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ") m\n";
    
    // 获取末端姿态
    auto rot = fk.getEndEffectorOrientation();
    
    return 0;
}
```

---

### 2. 逆向运动学 (inverse_kinematics)

根据目标末端位姿计算所需的关节角度。

#### 头文件

```cpp
#include "inverse_kinematics.h"
```

#### 主要类型

```cpp
namespace rs_arm {

using IKMatrix4x4 = std::array<std::array<double, 4>, 4>;
using IKJointAngles = std::array<double, 6>;
using IKVector3 = std::array<double, 3>;
using IKMatrix3x3 = std::array<std::array<double, 3>, 3>;
using Jacobian = std::array<std::array<double, 6>, 6>;

// IK 求解结果
struct IKResult {
    bool success;                  // 是否成功
    IKJointAngles joint_angles;    // 关节角度
    int iterations;                // 迭代次数
    double position_error;         // 位置误差 (m)
    double orientation_error;      // 姿态误差 (rad)
    std::string message;           // 状态信息
};

// IK 配置
struct IKConfig {
    int max_iterations = 100;           // 最大迭代次数
    double position_tolerance = 1e-4;   // 位置容差 (m)
    double orientation_tolerance = 1e-3;// 姿态容差 (rad)
    double damping_factor = 0.01;       // 阻尼因子 (DLS)
    double step_size = 1.0;             // 步长
    bool use_joint_limits = true;       // 是否使用关节限位
    IKJointAngles joint_min;            // 关节下限 (rad)
    IKJointAngles joint_max;            // 关节上限 (rad)
};

}
```

#### RSArmIK 类 API

```cpp
class RSArmIK {
public:
    RSArmIK();
    
    // 配置
    void setJointDirections(const IKJointDirections& directions);
    void setConfig(const IKConfig& config);
    
    // IK 求解方法
    IKResult solveIK(const IKVector3& target_pos, 
                     const IKMatrix3x3& target_rot,
                     const IKJointAngles& initial_guess);
                     
    IKResult solveIKPosition(const IKVector3& target_pos,
                             const IKJointAngles& initial_guess);  // 仅位置
                             
    IKResult solveIK(const IKMatrix4x4& target_T,
                     const IKJointAngles& initial_guess);  // 4x4变换矩阵
    
    // 正向运动学 (验证用)
    IKMatrix4x4 computeFK(const IKJointAngles& q);
    
    // 雅可比矩阵
    Jacobian computeJacobian(const IKJointAngles& q);
    
    // 工具函数
    static IKVector3 rotationToEulerZYX(const IKMatrix3x3& R);
    static IKMatrix3x3 eulerZYXToRotation(const IKVector3& euler);
    static void printResult(const IKResult& result);
};
```

#### 使用示例

```cpp
#include "inverse_kinematics.h"
using namespace rs_arm;

int main() {
    RSArmIK ik;
    
    // 配置IK
    IKConfig config;
    config.max_iterations = 200;
    config.position_tolerance = 1e-4;
    config.use_joint_limits = true;
    ik.setConfig(config);
    
    // 目标位置和姿态
    IKVector3 target_pos = {0.3, 0.0, 0.2};  // 米
    IKMatrix3x3 target_rot = {{
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }};
    
    // 初始猜测
    IKJointAngles initial_guess = {0, 0, 0, 0, 0, 0};
    
    // 求解
    IKResult result = ik.solveIK(target_pos, target_rot, initial_guess);
    
    if (result.success) {
        std::cout << "IK 求解成功!\n";
        std::cout << "迭代次数: " << result.iterations << "\n";
        std::cout << "位置误差: " << result.position_error << " m\n";
        RSArmIK::printResult(result);
    }
    
    return 0;
}
```

---

### 3. 关节轨迹规划 (joint_trajectory)

在关节空间生成平滑的运动轨迹。

#### 头文件

```cpp
#include "joint_trajectory.h"
```

#### 主要类型

```cpp
namespace rs_arm {

using JTJointAngles = std::array<double, 6>;
using JTJointVelocities = std::array<double, 6>;
using JTJointAccelerations = std::array<double, 6>;

// 轨迹点
struct TrajectoryPoint {
    double time;                       // 时间 (s)
    JTJointAngles position;            // 位置 (rad)
    JTJointVelocities velocity;        // 速度 (rad/s)
    JTJointAccelerations acceleration; // 加速度 (rad/s²)
};

// 轨迹类型
enum class TrajectoryType {
    CUBIC,       // 三次多项式
    QUINTIC,     // 五次多项式
    TRAPEZOIDAL, // 梯形速度
    SCURVE       // S曲线
};

// 轨迹配置
struct TrajectoryConfig {
    double max_velocity = 2.0;       // 最大速度 (rad/s)
    double max_acceleration = 5.0;   // 最大加速度 (rad/s²)
    double max_jerk = 20.0;          // 最大加加速度 (rad/s³)
    double sample_dt = 0.01;         // 采样间隔 (s)
};

// 完整轨迹
struct Trajectory {
    std::vector<TrajectoryPoint> points;
    double total_time;
    TrajectoryType type;
    bool valid;
    std::string message;
};

}
```

#### JointTrajectoryPlanner 类 API

```cpp
class JointTrajectoryPlanner {
public:
    JointTrajectoryPlanner();
    
    void setConfig(const TrajectoryConfig& config);
    
    // 点到点轨迹规划
    Trajectory planQuintic(const JTJointAngles& start, 
                           const JTJointAngles& end,
                           double duration,
                           const JTJointVelocities& start_vel = {},
                           const JTJointVelocities& end_vel = {});
                           
    Trajectory planCubic(const JTJointAngles& start, 
                         const JTJointAngles& end,
                         double duration,
                         const JTJointVelocities& start_vel = {},
                         const JTJointVelocities& end_vel = {});
                         
    Trajectory planTrapezoidal(const JTJointAngles& start, 
                               const JTJointAngles& end);
                               
    Trajectory planSCurve(const JTJointAngles& start, 
                          const JTJointAngles& end);
    
    // 多点轨迹
    Trajectory planMultiPoint(const std::vector<JTJointAngles>& waypoints,
                              const std::vector<double>& durations,
                              TrajectoryType type = TrajectoryType::QUINTIC);
    
    // 插值
    TrajectoryPoint interpolate(const Trajectory& traj, double t);
    
    // 导出
    static void printTrajectory(const Trajectory& traj, int max_points = 10);
    static bool exportToCSV(const Trajectory& traj, const std::string& filename);
};
```

#### 使用示例

```cpp
#include "joint_trajectory.h"
using namespace rs_arm;

int main() {
    JointTrajectoryPlanner planner;
    
    // 配置
    TrajectoryConfig config;
    config.max_velocity = 1.5;
    config.max_acceleration = 3.0;
    config.sample_dt = 0.01;
    planner.setConfig(config);
    
    // 起点和终点
    JTJointAngles start = {0, 0, 0, 0, 0, 0};
    JTJointAngles end = {0.5, 0.3, -0.2, 0.1, 0.4, 0};
    
    // 五次多项式轨迹 (2秒)
    Trajectory traj = planner.planQuintic(start, end, 2.0);
    
    if (traj.valid) {
        std::cout << "轨迹点数: " << traj.points.size() << "\n";
        std::cout << "总时间: " << traj.total_time << " s\n";
        
        // 导出到CSV
        JointTrajectoryPlanner::exportToCSV(traj, "trajectory.csv");
    }
    
    // 多点轨迹
    std::vector<JTJointAngles> waypoints = {
        {0, 0, 0, 0, 0, 0},
        {0.3, 0.2, -0.1, 0, 0.2, 0},
        {0.5, 0.4, -0.2, 0.1, 0.3, 0}
    };
    std::vector<double> durations = {1.5, 1.5};  // 每段时间
    
    Trajectory multi_traj = planner.planMultiPoint(waypoints, durations);
    
    return 0;
}
```

---

### 4. 笛卡尔路径规划 (cartesian_planner)

在笛卡尔空间规划末端路径（直线、圆弧、样条）。

#### 头文件

```cpp
#include "cartesian_planner.h"
```

#### 主要类型

```cpp
namespace rs_arm {

using CPVector3 = std::array<double, 3>;
using CPMatrix3x3 = std::array<std::array<double, 3>, 3>;
using CPMatrix4x4 = std::array<std::array<double, 4>, 4>;

// 笛卡尔位姿
struct CartesianPose {
    CPVector3 position;       // 位置 (m)
    CPMatrix3x3 orientation;  // 姿态 (旋转矩阵)
    
    static CartesianPose fromEuler(const CPVector3& pos, const CPVector3& euler_zyx);
    CPMatrix4x4 toMatrix() const;
    static CartesianPose fromMatrix(const CPMatrix4x4& T);
};

// 路径点
struct CartesianPathPoint {
    double s;                    // 路径参数 [0, 1]
    CartesianPose pose;          // 位姿
    CPVector3 linear_velocity;   // 线速度 (m/s)
    CPVector3 angular_velocity;  // 角速度 (rad/s)
};

// 路径类型
enum class PathType { LINEAR, CIRCULAR, SPLINE };

// 路径配置
struct CartesianPathConfig {
    double linear_velocity = 0.1;      // 线速度 (m/s)
    double linear_acceleration = 0.5;  // 线加速度 (m/s²)
    double angular_velocity = 1.0;     // 角速度 (rad/s)
    double sample_ds = 0.01;           // 采样步长
    bool interpolate_orientation = true;
};

// 路径
struct CartesianPath {
    std::vector<CartesianPathPoint> points;
    double total_length;  // 路径长度 (m)
    double total_time;    // 总时间 (s)
    PathType type;
    bool valid;
    std::string message;
};

}
```

#### CartesianPathPlanner 类 API

```cpp
class CartesianPathPlanner {
public:
    CartesianPathPlanner();
    
    void setConfig(const CartesianPathConfig& config);
    
    // 直线路径
    CartesianPath planLinear(const CartesianPose& start, const CartesianPose& end);
    CartesianPath planLinearPosition(const CPVector3& start_pos, 
                                     const CPVector3& end_pos,
                                     const CPMatrix3x3& orientation);
    
    // 圆弧路径
    CartesianPath planCircular(const CPVector3& start, 
                               const CPVector3& via,
                               const CPVector3& end,
                               const CPMatrix3x3& orientation);
                               
    CartesianPath planCircularByAngle(const CPVector3& center,
                                      const CPVector3& start,
                                      double angle,
                                      const CPVector3& normal,
                                      const CPMatrix3x3& orientation);
    
    // 样条路径
    CartesianPath planSpline(const std::vector<CPVector3>& waypoints,
                             const std::vector<CPMatrix3x3>& orientations = {});
    
    // 路径组合
    CartesianPath combinePaths(const std::vector<CartesianPath>& paths);
    
    // 插值
    CartesianPathPoint interpolate(const CartesianPath& path, double s);
    double computePathLength(const CartesianPath& path);
    
    // 工具函数
    static CPMatrix3x3 eulerToRotation(const CPVector3& euler_zyx);
    static CPVector3 rotationToEuler(const CPMatrix3x3& R);
    static CPMatrix3x3 slerp(const CPMatrix3x3& R1, const CPMatrix3x3& R2, double t);
    
    static void printPath(const CartesianPath& path, int max_points = 10);
};
```

#### 使用示例

```cpp
#include "cartesian_planner.h"
using namespace rs_arm;

int main() {
    CartesianPathPlanner planner;
    
    // 配置
    CartesianPathConfig config;
    config.linear_velocity = 0.05;  // 5 cm/s
    config.sample_ds = 0.02;
    planner.setConfig(config);
    
    // 直线路径
    CartesianPose start, end;
    start.position = {0.3, 0.0, 0.15};
    start.orientation = CartesianPathPlanner::eulerToRotation({0, 0, 0});
    end.position = {0.35, 0.05, 0.20};
    end.orientation = CartesianPathPlanner::eulerToRotation({0, 0.1, 0});
    
    CartesianPath linear_path = planner.planLinear(start, end);
    
    if (linear_path.valid) {
        std::cout << "路径长度: " << linear_path.total_length << " m\n";
        std::cout << "预计时间: " << linear_path.total_time << " s\n";
    }
    
    // 圆弧路径 (三点定义)
    CPVector3 p1 = {0.3, 0.0, 0.15};
    CPVector3 p2 = {0.32, 0.05, 0.18};  // 经过点
    CPVector3 p3 = {0.35, 0.0, 0.20};
    
    CartesianPath arc_path = planner.planCircular(p1, p2, p3, start.orientation);
    
    return 0;
}
```

---

### 5. 电机驱动 (driver)

通过 SocketCAN 控制 RobStride 系列电机。

#### 头文件

```cpp
#include "driver.h"
```

#### 主要类型

```cpp
namespace rs_arm {

// 电机类型
enum class ActuatorType {
    ROBSTRIDE_00 = 0,
    ROBSTRIDE_01 = 1,
    ROBSTRIDE_02 = 2,
    ROBSTRIDE_03 = 3,
    ROBSTRIDE_04 = 4,
    ROBSTRIDE_05 = 5,
    ROBSTRIDE_06 = 6
};

// 电机运行参数
struct ActuatorOperation {
    double position;  // 位置范围 (rad)
    double velocity;  // 速度范围 (rad/s)
    double torque;    // 力矩范围 (Nm)
    double kp;        // 默认位置增益
    double kd;        // 默认速度阻尼
};

}
```

#### RobStrideMotor 类 API

```cpp
class RobStrideMotor {
public:
    RobStrideMotor(const std::string can_interface, 
                   uint8_t master_id,
                   uint8_t motor_id, 
                   int actuator_type);
    ~RobStrideMotor();
    
    // 基本控制
    std::tuple<float, float, float, float> enable_motor();    // 使能
    void Disenable_Motor(uint8_t clear_error);                // 禁用
    
    // 运控模式 (位置+速度+力矩 混合控制)
    std::tuple<float, float, float, float>
    send_motion_command(float torque, float position_rad, float velocity_rad_s,
                        float kp = 0.5f, float kd = 0.1f);
    
    // 位置模式 PP (点对点)
    std::tuple<float, float, float, float>
    RobStrite_Motor_PosPP_control(float Speed, float Acceleration, float Angle);
    
    // 位置模式 CSP (连续同步)
    std::tuple<float, float, float, float>
    RobStrite_Motor_PosCSP_control(float Speed, float Angle);
    
    // 速度模式
    std::tuple<float, float, float, float>
    send_velocity_mode_command(float velocity_rad_s);
    
    // 电流模式
    std::tuple<float, float, float, float>
    RobStrite_Motor_Current_control(float IqCommand);
    
    // 零点设置
    void RobStrite_Motor_Set_Zero_control();
    void Set_ZeroPos();
    
    // 参数读写
    void Set_RobStrite_Motor_parameter(uint16_t Index, float Value, char Value_mode);
    void Get_RobStrite_Motor_parameter(uint16_t Index);
    
    // 状态读取
    float read_initial_position();
    std::tuple<float, float, float, float> return_data_pvtt(); // 位置,速度,力矩,温度
    
    // CAN ID 设置
    void Set_CAN_ID(uint8_t new_id);
    
public:
    float position_;     // 当前位置 (rad)
    float velocity_;     // 当前速度 (rad/s)
    float torque_;       // 当前力矩 (Nm)
    float temperature_;  // 当前温度 (°C)
};
```

#### 使用示例

```cpp
#include "driver.h"
using namespace rs_arm;

int main() {
    // 创建电机实例
    // 参数: CAN接口, 主机ID, 电机ID, 电机类型
    RobStrideMotor motor("can0", 0x00, 1, 
                         static_cast<int>(ActuatorType::ROBSTRIDE_01));
    
    // 使能电机
    auto [pos, vel, torque, temp] = motor.enable_motor();
    std::cout << "初始位置: " << pos << " rad\n";
    
    // 运控模式 - 发送位置命令
    float target_pos = 0.5;    // 目标位置 (rad)
    float target_vel = 1.0;    // 目标速度 (rad/s)
    float cmd_torque = 0.0;    // 前馈力矩
    float kp = 30.0;           // 位置增益
    float kd = 1.0;            // 速度阻尼
    
    auto result = motor.send_motion_command(cmd_torque, target_pos, target_vel, kp, kd);
    
    // 读取状态
    auto state = motor.return_data_pvtt();
    std::cout << "位置: " << std::get<0>(state) << " rad\n";
    std::cout << "速度: " << std::get<1>(state) << " rad/s\n";
    std::cout << "力矩: " << std::get<2>(state) << " Nm\n";
    std::cout << "温度: " << std::get<3>(state) << " °C\n";
    
    // 禁用电机
    motor.Disenable_Motor(0);
    
    return 0;
}
```

#### CAN 接口配置

```bash
# 配置 CAN 接口 (以 can0 为例)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 查看 CAN 状态
ip -details link show can0

# 监听 CAN 数据
candump can0
```

---

## 综合示例

### smooth_motion_demo

该示例演示了如何整合所有模块实现完整的机械臂平滑运动：

```
FK → IK → 笛卡尔路径规划 → 关节轨迹规划 → 电机控制
```

#### 功能流程

1. **初始化**: 设置关节方向、IK配置、轨迹参数
2. **零位计算**: 使用FK计算零位末端位姿
3. **目标定义**: 定义笛卡尔空间目标位姿
4. **路径规划**: 规划笛卡尔直线/圆弧路径
5. **轨迹转换**: 对路径点进行IK求解，生成关节轨迹
6. **平滑处理**: 五次多项式插值保证速度连续
7. **电机执行**: (可选) 发送到真实电机

#### 运行方式

```bash
cd build
./bin/smooth_motion_demo
```

#### 代码结构

```cpp
// 核心规划器类
class SmoothMotionPlanner {
    RSArmFK fk_;              // 正向运动学
    RSArmIK ik_;              // 逆向运动学
    CartesianPathPlanner cartesian_;  // 笛卡尔路径
    JointTrajectoryPlanner trajectory_; // 关节轨迹
    
    // 规划从起点到目标的平滑运动
    Trajectory planMotion(const IKJointAngles& start_q,
                          const CPVector3& target_pos,
                          const CPMatrix3x3& target_rot,
                          double duration);
};

// 电机控制器 (可选)
class MotorController {
    std::vector<std::unique_ptr<RobStrideMotor>> motors;
    
    bool init(const std::string& can_interface);
    bool enableAll();
    void sendJointPositions(positions, velocities);
    void disableAll();
};
```

#### 启用真实电机控制

修改 `examples/smooth_motion_demo.cpp` 顶部的宏定义:

```cpp
#define ENABLE_REAL_MOTOR_CONTROL true  // 启用真实电机
#define CAN_INTERFACE "can0"            // CAN接口名
```

---

## MDH 参数

RS-A3 机械臂使用 Modified DH 参数：

| Joint | θ_offset (rad) | d (m) | a (m) | α (rad) |
|-------|----------------|-------|-------|---------|
| 1 | 0.0 | 0.064 | 0.0 | 0.0 |
| 2 | 0.0 | 0.0259 | -0.0171 | π/2 |
| 3 | 2.7611 | 0.0 | 0.19 | 0.0 |
| 4 | -2.7611 | -0.0259 | 0.1616 | 0.0 |
| 5 | -π/2 | 0.0 | -0.0492 | π/2 |
| 6 | 0.0 | -0.00805 | 0.0 | π/2 |

---

## 电机方向配置

默认所有电机 **顺时针为正** (方向系数 = -1)。

方向系数说明：
- `+1`: 电机正向 = DH正向 (逆时针为正)
- `-1`: 电机正向 = DH负向 (顺时针为正)

```cpp
// FK
JointDirections fk_dirs = {-1, -1, -1, -1, -1, -1};
fk.setJointDirections(fk_dirs);

// IK
IKJointDirections ik_dirs = {-1, -1, -1, -1, -1, -1};
ik.setJointDirections(ik_dirs);

// 如果某个关节方向相反，修改对应值
// 例如：Joint 3 逆时针为正
JointDirections dirs = {-1, -1, +1, -1, -1, -1};
```

---

## 链接库方法

### CMake 项目

```cmake
# 添加库路径
add_subdirectory(path/to/rs_arm_dynamics)

# 或直接链接预编译库
target_link_libraries(your_program
    ${RS_ARM_LIB_DIR}/libfk_lib.a
    ${RS_ARM_LIB_DIR}/libik_lib.a
    ${RS_ARM_LIB_DIR}/libtrajectory_lib.a
    ${RS_ARM_LIB_DIR}/libcartesian_lib.a
    ${RS_ARM_LIB_DIR}/libdriver_lib.a
)

target_include_directories(your_program PRIVATE
    path/to/rs_arm_dynamics/forward_kinematics/include
    path/to/rs_arm_dynamics/inverse_kinematics/include
    path/to/rs_arm_dynamics/joint_trajectory/include
    path/to/rs_arm_dynamics/cartesian_planner/include
    path/to/rs_arm_dynamics/driver/include
)
```

### Makefile 项目

```makefile
RS_ARM_DIR = /path/to/rs_arm_dynamics

INCLUDE = -I$(RS_ARM_DIR)/forward_kinematics/include \
          -I$(RS_ARM_DIR)/inverse_kinematics/include \
          -I$(RS_ARM_DIR)/joint_trajectory/include \
          -I$(RS_ARM_DIR)/cartesian_planner/include \
          -I$(RS_ARM_DIR)/driver/include

LIBS = -L$(RS_ARM_DIR)/build/lib \
       -lfk_lib -lik_lib -ltrajectory_lib -lcartesian_lib -ldriver_lib

your_program: main.cpp
	g++ -std=c++17 -o $@ $< $(INCLUDE) $(LIBS)
```

---

## 常见问题

### 1. IK 求解失败

**可能原因：**
- 目标位置超出工作空间
- 初始猜测距离解太远
- 关节限位过紧

**解决方法：**
```cpp
IKConfig config;
config.max_iterations = 300;       // 增加迭代次数
config.damping_factor = 0.05;      // 调整阻尼因子
config.use_joint_limits = false;   // 暂时关闭限位
ik.setConfig(config);
```

### 2. CAN 通信失败

**检查步骤：**
```bash
# 1. 确认 CAN 接口已启动
ip link show can0

# 2. 确认波特率正确 (1Mbps)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# 3. 检查是否有错误帧
candump can0

# 4. 确认电机ID正确
```

### 3. 轨迹不平滑

**建议：**
- 使用五次多项式 (`QUINTIC`) 而非三次多项式
- 增加采样率 (`sample_dt = 0.005`)
- 检查关节限速设置是否合理

### 4. FK/IK 结果不一致

**检查：**
- 确保 FK 和 IK 使用相同的关节方向配置
- 验证 MDH 参数一致性
- 注意角度单位（弧度 vs 角度）

---

---

## 可视化工具

项目提供两个 Python 可视化脚本，用于查看轨迹规划结果。

### 1. visualize_motion.py (Matplotlib 可视化)

基于 Matplotlib 的轻量级可视化，无需 GPU。

#### 依赖安装

```bash
pip install numpy matplotlib
```

#### 使用方法

```bash
cd rs_arm_dynamics/examples

# 方式1: 指定CSV文件
python visualize_motion.py smooth_motion_trajectory.csv

# 方式2: 自动查找 (在build目录下)
python visualize_motion.py
```

#### 可视化模式

| 模式 | 说明 |
|------|------|
| 1 | 3D动画 - 机械臂骨架运动 |
| 2 | 关节角度曲线 - 6个关节的角度和速度 |
| 3 | 末端运动状态 - 位置/速度/姿态/角速度 |
| 4 | 3D静态轨迹图 - 按时间着色 |
| 5 | 全部显示 (默认) |
| 6 | 保存动画为 GIF |

#### 输出示例

- 3D 机械臂骨架动画
- 关节角度/速度曲线 (6子图)
- 末端位置、姿态、角速度曲线
- `motion.gif` 动画文件 (模式6)

---

### 2. visualize_urdf_motion.py (PyBullet 3D 可视化)

基于 PyBullet 的真实 URDF 模型可视化，支持物理仿真。

#### 依赖安装

```bash
pip install pybullet numpy
```

#### 使用方法

```bash
cd rs_arm_dynamics/examples

# 方式1: 使用默认路径
python visualize_urdf_motion.py

# 方式2: 指定CSV文件
python visualize_urdf_motion.py smooth_motion_trajectory.csv

# 方式3: 同时指定URDF和CSV
python visualize_urdf_motion.py /path/to/robot.urdf trajectory.csv
```

#### 可视化模式

| 模式 | 说明 |
|------|------|
| 1 | PyBullet GUI 实时动画 (默认) |
| 2 | PyBullet GUI + 视频录制 |
| 3 | 保存关键帧图片 |
| 4 | 显示关键帧信息 |

#### GUI 控制

- **Speed 滑块**: 调节播放速度 (0.1x ~ 5.0x)
- **Pause/Resume**: 暂停/继续播放
- **鼠标拖拽**: 旋转视角
- **滚轮**: 缩放
- **Ctrl+C**: 退出

#### 输出示例

- 实时 3D 机械臂动画 (带 URDF 模型)
- `robot_motion_urdf.mp4` 视频文件 (模式2)
- `urdf_frames/` 关键帧图片 (模式3)

---

### 完整工作流程

```bash
# 1. 构建并运行 C++ 程序生成轨迹
cd rs_arm_dynamics/build
./bin/smooth_motion_demo
# 生成: smooth_motion_trajectory.csv

# 2. Matplotlib 可视化 (轻量)
cd ../examples
python visualize_motion.py ../build/smooth_motion_trajectory.csv

# 3. PyBullet 可视化 (3D模型)
python visualize_urdf_motion.py ../build/smooth_motion_trajectory.csv
```

---

## 许可证

MIT License

## 联系方式

如有问题或建议，请提交 Issue 或联系开发者。
