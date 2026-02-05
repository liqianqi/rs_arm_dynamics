# RS-A3 机械臂动力学库 (C++)

基于 Modified DH 参数的 RS-A3 六轴机械臂运动学与动力学库。

## 项目结构

```
rs_arm_dynamics/
├── CMakeLists.txt          # CMake 构建文件
├── Makefile                # Make 构建文件
├── README.md               # 说明文档
├── include/                # 头文件
│   └── forward_kinematics.hpp   # 正向运动学
├── src/                    # 源文件
│   └── forward_kinematics.cpp   # 正向运动学实现
├── tests/                  # 测试程序
│   └── test_fk.cpp              # 正向运动学测试
└── build/                  # 构建输出 (自动生成)
    ├── librs_arm_fk.a           # 正向运动学静态库
    └── test_fk                  # 测试程序
```

## 可用模块

| 模块 | 头文件 | 库文件 | 状态 |
|------|--------|--------|------|
| 正向运动学 (FK) | `forward_kinematics.hpp` | `librs_arm_fk.a` | ✅ 完成 |
| 逆向运动学 (IK) | `inverse_kinematics.hpp` | `librs_arm_ik.a` | 🔜 计划中 |
| 动力学 | `dynamics.hpp` | `librs_arm_dyn.a` | 🔜 计划中 |

## 构建方法

### 使用 Make (推荐)

```bash
cd rs_arm_dynamics
make
make run
```

### 使用 CMake

```bash
cd rs_arm_dynamics
mkdir -p build && cd build
cmake ..
make
./bin/test_fk
```

## 使用示例

```cpp
#include "forward_kinematics.hpp"

using namespace rs_arm;

int main() {
    RSArmFK robot;
    
    // 设置关节角度 (弧度) - 电机角度，正值=顺时针
    JointAngles q = {0.1, 0.2, -0.1, 0.3, -0.2, 0.0};
    robot.setJointAngles(q);
    
    // 计算正向运动学
    auto T = robot.computeFK();
    
    // 获取末端位置
    auto pos = robot.getEndEffectorPosition();
    std::cout << "末端位置: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
    
    return 0;
}
```

## 链接库

在你的项目中链接：

```makefile
# Makefile
LIBS = -L/path/to/rs_arm_dynamics/build -lrs_arm_fk
INCLUDE = -I/path/to/rs_arm_dynamics/include
```

```cmake
# CMakeLists.txt
target_link_libraries(your_program rs_arm_fk)
target_include_directories(your_program PRIVATE /path/to/rs_arm_dynamics/include)
```

## MDH 参数

| Joint | θ_offset (rad) | d (m) | a (m) | α (rad) |
|-------|----------------|-------|-------|---------|
| 1 | 0.0 | 0.064 | 0.0 | 0.0 |
| 2 | 0.0 | 0.0259 | -0.0171 | π/2 |
| 3 | 2.7611 | 0.0 | 0.19 | 0.0 |
| 4 | -2.7611 | -0.0259 | 0.1616 | 0.0 |
| 5 | -π/2 | 0.0 | -0.0492 | π/2 |
| 6 | 0.0 | -0.00805 | 0.0 | π/2 |

## 电机方向

默认所有电机 **顺时针为正** (方向系数 = -1)。

如需修改：
```cpp
JointDirections dirs = {-1, -1, +1, -1, -1, -1};  // Joint 3 逆时针为正
robot.setJointDirections(dirs);
```
  