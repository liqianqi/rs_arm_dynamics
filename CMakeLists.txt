cmake_minimum_required(VERSION 3.10)
project(rs_arm_dynamics VERSION 1.0)

# ============================================
# 基本设置
# ============================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -O2")

# 设置库输出目录
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# ============================================
# 子包
# ============================================
add_subdirectory(forward_kinematics)
add_subdirectory(inverse_kinematics)
add_subdirectory(joint_trajectory)
add_subdirectory(cartesian_planner)
add_subdirectory(driver)

# ============================================
# 综合示例程序
# ============================================
add_executable(smooth_motion_demo
    examples/smooth_motion_demo.cpp
)
target_include_directories(smooth_motion_demo PRIVATE
    ${CMAKE_SOURCE_DIR}/forward_kinematics/include
    ${CMAKE_SOURCE_DIR}/inverse_kinematics/include
    ${CMAKE_SOURCE_DIR}/cartesian_planner/include
    ${CMAKE_SOURCE_DIR}/joint_trajectory/include
    ${CMAKE_SOURCE_DIR}/driver/include
)
target_link_libraries(smooth_motion_demo
    fk_lib
    ik_lib
    cartesian_lib
    trajectory_lib
    driver_lib
)

# ============================================
# 打印构建信息
# ============================================
message(STATUS "")
message(STATUS "========================================")
message(STATUS "  RS-A3 机械臂动力学库")
message(STATUS "========================================")
message(STATUS "")
message(STATUS "子包:")
message(STATUS "  - forward_kinematics  (正向运动学)")
message(STATUS "  - inverse_kinematics  (逆运动学)")
message(STATUS "  - joint_trajectory    (关节轨迹规划)")
message(STATUS "  - cartesian_planner   (笛卡尔路径规划)")
message(STATUS "  - driver              (电机驱动)")
message(STATUS "")
message(STATUS "测试程序:")
message(STATUS "  - fk_test")
message(STATUS "  - ik_test")
message(STATUS "  - trajectory_test")
message(STATUS "  - cartesian_test")
message(STATUS "  - driver_test")
message(STATUS "")
message(STATUS "综合示例:")
message(STATUS "  - smooth_motion_demo  (平滑运动演示)")
message(STATUS "")
message(STATUS "========================================")
