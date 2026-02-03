# joint_trajectory 包 - 关节空间轨迹规划

# 库
add_library(trajectory_lib SHARED
    src/joint_trajectory.cpp
)
target_include_directories(trajectory_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 测试程序
add_executable(trajectory_test sample/trajectory_test.cpp)
target_link_libraries(trajectory_test trajectory_lib)

