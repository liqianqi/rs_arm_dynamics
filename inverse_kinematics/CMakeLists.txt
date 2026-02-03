# inverse_kinematics 包 - 逆运动学求解器

# 库
add_library(ik_lib SHARED
    src/inverse_kinematics.cpp
)
target_include_directories(ik_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 测试程序
add_executable(ik_test sample/ik_test.cpp)
target_link_libraries(ik_test ik_lib)

