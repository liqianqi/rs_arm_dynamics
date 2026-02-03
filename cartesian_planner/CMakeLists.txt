# cartesian_planner 包 - 笛卡尔空间路径规划

# 库
add_library(cartesian_lib SHARED
    src/cartesian_planner.cpp
)
target_include_directories(cartesian_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 测试程序
add_executable(cartesian_test sample/cartesian_test.cpp)
target_link_libraries(cartesian_test cartesian_lib)

