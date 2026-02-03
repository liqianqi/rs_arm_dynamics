# forward_kinematics 包

# 库
add_library(fk_lib SHARED
    src/forward_kinematics.cpp
)
target_include_directories(fk_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 测试程序
add_executable(fk_test sample/fk_test.cpp)
target_link_libraries(fk_test fk_lib)

