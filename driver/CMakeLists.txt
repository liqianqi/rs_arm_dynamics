# driver 包 - RobStride 电机 SocketCAN 驱动

# 查找线程库
find_package(Threads REQUIRED)

# 库
add_library(driver_lib SHARED
    src/driver.cpp
)
target_include_directories(driver_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(driver_lib Threads::Threads)

# 测试程序
add_executable(driver_test sample/driver_test.cpp)
target_link_libraries(driver_test driver_lib)
