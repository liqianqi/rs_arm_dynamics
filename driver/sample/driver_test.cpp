/**
 * @file driver_test.cpp
 * @brief RobStride 电机驱动测试程序（无 ROS2 依赖）
 * 
 * 使用方法:
 *   1. 确保 CAN 接口已启用: sudo ip link set can0 up type can bitrate 1000000
 *   2. 运行: ./driver_test
 */

#include "driver.h"
#include <atomic>
#include <csignal>
#include <thread>

using namespace rs_arm;

// 全局标志，用于优雅退出
std::atomic<bool> running{true};

void signal_handler(int signum) {
    std::cout << "\n[!] 接收到信号 " << signum << "，正在退出..." << std::endl;
    running = false;
}

void print_usage() {
    std::cout << "========================================\n";
    std::cout << "  RobStride 电机驱动测试\n";
    std::cout << "========================================\n";
    std::cout << "\n";
    std::cout << "参数说明:\n";
    std::cout << "  CAN 接口:    can0 (默认)\n";
    std::cout << "  主机 ID:     0xFF\n";
    std::cout << "  电机 ID:     0x01\n";
    std::cout << "  电机类型:    ROBSTRIDE_00\n";
    std::cout << "\n";
    std::cout << "控制模式:\n";
    std::cout << "  0 - 运控模式 (位置+速度+KP+KD)\n";
    std::cout << "  1 - 位置模式 PP\n";
    std::cout << "  2 - 速度模式\n";
    std::cout << "  3 - 电流模式\n";
    std::cout << "  4 - 零点模式\n";
    std::cout << "  5 - 位置模式 CSP\n";
    std::cout << "\n";
    std::cout << "按 Ctrl+C 退出\n";
    std::cout << "========================================\n\n";
}

int main(int argc, char** argv) {
    // 设置信号处理
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    print_usage();
    
    // 默认参数
    std::string can_interface = "can0";
    uint8_t master_id = 0xFF;
    uint8_t motor_id = 0x01;
    int actuator_type = static_cast<int>(ActuatorType::ROBSTRIDE_00);
    
    // 解析命令行参数
    if (argc >= 2) {
        can_interface = argv[1];
    }
    if (argc >= 3) {
        motor_id = static_cast<uint8_t>(std::stoi(argv[2], nullptr, 16));
    }
    
    std::cout << "[INFO] CAN 接口: " << can_interface << std::endl;
    std::cout << "[INFO] 主机 ID: 0x" << std::hex << static_cast<int>(master_id) << std::dec << std::endl;
    std::cout << "[INFO] 电机 ID: 0x" << std::hex << static_cast<int>(motor_id) << std::dec << std::endl;
    std::cout << "\n";
    
    try {
        // 创建电机对象
        RobStrideMotor motor(can_interface, master_id, motor_id, actuator_type);
        
        // 获取当前模式
        std::cout << "[1] 获取电机参数..." << std::endl;
        motor.Get_RobStrite_Motor_parameter(0x7005);
        usleep(1000);
        
        // 使能电机
        std::cout << "[2] 使能电机..." << std::endl;
        motor.enable_motor();
        usleep(1000);
        
        // 主循环
        std::cout << "[3] 进入控制循环..." << std::endl;
        
        float velocity = 1.0f;  // rad/s
        
        while (running) {
            // 示例：速度模式
            auto [position, vel, torque, temp] = motor.send_velocity_mode_command(velocity);
            
            // 可选：其他控制模式
            // motor.send_motion_command(0.0, position, velocity, 1.1f, 0.1f);
            // motor.RobStrite_Motor_PosPP_control(velocity, 0.5f, position);
            // motor.RobStrite_Motor_Current_control(-0.1);
            // motor.RobStrite_Motor_PosCSP_control(velocity, position);
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
        // 禁用电机
        std::cout << "\n[4] 禁用电机..." << std::endl;
        motor.Disenable_Motor(0);
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "[DONE] 测试完成" << std::endl;
    return 0;
}
