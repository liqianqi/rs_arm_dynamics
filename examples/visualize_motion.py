#!/usr/bin/env python3
"""
RS-A3 机械臂运动可视化

功能：
  1. 读取CSV轨迹文件
  2. 3D动画显示末端轨迹
  3. 简化机械臂骨架动画
  4. 姿态变化曲线

依赖：
  pip install numpy matplotlib

使用：
  python visualize_motion.py smooth_motion_trajectory.csv
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import sys
import os

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Arial Unicode MS']
plt.rcParams['axes.unicode_minus'] = False

# MDH参数 (与C++代码一致)
MDH_PARAMS = [
    {'theta_offset': 0.0, 'd': 0.06400010, 'a': 0.0, 'alpha': 0.0},
    {'theta_offset': 0.0, 'd': 0.02590000, 'a': -0.01714870, 'alpha': np.pi/2},
    {'theta_offset': 2.76108628, 'd': 0.0, 'a': 0.19000000, 'alpha': 0.0},
    {'theta_offset': -2.76108628, 'd': -0.02590014, 'a': 0.16155494, 'alpha': 0.0},
    {'theta_offset': -np.pi/2, 'd': -0.00000003, 'a': -0.04920000, 'alpha': np.pi/2},
    {'theta_offset': 0.0, 'd': -0.00805000, 'a': -0.00000003, 'alpha': np.pi/2},
]

# 关节方向系数
JOINT_DIRECTIONS = [-1, -1, -1, -1, -1, -1]


def mdh_transform(theta_offset, d, a, alpha, q_motor, direction):
    """计算MDH变换矩阵"""
    q_dh = direction * q_motor
    theta = theta_offset + q_dh
    
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    T = np.array([
        [ct, -st, 0, a],
        [st*ca, ct*ca, -sa, -d*sa],
        [st*sa, ct*sa, ca, d*ca],
        [0, 0, 0, 1]
    ])
    return T


def forward_kinematics(q):
    """正向运动学：计算各关节位置"""
    T = np.eye(4)
    positions = [T[:3, 3].copy()]  # 基座位置
    
    for i in range(6):
        p = MDH_PARAMS[i]
        Ti = mdh_transform(p['theta_offset'], p['d'], p['a'], p['alpha'], 
                          q[i], JOINT_DIRECTIONS[i])
        T = T @ Ti
        positions.append(T[:3, 3].copy())
    
    return positions, T


def load_trajectory(filename):
    """加载CSV轨迹文件"""
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    
    trajectory = {
        'time': data[:, 0],
        'q': data[:, 1:7],      # 关节角度
        'v': data[:, 7:13],     # 关节速度
        'pos': data[:, 13:16],  # 末端位置
        'euler': data[:, 16:19] # 末端姿态欧拉角
    }
    return trajectory


def visualize_trajectory_3d(trajectory, save_gif=False):
    """3D轨迹可视化"""
    fig = plt.figure(figsize=(14, 6))
    
    # 3D视图
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_title('Robot Arm Motion Trajectory')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.set_zlabel('Z (m)')
    
    # 绘制完整轨迹
    ax1.plot(trajectory['pos'][:, 0], trajectory['pos'][:, 1], trajectory['pos'][:, 2],
             'b-', alpha=0.3, linewidth=1, label='End-effector path')
    
    # 起点和终点
    ax1.scatter(*trajectory['pos'][0], c='green', s=100, marker='o', label='Start')
    ax1.scatter(*trajectory['pos'][-1], c='red', s=100, marker='*', label='End')
    
    # 机械臂骨架
    positions, _ = forward_kinematics(trajectory['q'][0])
    positions = np.array(positions)
    arm_line, = ax1.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                         'ko-', linewidth=2, markersize=5, label='Robot arm')
    
    # 末端点
    end_point, = ax1.plot([positions[-1, 0]], [positions[-1, 1]], [positions[-1, 2]],
                          'r^', markersize=10)
    
    # 设置坐标轴范围
    all_pos = trajectory['pos']
    margin = 0.05
    ax1.set_xlim([all_pos[:, 0].min() - margin, all_pos[:, 0].max() + margin])
    ax1.set_ylim([all_pos[:, 1].min() - margin, all_pos[:, 1].max() + margin])
    ax1.set_zlim([all_pos[:, 2].min() - margin, all_pos[:, 2].max() + margin])
    ax1.legend(loc='upper left')
    
    # 姿态曲线
    ax2 = fig.add_subplot(122)
    ax2.set_title('End-effector Orientation')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Euler Angles (deg)')
    
    euler_deg = np.degrees(trajectory['euler'])
    ax2.plot(trajectory['time'], euler_deg[:, 0], 'r-', label='Yaw (Z)', linewidth=1.5)
    ax2.plot(trajectory['time'], euler_deg[:, 1], 'g-', label='Pitch (Y)', linewidth=1.5)
    ax2.plot(trajectory['time'], euler_deg[:, 2], 'b-', label='Roll (X)', linewidth=1.5)
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 时间指示线
    time_line = ax2.axvline(x=0, color='k', linestyle='--', alpha=0.5)
    time_text = ax2.text(0.02, 0.95, '', transform=ax2.transAxes)
    
    plt.tight_layout()
    
    def update(frame):
        idx = frame
        
        # 更新机械臂骨架
        positions, _ = forward_kinematics(trajectory['q'][idx])
        positions = np.array(positions)
        arm_line.set_data(positions[:, 0], positions[:, 1])
        arm_line.set_3d_properties(positions[:, 2])
        
        # 更新末端点
        end_point.set_data([positions[-1, 0]], [positions[-1, 1]])
        end_point.set_3d_properties([positions[-1, 2]])
        
        # 更新时间线
        time_line.set_xdata([trajectory['time'][idx], trajectory['time'][idx]])
        time_text.set_text(f't = {trajectory["time"][idx]:.2f} s')
        
        return arm_line, end_point, time_line, time_text
    
    # 降采样以加快动画
    step = max(1, len(trajectory['time']) // 200)
    frames = range(0, len(trajectory['time']), step)
    
    ani = FuncAnimation(fig, update, frames=frames, interval=50, blit=False)
    
    if save_gif:
        print("保存动画到 motion.gif ...")
        ani.save('motion.gif', writer='pillow', fps=20)
        print("保存完成!")
    
    plt.show()
    return ani


def visualize_joint_angles(trajectory):
    """关节角度变化可视化"""
    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.suptitle('Joint Angles and Velocities', fontsize=14)
    
    for i in range(6):
        ax = axes[i // 3, i % 3]
        
        # 角度
        ax.plot(trajectory['time'], np.degrees(trajectory['q'][:, i]), 
                'b-', label='Angle (deg)', linewidth=1.5)
        ax.set_ylabel('Angle (deg)', color='b')
        ax.tick_params(axis='y', labelcolor='b')
        
        # 速度
        ax2 = ax.twinx()
        ax2.plot(trajectory['time'], np.degrees(trajectory['v'][:, i]),
                 'r-', alpha=0.7, label='Velocity (deg/s)', linewidth=1)
        ax2.set_ylabel('Velocity (deg/s)', color='r')
        ax2.tick_params(axis='y', labelcolor='r')
        
        ax.set_title(f'Joint {i+1}')
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def visualize_end_effector(trajectory):
    """末端位置和速度可视化"""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('End-effector Motion State', fontsize=14)
    
    # 位置
    ax1 = axes[0, 0]
    ax1.plot(trajectory['time'], trajectory['pos'][:, 0] * 1000, 'r-', label='X', linewidth=1.5)
    ax1.plot(trajectory['time'], trajectory['pos'][:, 1] * 1000, 'g-', label='Y', linewidth=1.5)
    ax1.plot(trajectory['time'], trajectory['pos'][:, 2] * 1000, 'b-', label='Z', linewidth=1.5)
    ax1.set_title('End-effector Position')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position (mm)')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # 位置速度 (数值微分)
    dt = np.diff(trajectory['time'])
    vel = np.diff(trajectory['pos'], axis=0) / dt[:, np.newaxis]
    
    ax2 = axes[0, 1]
    ax2.plot(trajectory['time'][1:], vel[:, 0] * 1000, 'r-', label='Vx', linewidth=1)
    ax2.plot(trajectory['time'][1:], vel[:, 1] * 1000, 'g-', label='Vy', linewidth=1)
    ax2.plot(trajectory['time'][1:], vel[:, 2] * 1000, 'b-', label='Vz', linewidth=1)
    ax2.set_title('End-effector Linear Velocity')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity (mm/s)')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    # 姿态
    ax3 = axes[1, 0]
    euler_deg = np.degrees(trajectory['euler'])
    ax3.plot(trajectory['time'], euler_deg[:, 0], 'r-', label='Yaw', linewidth=1.5)
    ax3.plot(trajectory['time'], euler_deg[:, 1], 'g-', label='Pitch', linewidth=1.5)
    ax3.plot(trajectory['time'], euler_deg[:, 2], 'b-', label='Roll', linewidth=1.5)
    ax3.set_title('End-effector Orientation (Euler)')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Angle (deg)')
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    
    # 姿态角速度
    omega = np.diff(trajectory['euler'], axis=0) / dt[:, np.newaxis]
    
    ax4 = axes[1, 1]
    ax4.plot(trajectory['time'][1:], np.degrees(omega[:, 0]), 'r-', label='wz (Yaw)', linewidth=1)
    ax4.plot(trajectory['time'][1:], np.degrees(omega[:, 1]), 'g-', label='wy (Pitch)', linewidth=1)
    ax4.plot(trajectory['time'][1:], np.degrees(omega[:, 2]), 'b-', label='wx (Roll)', linewidth=1)
    ax4.set_title('End-effector Angular Velocity')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Angular Vel (deg/s)')
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def visualize_3d_path(trajectory):
    """3D路径静态图"""
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # 按时间着色的轨迹
    pos = trajectory['pos']
    t = trajectory['time']
    
    # 使用颜色映射
    colors = plt.cm.viridis(t / t.max())
    
    for i in range(len(t) - 1):
        ax.plot(pos[i:i+2, 0], pos[i:i+2, 1], pos[i:i+2, 2],
                color=colors[i], linewidth=2)
    
    # 绘制几个关键帧的机械臂
    key_frames = [0, len(t)//4, len(t)//2, 3*len(t)//4, len(t)-1]
    
    for idx in key_frames:
        positions, _ = forward_kinematics(trajectory['q'][idx])
        positions = np.array(positions)
        
        alpha = 0.3 if idx != len(t)-1 and idx != 0 else 0.8
        ax.plot(positions[:, 0], positions[:, 1], positions[:, 2],
                'k-', linewidth=1, alpha=alpha)
        ax.scatter(positions[1:, 0], positions[1:, 1], positions[1:, 2],
                   c='gray', s=20, alpha=alpha)
    
    # 起点终点
    ax.scatter(*pos[0], c='green', s=150, marker='o', label='Start', zorder=5)
    ax.scatter(*pos[-1], c='red', s=150, marker='*', label='End', zorder=5)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Robot Arm Motion Trajectory (color: time)')
    ax.legend()
    
    # 添加颜色条
    sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(0, t.max()))
    sm.set_array([])
    cbar = plt.colorbar(sm, ax=ax, shrink=0.5, aspect=10)
    cbar.set_label('Time (s)')
    
    plt.tight_layout()
    plt.show()


def main():
    # 默认CSV文件路径
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
    else:
        # 尝试在build目录下找
        possible_paths = [
            'smooth_motion_trajectory.csv',
            '../build/smooth_motion_trajectory.csv',
            'build/smooth_motion_trajectory.csv',
        ]
        csv_file = None
        for path in possible_paths:
            if os.path.exists(path):
                csv_file = path
                break
        
        if csv_file is None:
            print("用法: python visualize_motion.py <trajectory.csv>")
            print("\n请先运行 smooth_motion_demo 生成轨迹文件")
            return
    
    if not os.path.exists(csv_file):
        print(f"文件不存在: {csv_file}")
        return
    
    print(f"加载轨迹文件: {csv_file}")
    trajectory = load_trajectory(csv_file)
    
    print(f"轨迹点数: {len(trajectory['time'])}")
    print(f"时间范围: 0 ~ {trajectory['time'][-1]:.2f} s")
    print(f"起点: ({trajectory['pos'][0, 0]:.4f}, {trajectory['pos'][0, 1]:.4f}, {trajectory['pos'][0, 2]:.4f}) m")
    print(f"终点: ({trajectory['pos'][-1, 0]:.4f}, {trajectory['pos'][-1, 1]:.4f}, {trajectory['pos'][-1, 2]:.4f}) m")
    
    print("\n选择可视化模式:")
    print("  1. 3D动画 (机械臂运动)")
    print("  2. 关节角度曲线")
    print("  3. 末端运动状态")
    print("  4. 3D静态轨迹图")
    print("  5. 全部显示")
    print("  6. 保存动画GIF")
    
    try:
        choice = input("\n请选择 (1-6, 默认5): ").strip() or "5"
    except EOFError:
        choice = "5"
    
    if choice == '1':
        visualize_trajectory_3d(trajectory)
    elif choice == '2':
        visualize_joint_angles(trajectory)
    elif choice == '3':
        visualize_end_effector(trajectory)
    elif choice == '4':
        visualize_3d_path(trajectory)
    elif choice == '5':
        visualize_3d_path(trajectory)
        visualize_joint_angles(trajectory)
        visualize_end_effector(trajectory)
        visualize_trajectory_3d(trajectory)
    elif choice == '6':
        visualize_trajectory_3d(trajectory, save_gif=True)
    else:
        print("无效选择")


if __name__ == '__main__':
    main()

