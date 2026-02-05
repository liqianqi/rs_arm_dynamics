#!/usr/bin/env python3
"""
RS-A3 机械臂运动可视化 (带URDF模型)

使用 PyBullet 加载 URDF 并可视化运动轨迹

依赖：
  pip install pybullet numpy

使用：
  python visualize_urdf_motion.py [trajectory.csv]
"""

import pybullet as p
import pybullet_data
import numpy as np
import time
import sys
import os

# 默认路径
URDF_PATH = "/home/ubuntu/URDFly/urdf/rs.urdf"
CSV_PATH = "smooth_motion_trajectory.csv"


def load_trajectory(filename):
    """加载CSV轨迹文件"""
    data = np.genfromtxt(filename, delimiter=',', skip_header=1)
    return {
        'time': data[:, 0],
        'q': data[:, 1:7],      # 关节角度
        'pos': data[:, 13:16],  # 末端位置
    }


def motor_to_urdf_angles(q_motor):
    """
    将电机角度转换为URDF关节角度
    
    方向系数: {-1, 1, -1, 1, -1, -1}
    - dir=-1: URDF角度 = -motor角度
    - dir=+1: URDF角度 = +motor角度 (DH正方向=电机正方向)
    """
    dirs = np.array([-1, 1, -1, 1, -1, -1])
    return dirs * q_motor


def visualize_with_pybullet(urdf_path, trajectory, speed=1.0, record_video=False):
    """使用PyBullet可视化"""
    
    # 连接物理引擎 (GUI模式)
    physics_client = p.connect(p.GUI)
    
    # 设置搜索路径
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 设置重力和时间步长
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1./240.)
    
    # 设置相机视角
    p.resetDebugVisualizerCamera(
        cameraDistance=0.5,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[0, 0, 0.1]
    )
    
    # 加载地面
    plane_id = p.loadURDF("plane.urdf")
    
    # 加载机械臂URDF
    try:
        robot_id = p.loadURDF(
            urdf_path,
            basePosition=[0, 0, 0],
            baseOrientation=p.getQuaternionFromEuler([0, 0, 0]),
            useFixedBase=True
        )
        print(f"成功加载URDF: {urdf_path}")
    except Exception as e:
        print(f"加载URDF失败: {e}")
        p.disconnect()
        return
    
    # 获取关节信息
    num_joints = p.getNumJoints(robot_id)
    print(f"关节数量: {num_joints}")
    
    # 找到可动关节
    movable_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        joint_type = joint_info[2]
        if joint_type != p.JOINT_FIXED:
            movable_joints.append(i)
            print(f"  关节 {i}: {joint_name} (类型: {joint_type})")
    
    print(f"可动关节索引: {movable_joints}")
    
    # 绘制末端轨迹
    print("\n绘制末端轨迹...")
    trail_color = [0, 0, 1]  # 蓝色
    for i in range(0, len(trajectory['pos']) - 1, 10):
        p.addUserDebugLine(
            trajectory['pos'][i],
            trajectory['pos'][i + 1],
            trail_color,
            lineWidth=2,
            lifeTime=0
        )
    
    # 标记起点和终点
    p.addUserDebugText("Start", trajectory['pos'][0], textColorRGB=[0, 1, 0], textSize=1.5)
    p.addUserDebugText("End", trajectory['pos'][-1], textColorRGB=[1, 0, 0], textSize=1.5)
    
    # 录制视频
    if record_video:
        log_id = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "robot_motion_urdf.mp4")
    
    # 播放轨迹
    print("\n开始播放轨迹... (按 Ctrl+C 退出)")
    print(f"  轨迹时长: {trajectory['time'][-1]:.2f} s")
    print(f"  播放速度: {speed}x")
    
    # 添加控制滑块
    speed_slider = p.addUserDebugParameter("Speed", 0.1, 5.0, speed)
    pause_button = p.addUserDebugParameter("Pause/Resume", 1, 0, 0)
    
    try:
        frame_idx = 0
        last_pause_val = 0
        paused = False
        
        while frame_idx < len(trajectory['time']):
            # 检查暂停按钮
            pause_val = p.readUserDebugParameter(pause_button)
            if pause_val != last_pause_val:
                paused = not paused
                last_pause_val = pause_val
            
            if paused:
                time.sleep(0.1)
                continue
            
            # 读取速度
            current_speed = p.readUserDebugParameter(speed_slider)
            
            # 设置关节角度 (电机角度转换为URDF角度)
            q_motor = trajectory['q'][frame_idx]
            q_urdf = motor_to_urdf_angles(q_motor)
            for i, joint_idx in enumerate(movable_joints[:6]):  # 只控制前6个关节
                p.resetJointState(robot_id, joint_idx, q_urdf[i])
            
            # 步进仿真
            p.stepSimulation()
            
            # 显示时间
            t = trajectory['time'][frame_idx]
            
            # 控制播放速度
            if frame_idx < len(trajectory['time']) - 1:
                dt = trajectory['time'][frame_idx + 1] - t
                time.sleep(dt / current_speed)
            
            frame_idx += 1
        
        print("\n轨迹播放完成!")
        
        # 循环播放
        print("按任意键退出，或等待自动循环...")
        while True:
            # 重新播放
            for frame_idx in range(len(trajectory['time'])):
                current_speed = p.readUserDebugParameter(speed_slider)
                q_motor = trajectory['q'][frame_idx]
                q_urdf = motor_to_urdf_angles(q_motor)
                for i, joint_idx in enumerate(movable_joints[:6]):
                    p.resetJointState(robot_id, joint_idx, q_urdf[i])
                p.stepSimulation()
                
                if frame_idx < len(trajectory['time']) - 1:
                    dt = trajectory['time'][frame_idx + 1] - trajectory['time'][frame_idx]
                    time.sleep(dt / current_speed)
            
            time.sleep(1.0)  # 暂停1秒后重新播放
            
    except KeyboardInterrupt:
        print("\n用户中断")
    
    if record_video:
        p.stopStateLogging(log_id)
        print("视频已保存: robot_motion_urdf.mp4")
    
    p.disconnect()


def visualize_static_frames(urdf_path, trajectory, num_frames=5):
    """显示静态关键帧"""
    
    physics_client = p.connect(p.DIRECT)  # 无头模式
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # 加载URDF
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
    
    num_joints = p.getNumJoints(robot_id)
    movable_joints = []
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        if joint_info[2] != p.JOINT_FIXED:
            movable_joints.append(i)
    
    # 选择关键帧
    indices = np.linspace(0, len(trajectory['time']) - 1, num_frames, dtype=int)
    
    print(f"\n关键帧位置 (共{num_frames}帧):")
    for idx in indices:
        t = trajectory['time'][idx]
        q = trajectory['q'][idx]
        pos = trajectory['pos'][idx]
        print(f"  t={t:.2f}s: 位置=({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
    
    p.disconnect()


def save_frames_as_images(urdf_path, trajectory, output_dir="frames", num_frames=10):
    """保存关键帧图片"""
    
    os.makedirs(output_dir, exist_ok=True)
    
    # 使用EGL渲染 (无需显示器)
    physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # 加载场景
    p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True)
    
    # 获取可动关节
    num_joints = p.getNumJoints(robot_id)
    movable_joints = [i for i in range(num_joints) if p.getJointInfo(robot_id, i)[2] != p.JOINT_FIXED]
    
    # 选择关键帧
    indices = np.linspace(0, len(trajectory['time']) - 1, num_frames, dtype=int)
    
    # 渲染设置
    width, height = 640, 480
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=[0.3, 0.3, 0.3],
        cameraTargetPosition=[0, 0, 0.1],
        cameraUpVector=[0, 0, 1]
    )
    proj_matrix = p.computeProjectionMatrixFOV(
        fov=60, aspect=width/height, nearVal=0.01, farVal=10
    )
    
    print(f"\n保存 {num_frames} 帧图片到 {output_dir}/")
    
    for frame_num, idx in enumerate(indices):
        # 设置关节角度 (电机角度转换为URDF角度)
        q_motor = trajectory['q'][idx]
        q_urdf = motor_to_urdf_angles(q_motor)
        for i, joint_idx in enumerate(movable_joints[:6]):
            p.resetJointState(robot_id, joint_idx, q_urdf[i])
        
        p.stepSimulation()
        
        # 渲染图片
        _, _, rgb, _, _ = p.getCameraImage(
            width, height, view_matrix, proj_matrix,
            renderer=p.ER_TINY_RENDERER
        )
        
        # 保存图片
        import matplotlib.pyplot as plt
        rgb_array = np.array(rgb, dtype=np.uint8).reshape(height, width, 4)[:, :, :3]
        
        filename = os.path.join(output_dir, f"frame_{frame_num:03d}_t{trajectory['time'][idx]:.2f}s.png")
        plt.imsave(filename, rgb_array)
        print(f"  保存: {filename}")
    
    p.disconnect()
    print(f"完成! 图片保存在 {output_dir}/")


def main():
    # 解析参数
    csv_file = CSV_PATH
    urdf_file = URDF_PATH
    
    # 检查命令行参数
    for arg in sys.argv[1:]:
        if arg.endswith('.csv'):
            csv_file = arg
        elif arg.endswith('.urdf'):
            urdf_file = arg
    
    # 查找CSV文件
    possible_paths = [
        csv_file,
        f"../build/{csv_file}",
        f"build/{csv_file}",
        f"/home/ubuntu/URDFly/rs_arm_dynamics/build/{csv_file}",
    ]
    
    for path in possible_paths:
        if os.path.exists(path):
            csv_file = path
            break
    
    if not os.path.exists(csv_file):
        print(f"找不到轨迹文件: {csv_file}")
        print("请先运行 smooth_motion_demo 生成轨迹")
        return
    
    if not os.path.exists(urdf_file):
        print(f"找不到URDF文件: {urdf_file}")
        return
    
    print("=" * 50)
    print("RS-A3 机械臂运动可视化 (URDF模型)")
    print("=" * 50)
    print(f"URDF文件: {urdf_file}")
    print(f"轨迹文件: {csv_file}")
    
    # 加载轨迹
    trajectory = load_trajectory(csv_file)
    print(f"轨迹点数: {len(trajectory['time'])}")
    print(f"时间范围: 0 ~ {trajectory['time'][-1]:.2f} s")
    
    print("\n选择可视化模式:")
    print("  1. PyBullet GUI (实时动画)")
    print("  2. PyBullet GUI (带视频录制)")
    print("  3. 保存关键帧图片")
    print("  4. 显示关键帧信息")
    
    try:
        choice = input("\n请选择 (1-4, 默认1): ").strip() or "1"
    except EOFError:
        choice = "3"  # 非交互模式默认保存图片
    
    if choice == '1':
        visualize_with_pybullet(urdf_file, trajectory, speed=1.0, record_video=False)
    elif choice == '2':
        visualize_with_pybullet(urdf_file, trajectory, speed=1.0, record_video=True)
    elif choice == '3':
        save_frames_as_images(urdf_file, trajectory, output_dir="urdf_frames", num_frames=10)
    elif choice == '4':
        visualize_static_frames(urdf_file, trajectory, num_frames=10)
    else:
        print("无效选择")


if __name__ == '__main__':
    main()

