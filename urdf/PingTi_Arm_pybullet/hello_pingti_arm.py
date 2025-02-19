import pybullet as p
import pybullet_data
import time
import numpy as np

# 连接 PyBullet 仿真环境（GUI 模式）
physicsClient = p.connect(p.GUI)

# 设置默认搜索路径（用于加载 URDF 等资源）
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 加载地面
p.loadURDF("plane.urdf")

# 你的机械臂 URDF 文件路径（替换成你的路径）
urdf_path = "PingTi_Arm.urdf"

# 加载机械臂
robot_id = p.loadURDF(urdf_path, basePosition=[0, 0, 0], useFixedBase=True, globalScaling=3.0)

# 设置重力
p.setGravity(0, 0, -9.8)

num_joints = p.getNumJoints(robot_id)
print(f"机械臂的关节数量: {num_joints}")

rotation_joint_indices = []

for i in range(num_joints):
    joint_info = p.getJointInfo(robot_id, i)
    print(f"关节 {i}: 名称 = {joint_info[1].decode()}, 类型 = {joint_info[2]}")
    if joint_info[2] == 0:
        rotation_joint_indices.append(i)

target_positions = [0.5, -0.3, 0.8, -0.5, 0.2, 0.5]  # 根据机械臂的 DOF 调整

assert len(rotation_joint_indices) == len(target_positions)

while True:
    for joint_idx, target_position in zip(rotation_joint_indices, target_positions):
        p.setJointMotorControl2(robot_id, joint_idx, p.POSITION_CONTROL, target_position)

    t = time.time() % 10 
    traj_positions = [0.5 * np.sin(2 * np.pi * t / 10), -0.3 * np.cos(2 * np.pi * t / 10), 0.8, -0.5, 0.2]

    for i in range(len(traj_positions)):
        p.setJointMotorControl2(robot_id, joint_idx, p.POSITION_CONTROL, traj_positions[i])

    p.stepSimulation()

    time.sleep(1. / 240.)

p.disconnect()
