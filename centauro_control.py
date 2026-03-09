cat > centauro_control.py << 'EOF'
#!/usr/bin/env python3
"""
半人马机器人 MuJoCo 控制
轮足混合 + 平衡 + 双臂
"""

import os
import sys

# 检查依赖
try:
    import mujoco
    import numpy as np
    import time
except ImportError as e:
    print(f"错误: 缺少依赖 {e}")
    print("请安装: pip install mujoco numpy")
    sys.exit(1)

# ==============================
# 配置
# ==============================

# 模型路径 - 修改为你的实际路径
MODEL_PATH = os.path.expanduser("~/mujoco_menagerie/centauro/centauro.xml")

# 如果没有半人马模型，使用G1作为替代演示
if not os.path.exists(MODEL_PATH):
    print(f"警告: 找不到 {MODEL_PATH}")
    print("使用 G1 人形机器人作为替代演示...")
    MODEL_PATH = os.path.expanduser("~/mujoco_menagerie/unitree_g1/scene.xml")
    
    if not os.path.exists(MODEL_PATH):
        print(f"错误: 也找不到 G1 模型 {MODEL_PATH}")
        print("请确认 mujoco_menagerie 路径正确")
        sys.exit(1)

print(f"加载模型: {MODEL_PATH}")

# ==============================
# 加载模型
# ==============================

try:
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"加载模型失败: {e}")
    sys.exit(1)

print(f"模型加载成功")
print(f"自由度: {model.nv}, 控制数: {model.nu}, 关节数: {model.njnt}")

# 获取关节信息
print("\n关节列表:")
for i in range(min(20, model.njnt)):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    if name:
        print(f"  {i}: {name}")

# ==============================
# 启动查看器
# ==============================

try:
    viewer = mujoco.viewer.launch_passive(model, data)
except Exception as e:
    print(f"启动查看器失败: {e}")
    sys.exit(1)

print("\n查看器启动成功")
print("按 Ctrl+C 停止")

# ==============================
# 控制参数
# ==============================

DT = 0.002          # 仿真步长
target_speed = 0.5  # 轮速目标
kp_wheel = 12       # 轮子PD
kp_leg = 100        # 腿部PD
kp_arm = 80         # 手臂PD

# 重置到初始姿势
mujoco.mj_resetDataKeyframe(model, data, 0)
home_qpos = data.qpos.copy()

# ==============================
# PD控制器
# ==============================

def pd(q, dq, q_des, kp=60, kd=8):
    """PD控制器"""
    return kp * (q_des - q) - kd * dq

# ==============================
# 主控制循环
# ==============================

step = 0
start_time = time.time()

try:
    while viewer.is_running():
        loop_start = time.time()
        
        # 获取当前状态
        q = data.qpos.copy()
        dq = data.qvel.copy()
        
        # 初始化控制力矩
        tau = np.zeros(model.nu)
        
        # ========================================
        # 控制策略（根据实际模型调整）
        # ========================================
        
        # 检测模型类型
        is_g1 = "g1" in MODEL_PATH.lower() or "unitree_g1" in MODEL_PATH.lower()
        is_centauro = "centauro" in MODEL_PATH.lower()
        
        if is_centauro and model.nu >= 18:
            # ===== 半人马控制 =====
            # 轮控: 4个轮子前进
            wheel_ids = list(range(8, 12)) if model.nu > 12 else []
            for j in wheel_ids:
                if j < model.nu:
                    tau[j] = kp_wheel * (target_speed - data.qvel[j])
            
            # 腿平衡: 保持站立
            leg_ids = list(range(0, 8)) if model.nu > 8 else []
            for j in leg_ids:
                if j < len(q) and j < model.nu:
                    tau[j] = pd(q[j], dq[j], home_qpos[j], kp=kp_leg, kd=12)
            
            # 手臂稳定
            arm_ids = list(range(12, 18)) if model.nu > 18 else []
            for j in arm_ids:
                if j < len(q) and j < model.nu:
                    tau[j] = pd(q[j], dq[j], home_qpos[j], kp=kp_arm, kd=10)
                    
        elif is_g1:
            # ===== G1人形机器人控制 =====
            # 行走步态
            phase = 2 * np.pi * 1.0 * data.time  # 1Hz步态
            
            # 腿部关节索引（根据G1模型调整）
            leg_joints = {
                'left_hip': 7, 'left_knee': 8, 'left_ankle': 9,
                'right_hip': 10, 'right_knee': 11, 'right_ankle': 12
            }
            
            # 简单行走: 正弦波驱动
            for name, idx in leg_joints.items():
                if idx < len(q) and idx < model.nu:
                    if 'left' in name:
                        offset = 0.3 * np.sin(phase) if 'hip' in name else 0.4 * max(0, np.sin(phase - 0.5))
                    else:
                        offset = 0.3 * np.sin(phase + np.pi) if 'hip' in name else 0.4 * max(0, np.sin(phase + np.pi - 0.5))
                    
                    target = home_qpos[idx] + offset
                    tau[idx] = pd(q[idx], dq[idx], target, kp=150, kd=10)
            
            # 手臂摆动
            arm_joints = {'left_shoulder': 13, 'right_shoulder': 16}
            for name, idx in arm_joints.items():
                if idx < len(q) and idx < model.nu:
                    offset = 0.4 * np.sin(phase + np.pi if 'left' in name else phase)
                    target = home_qpos[idx] + offset
                    tau[idx] = pd(q[idx], dq[idx], target, kp=80, kd=8)
        
        else:
            # ===== 通用控制: 保持home姿势 =====
            for j in range(min(model.nu, len(q))):
                tau[j] = pd(q[j], dq[j], home_qpos[j], kp=100, kd=10)
        
        # 应用控制
        data.ctrl[:] = np.clip(tau, -100, 100)
        
        # 物理步进
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 控制频率
        elapsed = time.time() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)
        
        # 打印状态
        step += 1
        if step % 500 == 0:
            pos = data.qpos[:3]
            vel = np.linalg.norm(data.qvel[:3])
            print(f"\r时间: {data.time:5.1f}s | 位置: ({pos[0]:4.1f}, {pos[1]:4.1f}, {pos[2]:4.2f}) | 速度: {vel:4.2f}", end='')

except KeyboardInterrupt:
    print("\n\n用户停止")

except Exception as e:
    print(f"\n错误: {e}")
    import traceback
    traceback.print_exc()

finally:
    # 安全关闭
    print("\n关闭查看器...")
    viewer.close()
    print("完成")
EOF

chmod +x centauro_control.py
python3 centauro_control.py



# 1. 确认mujoco安装
python3 -c "import mujoco; print(mujoco.__version__)"

# 2. 确认模型路径
ls ~/mujoco_menagerie/

# 3. 手动修改脚本中的 MODEL_PATH
nano centauro_control.py
