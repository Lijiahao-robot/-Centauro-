import mujoco
import mujoco.viewer
import numpy as np
import time
import keyboard

# ====================== 全局参数与控制器定义 ======================
# 控制参数
DT = 0.002
WHEEL_SPEED = 8.0
ARM_KP, ARM_KD = 35, 5
LEG_BALANCE_KP, LEG_BALANCE_KD = 70, 9
MPC_GAIN = 0.12

# 模式变量
wheel_leg_mode = 0  # 0-轮式模式，1-足式越障模式
arm_ctrl_flag = False  # 双臂操控标志

# PD控制器
def pd_controller(current_pos, current_vel, target_pos, kp, kd):
    """PD控制算法，实现关节精准定位"""
    return kp * (target_pos - current_pos) - kd * current_vel

# 全身MPC平衡控制器
def mpc_balance_controller(model, data):
    """基于IMU的模型预测平衡控制，修正基座姿态"""
    imu_ori = data.sensor("base_imu").data[0:3]  # 姿态数据
    imu_ang_vel = data.sensor("base_imu").data[3:6]  # 角速度数据
    # 姿态偏差计算
    pitch_err = imu_ori[1]
    roll_err = imu_ori[0]
    # 腿部修正力矩，实现动态平衡
    leg_torque = np.zeros(8)
    leg_torque[0::2] = MPC_GAIN * (roll_err * 50 - imu_ang_vel[0] * 5)
    leg_torque[1::2] = MPC_GAIN * (pitch_err * 60 - imu_ang_vel[1] * 6)
    return leg_torque

# 轮足模式切换
def switch_wheel_leg_mode():
    global wheel_leg_mode
    wheel_leg_mode = 1 - wheel_leg_mode
    print(f"已切换至：{'足式越障模式' if wheel_leg_mode else '轮式高速模式'}")

# 双臂操控开关
def toggle_arm_control():
    global arm_ctrl_flag
    arm_ctrl_flag = not arm_ctrl_flag
    print(f"双臂操控：{'开启' if arm_ctrl_flag else '关闭'}")

# ====================== 加载模型与初始化 ======================
model = mujoco.MjModel.from_xml_path("centauro_optimized.xml")
data = mujoco.MjData(model)

# 关节索引映射
leg_joint_ids = [7,8,9,10,11,12,13,14]  # 腿部关节
wheel_joint_ids = [15,16,17,18]  # 轮子关节
arm_joint_ids = [19,20,21,22,23,24]  # 手臂关节

# 初始化关节目标位置
leg_target_pos = np.zeros(8)
arm_target_pos = np.zeros(6)

# 键盘绑定
keyboard.on_press_key("up", lambda _: set_wheel_speed(WHEEL_SPEED))
keyboard.on_press_key("down", lambda _: set_wheel_speed(-WHEEL_SPEED))
keyboard.on_press_key("left", lambda _: set_turn_speed(0.6))
keyboard.on_press_key("right", lambda _: set_turn_speed(-0.6))
keyboard.on_press_key("space", lambda _: set_wheel_speed(0))
keyboard.on_press_key("m", lambda _: switch_wheel_leg_mode())
keyboard.on_press_key("a", lambda _: toggle_arm_control())

def set_wheel_speed(speed):
    """设置轮子转速，实现前后移动"""
    data.ctrl[8:12] = speed

def set_turn_speed(ratio):
    """设置转向转速，实现左右转向"""
    data.ctrl[8:10] = WHEEL_SPEED * (1 + ratio)
    data.ctrl[10:12] = WHEEL_SPEED * (1 - ratio)

# ====================== 主仿真循环 ======================
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        loop_start = time.time()
        
        # 1. MPC平衡控制（全模式生效）
        mpc_torque = mpc_balance_controller(model, data)
        data.ctrl[0:8] += mpc_torque
        
        # 2. 轮足模式控制
        if wheel_leg_mode == 0:
            # 轮式模式：腿部保持平衡，轮子驱动
            for i in range(8):
                data.ctrl[i] = pd_controller(data.qpos[leg_joint_ids[i]], 
                                           data.qvel[leg_joint_ids[i]], 
                                           leg_target_pos[i], LEG_BALANCE_KP, LEG_BALANCE_KD)
        else:
            # 足式模式：抬起腿部越障，停止轮子驱动
            data.ctrl[8:12] = 0
            leg_obs_pos = np.array([0, 30, 0, 30, 0, 30, 0, 30]) * np.pi / 180
            for i in range(8):
                data.ctrl[i] = pd_controller(data.qpos[leg_joint_ids[i]], 
                                           data.qvel[leg_joint_ids[i]], 
                                           leg_obs_pos[i], 55, 7)
        
        # 3. 双臂操控控制
        if arm_ctrl_flag:
            arm_target_pos = np.array([15, -20, 10, -15, -20, -10]) * np.pi / 180
            for i in range(6):
                data.ctrl[12+i] = pd_controller(data.qpos[arm_joint_ids[i]], 
                                               data.qvel[arm_joint_ids[i]], 
                                               arm_target_pos[i], ARM_KP, ARM_KD)
        else:
            # 双臂保持初始姿态
            for i in range(6):
                data.ctrl[12+i] = pd_controller(data.qpos[arm_joint_ids[i]], 
                                               data.qvel[arm_joint_ids[i]], 
                                               0, ARM_KP, ARM_KD)
        
        # 仿真步进与界面同步
        mujoco.mj_step(model, data)
        viewer.sync()
        
        # 控制频率校准
        loop_time = time.time() - loop_start
       < DT:
            time.sleep(DT - loop_time)
