import mujoco
import numpy as np
import time

# ==============================
# 半人马机器人 MuJoCo 控制
# 轮足混合 + 平衡 + 双臂
# ==============================

# 你只需要把这里换成你的半人马 XML
MODEL_PATH = "centauro.xml"

model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)
viewer = mujoco.Viewer(model, data)

DT = 0.002
freq = 500

# 简单PD控制器
def pd(q, dq, q_des, kp=60, kd=8):
    return kp * (q_des - q) - kd * dq

# 主循环
while viewer.is_running():
    t0 = time.time()

    q = data.qpos[:]
    dq = data.qvel[:]
    tau = np.zeros(model.nu)

    # 1. 轮控：前进
    wheel_ids = [8,9,10,11]
    for j in wheel_ids:
        tau[j] = 12 * (0.5 - data.qvel[j])

    # 2. 腿平衡
    leg_ids = [0,1,2,3,4,5,6,7]
    for j in leg_ids:
        tau[j] = pd(q[j], dq[j], 0.0, kp=100, kd=12)

    # 3. 手臂保持稳定
    arm_ids = [12,13,14,15,16,17]
    for j in arm_ids:
        tau[j] = pd(q[j], dq[j], 0.0, kp=80, kd=10)

    data.ctrl[:] = tau
    mujoco.mj_step(model, data)
    viewer.render()

    # 控制频率
    elapsed = time.time() - t0
    if elapsed < DT:
        time.sleep(DT - elapsed)
