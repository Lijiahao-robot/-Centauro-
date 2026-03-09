import mujoco
import mujoco.viewer
import numpy as np
import time

# ======================
# 加载半人马模型
# ======================
model = mujoco.MjModel.from_xml_path("centauro.xml")
data = mujoco.MjData(model)

# PD控制器
def pd_control(q, dq, q_des, kp=50, kd=6):
    return kp * (q_des - q) - kd * dq

# ======================
# 运行仿真
# ======================
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        start = time.time()

        # 轮子前进
        data.ctrl[8]  = 5.0
        data.ctrl[9]  = 5.0
        data.ctrl[10] = 5.0
        data.ctrl[11] = 5.0

        # 腿部保持平衡
        for j in range(8):
            data.ctrl[j] = pd_control(data.qpos[j+7], data.qvel[j+7], 0.0, kp=60, kd=8)

        # 手臂保持稳定
        data.ctrl[12] = pd_control(data.qpos[15], data.qvel[15], 0.0, 30, 4)
        data.ctrl[13] = pd_control(data.qpos[16], data.qvel[16], 0.0, 30, 4)
        data.ctrl[14] = pd_control(data.qpos[17], data.qvel[17], 0.0, 30, 4)
        data.ctrl[15] = pd_control(data.qpos[18], data.qvel[18], 0.0, 30, 4)

        mujoco.mj_step(model, data)
        viewer.sync()

        elapsed = time.time() - start
        if elapsed < 0.002:
            time.sleep(0.002 - elapsed)
