import mujoco
import numpy as np
import time

# ==============================
# MuJoCo 半人马机器人 轮足+全身控制
# 适用于：4轮足 + 躯干 + 双臂 的半人马构型
# ==============================

class CentauroMujocoController:
    def __init__(self, model_path="centauro.xml"):
        # 加载MuJoCo模型
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.viewer = mujoco.Viewer(self.model, self.data)
        
        # 控制频率
        self.freq = 500
        self.dt = 1.0 / self.freq

        # 关节索引（根据你的半人马URDF修改）
        self.leg_joints = [0,1,2,3,4,5,6,7]    # 四条腿关节
        self.wheel_joints = [8,9,10,11]         # 四个轮子
        self.arm_joints = [12,13,14,15,16,17]   # 双臂
        self.nv = self.model.nv

    def pd_control(self, q, dq, q_des, dq_des, kp=80, kd=10):
        return kp * (q_des - q) - kd * dq

    def wheel_control(self, vx=0.5):
        """轮式模式：平地高速"""
        tau = np.zeros(self.nv)
        for j in self.wheel_joints:
            tau[j] = 15 * (vx - self.data.qvel[j])
        return tau

    def leg_balance_control(self):
        """足式平衡控制"""
        tau = np.zeros(self.nv)
        base_z = self.data.qpos[2]
        base_pitch = self.data.qpos[4]
        tau_leg = self.pd_control(
            self.data.qpos[self.leg_joints],
            self.data.qvel[self.leg_joints],
            q_des=np.zeros(len(self.leg_joints)),
            dq_des=np.zeros(len(self.leg_joints)),
            kp=100, kd=15
        )
        tau[self.leg_joints] = tau_leg
        return tau

    def hybrid_control(self, mode="wheel"):
        """半人马核心：轮足混合切换"""
        if mode == "wheel":
            return self.wheel_control(vx=0.6)
        else:
            return self.leg_balance_control()

    def arm_track_control(self, x_des=np.array([0.5, 0, 0.8])):
        """双臂末端轨迹跟踪"""
        tau = np.zeros(self.nv)
        # 伪雅可比+PD（可替换成真雅可比）
        tau_arm = self.pd_control(
            self.data.qpos[self.arm_joints],
            self.data.qvel[self.arm_joints],
            q_des=np.zeros(len(self.arm_joints)),
            dq_des=np.zeros(len(self.arm_joints)),
            kp=120, kd=12
        )
        tau[self.arm_joints] = tau_arm
        return tau

    def run(self):
        """主控制循环"""
        print("半人马MuJoCo控制启动...")
        while self.viewer.is_running():
            step_start = time.time()

            # 模式切换：wheel / leg
            tau = self.hybrid_control(mode="wheel")  
            # 叠加手臂控制
            tau += self.arm_track_control()

            # 下发力矩
            self.data.ctrl[:] = tau
            mujoco.mj_step(self.model, self.data)
            self.viewer.render()

            # 控频
            elapsed = time.time() - step_start
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)

if __name__ == "__main__":
    # 把你的centauro.xml放在同目录
    ctrl = CentauroMujocoController(model_path="centauro.xml")
    ctrl.run()
