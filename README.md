# -Centauro-
这里整理了半人马具身智能机器人（Centauro为主） 最核心的GitHub开源项目+配套论文，直接可访问、可复现。
 Centauro 核心开源项目（GitHub）

1. wb_mpc_centauro（全身MPC控制）

• GitHub：https://github.com/ADVRHumanoids/wb_mpc_centauro￼￼￼

• 配套论文：

◦ Whole-body MPC for highly redundant legged manipulators (Humanoids 2023)￼￼￼

◦ 37自由度双臂四足机器人全身模型预测控制，含实验验证

2. XBotControl / XBotCore（实时控制框架）

• GitHub：https://github.com/ADVRHumanoids/XBotControl

• 配套论文：

◦ XBotCore: A real-time cross-robot software platform (Robotic Computing 2017)

◦ Centauro底层实时控制核心，ROS兼容

3. OpenSoT（全身运动生成库）

• GitHub：https://github.com/ADVRHumanoids/OpenSoT

• 配套论文：

◦ Robot control for dummies: Insights and examples using OpenSoT (Humanoids 2017)

◦ 用于冗余机器人全身运动规划与优化
 经典论文（含arXiv/项目页）

Centauro 本体与系统

1. CENTAURO: A Hybrid Locomotion and High Power Resilient Manipulation Platform (RAL 2019)

◦ arXiv：https://arxiv.org/abs/1903.03679

◦ 硬件设计、轮足复合、双臂操作核心论文

2. Flexible Disaster Response of Tomorrow – Final Presentation and Evaluation of the CENTAURO System (RAM 2019)

◦ 项目页：https://www.centauro-project.eu/publications/￼￼￼

◦ 系统总览、搜救场景验证

运动与导航

3. Autonomous Obstacle Crossing Strategies for the Hybrid Wheeled-Legged Robot Centauro (Frontiers 2021)

◦ 链接：https://www.frontiersin.org/articles/10.3389/frobt.2021.721001/full

◦ 轮足越障、地形自适应

4. Variable Configuration Planner for Legged-Rolling Obstacle Negotiation (ICRA 2020)

◦ arXiv：https://arxiv.org/abs/2002.09360

◦ 变构型规划、复杂地形通过

遥操作与具身智能

5. Remote Mobile Manipulation with the Centauro Robot (JFR 2019)

◦ arXiv：https://arxiv.org/abs/1908.01617

◦ 全身遥操作、自主辅助、灾难场景应用
快速入口汇总

• Centauro项目论文主页：https://www.centauro-project.eu/publications/￼￼￼

• IIT开源组织：https://github.com/ADVRHumanoids

• 国内半人马（LimX TRON2）：
pip install mujoco
python run_centauro.py
