# MiniSim — 肌肉骨骼仿真教学项目

一个用 C++ 教学为目的的简化肌肉骨骼仿真系统：2-DOF 关节（髋+膝）配 4 条拮抗肌，
外层包装成强化学习（Gym 风格）环境。代码同时演示 namespace / class / RAII /
shared_ptr / lambda 回调 / Eigen / CMake 库与可执行文件分离等 C++ 语法点。

## Language（领域术语表）

本项目混用了生物力学和强化学习两套词汇。下面是它们的精确含义，避免读代码时混淆。

### 生物力学

**DOF (Degree of Freedom, 自由度)**:
一个关节可独立运动的方向数。本项目 2-DOF = 髋关节屈伸 + 膝关节屈伸。
用 `int mNumDOF` 表示，所有关节向量的长度都等于它。
_Avoid_: 维度、关节数

**Position / Velocity**:
关节角度和角速度。用 `Eigen::VectorXd` 表示，长度 = DOF。
注意是"关节"的，不是"肢体末端"的。
_Avoid_: 角度（angle，单指 position 一个分量）

**Moment Arm (力矩臂)**:
某块肌肉对某个 DOF 的杠杆系数。力矩 = 力 × 力矩臂。
一块肌肉可以对多个 DOF 有力矩臂，所以是 `std::vector<double>`，长度 = DOF。
_Avoid_: 杠杆、臂长

**Activation (激活水平)**:
肌肉的发力程度，归一化到 [0, 1]。0 = 不发力，1 = 最大力。
是 Muscle 的输入，也是 RL Environment 的 action。
_Avoid_: 刺激、effort

**Antagonistic Muscles (拮抗肌)**:
成对出现、作用方向相反的肌肉。本项目 4 条肌肉 = 2 对拮抗（髋屈/伸、膝屈/伸）。
_Avoid_: 对抗肌

**Muscle Force / Joint Torque (肌肉力 / 关节力矩)**:
力 = 最大力 × 激活（标量）；力矩 = 力 × 力矩臂（向量，每个 DOF 一个）。
这是肌肉→关节的核心物理链条。

### 强化学习

**Environment**:
Gym 风格接口：`Reset / Step(action) / GetState / GetReward / IsEnd`。
本项目里它包装一个 Character，把"物理仿真"转成"RL 可交互的对象"。
_Avoid_: 世界、simulator

**Action**:
RL agent 给环境的输入。本项目里 action 向量的每个元素 = 一块肌肉的激活值，
长度 = 肌肉数。注意 action 不直接是力矩，它要先经过肌肉。
_Avoid_: 控制、输入、command

**State / Observation**:
环境对外暴露的状态。布局 = `[positions, velocities, phase]`，phase = 当前步数/最大步数。
_Avoid_: 状态（口语词，state 是 RL 术语）

**Reward**:
标量反馈。本项目 = -位置跟踪误差² + 存活奖励。越接近目标位置越高。
_Avoid_: 得分、分数

**Target Position**:
期望的关节位置，环境用来算跟踪奖励。设置一次，每步都拿来对比。
_Avoid_: 目标（太模糊）、参考轨迹
