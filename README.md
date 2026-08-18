# MiniSim — Mini Musculoskeletal Simulation

一个用 C++ 教学为目的的简化肌肉骨骼仿真系统：2-DOF 关节（髋+膝）配 4 条拮抗肌，
外层包装成强化学习（Gym 风格）环境。代码同时演示 namespace / class / RAII /
shared_ptr / lambda 回调 / Eigen / CMake 库与可执行文件分离等 C++ 语法点。

> 领域术语定义见 [CONTEXT.md](./CONTEXT.md)。下面只讲结构和读法。

---

## 架构总览

四层结构，从底到顶、职责单一、单向依赖：

```
┌─────────────────────────────────────────────────────┐
│  main.cpp                  ← 创建对象、跑仿真循环     │  ← 你读代码的入口
├─────────────────────────────────────────────────────┤
│  Environment (RL 接口)      ← Reset/Step/State/Reward │  ← 包装 Character 成 Gym 风格
├─────────────────────────────────────────────────────┤
│  Character (身体 + 物理)    ← 关节状态、肌肉列表、积分 │  ← 仿真主体
├─────────────────────────────────────────────────────┤
│  Muscle / MuscleInfo (肌肉) ← 力、力矩臂、激活         │  ← 最底层，不依赖别人
└─────────────────────────────────────────────────────┘
```

**依赖方向**：只能往下。`Muscle` 不知道 `Character`，`Character` 不知道 `Environment`。
这样底层可以单独测试、单独复用，不会被上层的改动牵连。

### 依赖关系图（谁持有谁）

```
main
 │
 ├─ std::shared_ptr<Character>  ── 拥有 ──► Character
 │                                              │
 │                                              ├─ std::vector<Muscle*>  ── 拥有 ──► Muscle
 │                                              │   (裸指针，析构里 delete，RAII 教学点)
 │                                              │
 │                                              └─ 持有 MuscleInfo 的拷贝 (via Muscle)
 │
 └─ Environment
        │
        └─ std::shared_ptr<Character>  ── 共享所有权 ──► (同一个 Character)
```

关键点：
- `main` 创建 `Character`（`shared_ptr`），同时把同一个 `shared_ptr` 传给 `Environment`，
  所以两者共享所有权——只要还有任意一方活着，`Character` 就不会被销毁。
- `Character` 用**裸指针** `vector<Muscle*>` 持有肌肉，在析构函数里手动 `delete`。
  这是为了演示 RAII 手动管理；真实项目里会用 `unique_ptr`，见 [TUTORIAL.md](./TUTORIAL.md) 的说明。

---

## 数据流：一次 `Step` 里发生了什么

这是理解整个项目的核心。从 RL 的 `action` 到关节运动，经过四步：

```
  RL action (Eigen::VectorXd, 长度=肌肉数)
  例: [0.8, 0.2, 0.7, 0.3]  ← 每个元素是一块肌肉的激活值
      │
      ▼  Environment::Step()
  ┌───────────────────────────────────────────────────────┐
  │ 1. character->SetMuscleActivations(action)            │
  │    action[i] → mMuscles[i]->SetActivation(...)        │
  │    （action 不直接是力矩，它要先经过肌肉这一层）        │
  └───────────────────────────────────────────────────────┘
      │
      ▼  Character::Step(dt, external_torque)
  ┌───────────────────────────────────────────────────────┐
  │ 2. muscle_torque = ComputeTotalTorque()                │
  │    对每块肌肉:                                          │
  │      force  = max_force × activation      (标量)       │
  │      torque = force × moment_arm           (向量/DOF)  │
  │    累加所有肌肉 → 总肌肉力矩                            │
  └───────────────────────────────────────────────────────┘
      │
      ▼
  ┌───────────────────────────────────────────────────────┐
  │ 3. total_torque = muscle_torque + external_torque      │
  │    acceleration(i) = total_torque(i) / mass(i)        │
  └───────────────────────────────────────────────────────┘
      │
      ▼  半隐式欧拉积分
  ┌───────────────────────────────────────────────────────┐
  │ 4. velocity += acceleration × dt                       │
  │    position += velocity × dt        ← 用更新后的速度   │
  │    velocity *= 0.99                 ← 阻尼，防止发散    │
  └───────────────────────────────────────────────────────┘
      │
      ▼
  Reward = -‖position - target‖² + alive_bonus
  State  = [positions, velocities, phase]
```

**核心物理链条**：`action → 激活 → 肌肉力 → 关节力矩 → 角加速度 → 角速度 → 角度`。
每一步都有明确的物理含义，读代码时按这个链条对照。

---

## 文件结构

```
exercise0/
├── CMakeLists.txt          # 构建配置：simcore 库 + mini_sim 可执行文件
├── CONTEXT.md              # 领域术语表（生物力学 + RL 词汇）
├── README.md               # 本文件：架构、依赖、数据流、快速上手
├── TUTORIAL.md             # 从零实现路线（想照着重写一遍时看）
├── include/
│   ├── Muscle.h            # Muscle 类 + MuscleInfo 结构体声明
│   ├── Character.h         # Character 类声明
│   └── Environment.h       # Environment 类声明
└── src/
    ├── Muscle.cpp          # Muscle 实现
    ├── Character.cpp       # Character 实现
    ├── Environment.cpp     # Environment 实现
    └── main.cpp            # 仿真主循环
```

### 每个文件的职责

| 文件 | 职责 | 依赖 |
|------|------|------|
| `Muscle.h/.cpp` | 一块肌肉：力 = max_force × activation；力矩 = 力 × 力矩臂。最底层。 | 无 |
| `Character.h/.cpp` | 一个身体：关节状态 + 肌肉列表 + 物理积分（半隐式欧拉）。 | Muscle |
| `Environment.h/.cpp` | RL 包装层：Reset/Step/State/Reward/IsEnd。 | Character |
| `main.cpp` | 串起来：创建 Character、加 4 块肌肉、跑 100 步循环。 | Environment, Character |

---

## 语法覆盖清单

本项目刻意覆盖了一组 C++ 语法点。下表是"在哪儿能看到"，详细讲解见各文件内的首次出现处注释。

| 语法点 | 在哪里用到 |
|--------|-----------|
| class + struct | Muscle/Character/Environment 用 class，MuscleInfo 用 struct |
| 头文件/实现分离 | .h 声明 + .cpp 实现 |
| 继承 + virtual | Character::GetDOF() 是虚函数 |
| shared_ptr / make_shared | Environment 持有 Character 的 shared_ptr |
| 裸指针 + 手动 delete (RAII) | Character 持有 vector<Muscle*>，析构里 delete |
| vector / map | 肌肉列表、奖励记录 |
| Eigen::VectorXd | 关节位置、速度、力矩 |
| lambda + 捕获 | ComputeTotalTorque 的回调参数，main 里的观察 lambda |
| std::function | 回调参数类型 |
| const T& 传参 | 所有 setter 方法 |
| const 成员函数 | 函数后的 const |
| auto | 遍历和类型推断 |
| namespace | 全部在 namespace MiniSim 中 |
| range-for | 遍历肌肉 |
| static_cast | size_t → int 转换 |
| std::clamp | SetActivation 限制激活到 [0,1] |
| 异常 (throw) | 维度不匹配等参数检查 |
| CMake: find_package | 找 Eigen3 |
| CMake: add_library + add_executable | 先建 simcore 库，再建 mini_sim |
| CMake: target_link_libraries PUBLIC/PRIVATE | simcore PUBLIC Eigen，mini_sim PRIVATE simcore |

---

## 编译运行

```bash
cd exercise0
cmake -B build
cmake --build build
./build/mini_sim
```

输出会看到：Character 创建日志、4 块肌肉创建日志、每步的 state/reward、每块肌肉的力，
最后是析构清理日志（验证 RAII）。

---

## 想深入？

- **想读懂现有代码** → 按本文件的"架构总览"和"数据流"对照读，从 `main.cpp` 入手。
- **想从零重写一遍** → 看 [TUTORIAL.md](./TUTORIAL.md)，按依赖关系分阶段实现。
