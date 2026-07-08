# Phase 0 练习题：Mini Musculoskeletal Simulation

## 题目

实现一个简化版的肌肉骨骼仿真系统，模拟一个 2-DOF 的关节（比如简化版的髋关节 + 膝关节），
用 4 条肌肉（2对拮抗肌）驱动。

### 需要实现的文件

```
phase0-exercise/
├── CMakeLists.txt          # 构建配置
├── include/
│   ├── Muscle.h            # Muscle 类声明
│   ├── Character.h         # Character 类声明
│   └── Environment.h       # Environment 类声明
└── src/
    ├── Muscle.cpp           # Muscle 实现
    ├── Character.cpp        # Character 实现
    ├── Environment.cpp      # Environment 实现
    └── main.cpp             # 仿真主循环
```

### 类设计要求

**Muscle 类**（Hill 型肌肉简化版）：
- 属性：名称(string)、最大力(double)、力矩臂(vector<double>)、激活水平(double)
- 方法：SetActivation()、ComputeForce()（力 = 最大力 × 激活）、ComputeJointTorque()（力矩 = 力 × 力矩臂）
- 析构时打印日志（验证 RAII）

**Character 类**（肌肉骨骼角色）：
- 属性：DOF数量、关节位置(Eigen::VectorXd)、关节速度(Eigen::VectorXd)、质量(Eigen::VectorXd)、肌肉列表(vector<Muscle*>)
- 方法：AddMuscle()、ApplyMuscleForces()（用 lambda 遍历肌肉计算总力矩）、Step()（半隐式欧拉积分）
- 析构时 delete 所有肌肉（RAII）
- GetDOF() 返回 int，是虚函数（为继承做准备）

**Environment 类**（RL 环境，Gym 风格）：
- 属性：Character（shared_ptr）、步数计数器、最大步数、奖励记录(map<string,double>)
- 方法：Reset()、Step()、GetState()、GetReward()、IsEnd()
- 奖励 = 位置跟踪误差的负值 + 存活奖励

**main.cpp**：
- 用 make_shared 创建 Character
- 添加 4 条肌肉（髋屈肌、髋伸肌、膝屈肌、膝伸肌）
- 运行 100 步仿真，每步设置肌肉激活，step 后打印状态和奖励
- 用 lambda 作为状态观察回调

### 语法覆盖清单

| 语法点 | 在哪里用到 |
|--------|-----------|
| class + struct | Muscle/Character/Environment 用 class，MuscleInfo 用 struct |
| 头文件/实现分离 | .h 声明 + .cpp 实现 |
| 继承 + virtual | Character::GetDOF() 是虚函数 |
| shared_ptr / unique_ptr / make_shared | Environment 持有 Character 的 shared_ptr |
| vector / map / unordered_map | 肌肉列表、奖励记录、参数映射 |
| Eigen::VectorXd | 关节位置、速度、力矩 |
| lambda + 捕获 | ApplyMuscleForces 用 lambda 遍历，main 里用 lambda 做回调 |
| RAII | Character 析构 delete 肌肉，Muscle 析构打印日志 |
| const T& 传参 | 所有 setter 方法 |
| auto | 遍历和类型推断 |
| namespace | 全部在 namespace MiniSim 中 |
| range-for | 遍历肌肉、打印状态 |
| CMake: find_package | 找 Eigen3 |
| CMake: add_library + add_executable | 先建 simcore 库，再建 main 可执行文件 |
| CMake: target_link_libraries PUBLIC/PRIVATE | simcore PUBLIC Eigen，main PRIVATE simcore |

## 编译运行

```bash
cd phase0-exercise
mkdir build && cd build
cmake ..
cmake --build .
./mini_sim
```

# 构建顺序建议
可以按**依赖关系**来构建：先写最底层的 `Muscle`，再写管理肌肉和关节状态的 `Character`，再写包装成 RL/Gym 风格接口的 `Environment`，最后写 `main.cpp` 把它们串起来。项目本身就是 2-DOF 髋/膝关节 + 4 条肌肉驱动，文件结构也按 `include/*.h` 声明、`src/*.cpp` 实现来组织。

## 总体构建顺序

推荐顺序是：

```text
1. README / 需求拆解
2. CMakeLists.txt 框架
3. Muscle.h
4. Muscle.cpp
5. Character.h
6. Character.cpp
7. Environment.h
8. Environment.cpp
9. main.cpp
10. 编译测试 + 修 bug
```

更准确地说，**代码依赖链**是：

```text
MuscleInfo / Muscle
        ↓
Character：持有多个 Muscle，负责关节状态和物理积分
        ↓
Environment：持有 Character，提供 Reset / Step / GetState / Reward
        ↓
main：创建对象，添加肌肉，运行仿真循环
        ↓
CMake：把 Muscle.cpp + Character.cpp + Environment.cpp 编成库，再链接 main
```

README 里也明确要求：`Muscle` 负责肌肉力和关节力矩，`Character` 负责肌肉列表、总力矩和积分，`Environment` 负责 `Reset/Step/GetState/GetReward/IsEnd`，`main.cpp` 负责创建 Character、添加 4 条肌肉并运行仿真。

---

## 第 1 步：先写 `Muscle.h`

这是最底层模块，不依赖 `Character` 和 `Environment`。

构建顺序：

```text
1. namespace MiniSim
2. struct MuscleInfo
3. class Muscle
4. 成员变量
5. 构造 / 析构声明
6. SetActivation()
7. GetName() / GetActivation() / GetMaxForce()
8. ComputeForce()
9. ComputeJointTorque()
```

为什么先写它？因为后面的 `Character` 需要用 `MuscleInfo` 来添加肌肉，也需要调用 `Muscle::ComputeForce()` 和 `Muscle::ComputeJointTorque()`。

`MuscleInfo` 只是纯数据结构，包含名称、最大力和力矩臂；`Muscle` 类再在此基础上加激活水平和计算函数。  当前 `Muscle` 类的核心接口包括 `SetActivation()`、`ComputeForce()`、`ComputeJointTorque()`，这些正好对应“设置激活 → 算肌肉力 → 算关节力矩”的物理链条。

---

## 第 2 步：再写 `Muscle.cpp`

函数实现顺序建议：

```text
1. Muscle::Muscle()
2. Muscle::~Muscle()
3. Muscle::SetActivation()
4. Muscle::ComputeForce()
5. Muscle::ComputeJointTorque()
```

逻辑上是：

```text
构造肌肉
  ↓
设置激活 activation
  ↓
F = Fmax * activation
  ↓
tau_i = F * moment_arm_i
```

这里 `SetActivation()` 要先实现，因为后面 `Character::Reset()` 会调用它；`ComputeForce()` 要先实现，因为 `ComputeJointTorque()` 内部需要调用它。

---

## 第 3 步：写 `Character.h`

`Character` 是中间层：它不直接面对 RL，而是负责“身体本体”的状态和动力学。

构建顺序：

```text
1. include Muscle.h
2. class Character
3. 构造 / 析构声明
4. AddMuscle()
5. GetNumMuscles()
6. GetDOF()
7. GetPositions() / GetVelocities()
8. SetPositions() / SetVelocities()
9. ComputeTotalTorque()
10. Step()
11. Reset()
12. private 成员变量
```

关键成员变量是：

```cpp
int mNumDOF;
Eigen::VectorXd mPositions;
Eigen::VectorXd mVelocities;
Eigen::VectorXd mMass;
std::vector<Muscle*> mMuscles;
```

这些变量决定了 `Character` 的职责：保存自由度数量、关节角度、关节角速度、每个 DOF 的等效质量，以及它拥有的肌肉列表。

---

## 第 4 步：写 `Character.cpp`

内部函数推荐顺序：

```text
1. Character::Character()
2. Character::~Character()
3. Character::AddMuscle()
4. Character::ComputeTotalTorque()
5. Character::Step()
6. Character::Reset()
```

### 4.1 构造函数先写

构造函数初始化：

```text
DOF 数量
关节位置 = 0
关节速度 = 0
质量 / 转动惯量
```

当前代码里构造函数就是把 `mPositions` 和 `mVelocities` 初始化成零向量，并保存质量参数。

### 4.2 析构函数第二个写

因为 `AddMuscle()` 里用了 `new Muscle(info)`，所以 `Character` 析构时必须 `delete` 每个肌肉对象。当前析构函数就是遍历 `mMuscles` 并逐个 `delete`。

### 4.3 `AddMuscle()` 第三个写

`AddMuscle()` 要先检查力矩臂维度是否等于 DOF 数量。比如 2-DOF 模型中，每条肌肉的 `moment_arms` 必须有两个元素：一个对应髋关节，一个对应膝关节。当前实现中维度不匹配会报错并 `return`，匹配时才 `new Muscle(info)` 放进 `mMuscles`。

### 4.4 `ComputeTotalTorque()` 第四个写

这是 `Character` 的核心函数。它做三件事：

```text
1. 创建 total_torque = 0
2. 遍历所有 Muscle
3. 每块肌肉：
   - ComputeForce()
   - ComputeJointTorque()
   - 累加到 total_torque
   - 可选调用 callback 观察肌肉力
```

当前代码正是这样遍历 `mMuscles`，计算每块肌肉的力和关节力矩，并可通过 callback 把每块肌肉的状态暴露给外部观察。

### 4.5 `Step()` 第五个写

`Step()` 是物理积分函数，依赖 `ComputeTotalTorque()`，所以必须放在后面实现。它的顺序是：

```text
1. muscle_torque = ComputeTotalTorque()
2. total_torque = muscle_torque + external_torque
3. acceleration_i = total_torque_i / mass_i
4. velocity += acceleration * dt
5. position += velocity * dt
6. velocity *= 0.99
```

当前代码用的是半隐式欧拉：先更新速度，再用新速度更新位置。

### 4.6 `Reset()` 最后写

`Reset()` 负责把状态归零，并把所有肌肉激活归零。它依赖 `Muscle::SetActivation()`，所以 `Muscle` 必须先完成。

---

## 第 5 步：写 `Environment.h`

`Environment` 是最外层的 RL 环境封装。它不关心每块肌肉的内部实现，只调用 `Character` 的接口。

构建顺序：

```text
1. include Character.h
2. class Environment
3. 构造函数
4. Reset()
5. Step(action)
6. GetState()
7. GetReward()
8. IsEnd()
9. GetStepCount()
10. GetRewardMap()
11. SetTargetPositions()
12. GetCharacter()
13. private 成员变量
```

`Environment` 的核心接口就是 `Reset()`、`Step()`、`GetState()`、`GetReward()`、`IsEnd()`，同时内部持有 `std::shared_ptr<Character>`、步数、最大步数、时间步长、目标位置和奖励 map。

---

## 第 6 步：写 `Environment.cpp`

内部函数推荐顺序：

```text
1. Environment::Environment()
2. Environment::Reset()
3. Environment::GetReward()
4. Environment::GetState()
5. Environment::IsEnd()
6. Environment::Step()
```

不过你现在的文件里是：

```text
构造函数 → Reset → Step → GetState → GetReward → IsEnd
```

也可以。只是从“从零构建”的角度，我更建议先写 `GetReward/GetState/IsEnd` 这些小函数，再写大的 `Step()`。

### 6.1 构造函数

构造时保存 `Character`，初始化步数、最大步数、仿真步长、目标位置和奖励 map。当前代码里 `mDt = 0.01`，也就是 10 ms 仿真步长。

### 6.2 `Reset()`

逻辑很简单：

```text
step count = 0
character->Reset()
reward map 清空
```

### 6.3 `GetReward()`

奖励逻辑：

```text
error = 当前关节位置 - 目标关节位置
reward = - ||error||²
```

也就是越接近目标位置，奖励越高。

### 6.4 `GetState()`

状态构建：

```text
state = [positions, velocities, phase]
```

其中 `phase = 当前步数 / 最大步数`。README 里也要求状态/奖励/结束判断都由 `Environment` 提供。

### 6.5 `Step(action)`

`Step()` 应该是：

```text
1. action → 每块肌肉激活
2. 计算外力矩
3. character->Step(dt, external_torque)
4. 计算 reward
5. 写入 reward map
6. step count++
```

但这里要注意：**你当前代码里 `Environment::Step(action)` 读取了 action，却没有真正把 action 设置到每块 Muscle 上。** 注释里也写了“不能直接访问 private 的 mMuscles，所以需要通过公共接口”，但目前 `Character` 没有提供“按编号设置肌肉激活”的接口。也就是说，现在 `action` 对仿真结果基本不起作用。

因此从零构建时，建议在 `Character` 里补一个接口：

```cpp
void SetMuscleActivation(int index, double activation);
```

或者：

```cpp
void SetMuscleActivations(const Eigen::VectorXd& activations);
```

然后 `Environment::Step(action)` 里真正调用它。

---

## 第 7 步：最后写 `main.cpp`

`main.cpp` 的构建顺序最清楚，它是把前面模块串起来：

```text
1. include 头文件
2. using namespace MiniSim
3. 创建 DOF 和 mass
4. make_shared 创建 Character
5. AddMuscle 添加 4 条肌肉
6. make_unique 创建 Environment
7. 设置目标位置
8. 定义 unordered_map 保存激活参数
9. 定义 lambda 回调观察肌肉力
10. env->Reset()
11. for 循环：
    - 构造 action
    - ComputeTotalTorque(callback)
    - env->Step(action)
    - env->GetState()
    - env->GetReward()
    - env->IsEnd()
12. 打印 reward map
13. 打印 force log
14. 演示 shared_ptr 引用计数和 RAII
```

当前 `main.cpp` 里先创建 `Character`，再添加 4 条肌肉，然后创建 `Environment` 并设置目标位置。  接着定义 `unordered_map` 参数和 lambda 回调，用于记录肌肉力。  主循环中构造 action、调用 `env->Step(action)`、读取状态和奖励，并检查是否结束。

---

## 第 8 步：最后整理 `CMakeLists.txt`

从逻辑上可以一开始就写 CMake 框架，但真正稳定下来一般放在代码文件齐了之后。构建顺序是：

```text
1. cmake_minimum_required
2. project
3. 设置 C++17
4. find_package(Eigen3 REQUIRED)
5. add_library(simcore SHARED Muscle.cpp Character.cpp Environment.cpp)
6. target_link_libraries(simcore PUBLIC Eigen3::Eigen)
7. target_include_directories(simcore PUBLIC include)
8. add_executable(mini_sim main.cpp)
9. target_link_libraries(mini_sim PRIVATE simcore)
```

README 里也强调 CMake 的顺序是先 `find_package` 找 Eigen，再 `add_library + add_executable`，并且 `simcore PUBLIC Eigen`、`main PRIVATE simcore`。

---

## 最推荐的“从零实现路线”

按最少踩坑的路线，可以这样写：

```text
阶段 1：先让 Muscle 单独可用
Muscle.h
Muscle.cpp
写一个临时 main 测 ComputeForce / ComputeJointTorque

阶段 2：再让 Character 可用
Character.h
Character.cpp
测试 AddMuscle / ComputeTotalTorque / Step / Reset

阶段 3：再封装 Environment
Environment.h
Environment.cpp
测试 Reset / Step / GetState / GetReward / IsEnd

阶段 4：写正式 main
创建 Character
添加肌肉
创建 Environment
循环 step
打印状态、奖励、日志

阶段 5：CMake 整体编译
simcore 库
mini_sim 可执行文件
```

核心原则是：**先写被别人依赖的底层类，再写依赖别人的上层类；先写小函数，再写组合函数；先让它编译，再让它物理逻辑正确。**
