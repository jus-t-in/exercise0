#include "Environment.h"
#include <cmath>
#include <iostream>
#include <memory>
#include <vector>

// ============================================================================
// main.cpp — 仿真主循环：把 Muscle/Character/Environment 串起来
// 流程（见 README.md「数据流」）：
//   创建 Character → 加 4 块肌肉 → 创建 Environment → 循环 100 步：
//     构造 action → env.Step(action) → 打印 state/reward → 观察肌肉力
// ============================================================================

int main()
{
    using namespace MiniSim;

    // --- 1. 创建身体：2-DOF（髋+膝），每 DOF 等效质量 1.0 ---
    Eigen::VectorXd mass(2);
    mass << 1.0, 1.0;
    // make_shared：一次性分配对象 + 控制块，比 new + shared_ptr 构造更安全高效。
    auto character = std::make_shared<Character>(2, mass);

    // --- 2. 加 4 块肌肉：2 对拮抗肌 ---
    //   每块的 moment_arms 长度 = DOF = 2：第一个分量作用髋关节，第二个作用膝关节。
    //   正负号表示方向相反（屈肌 vs 伸肌），这就是"拮抗"的体现。
    character->AddMuscle(MuscleInfo("hip_flexor",  120.0, { 0.05, 0.00}));
    character->AddMuscle(MuscleInfo("hip_extensor", 120.0, {-0.05, 0.00}));
    character->AddMuscle(MuscleInfo("knee_flexor",   80.0, {0.00,  0.04}));
    character->AddMuscle(MuscleInfo("knee_extensor", 80.0, {0.00, -0.04}));

    // --- 3. 创建 RL 环境，设目标位置，Reset ---
    Environment env(character, 100, 0.01);  // 100 步，每步 0.01s
    Eigen::VectorXd target(2);
    target << 0.2, -0.1;                    // 期望髋到 0.2，膝到 -0.1
    env.SetTargetPositions(target);
    env.Reset();

    // --- 4. 观察回调：用 lambda 把每块肌肉的力记录到 vector ---
    //   [&force_log] 按引用捕获外部变量，回调里能写回 force_log。
    //   这个 lambda 就是传给 ComputeTotalTorque 的 callback 参数。
    std::vector<double> force_log;
    auto observe_force = [&force_log](const Muscle& muscle, double force) {
        force_log.push_back(force);
        std::cout << "  " << muscle.GetName() << " force=" << force << std::endl;
    };

    // --- 5. 仿真主循环 ---
    for (int step = 0; step < 100 && !env.IsEnd(); step++){
        // 构造 action：4 个肌肉激活值，用正弦/余弦做周期性动作。
        Eigen::VectorXd action(character->GetNumMuscles());
        action(0) = 0.5 + 0.5 * std::sin(0.1 * step);   // 髋屈肌
        action(1) = 1.0 - action(0);                     // 髋伸肌（与屈肌拮抗，和为 1）
        action(2) = 0.5 + 0.5 * std::cos(0.1 * step);   // 膝屈肌
        action(3) = 1.0 - action(2);                     // 膝伸肌

        // 一次 Step 完成：action → 激活 → 力 → 力矩 → 积分 → 新状态 + reward
        double reward = env.Step(action);
        Eigen::VectorXd state = env.GetState();

        std::cout << "step=" << step
                  << " state=" << state.transpose()
                  << " reward=" << reward << std::endl;
        // 单独再算一次总力矩，纯粹为了用回调观察每块肌肉的力（不推进物理）。
        character->ComputeTotalTorque(observe_force);
    }

    std::cout << "logged forces: " << force_log.size() << std::endl;
    // 离开 main 作用域：character (shared_ptr) 和 env 析构，
    // Character 析构会 delete 所有 Muscle——这就是 RAII 的体现，看终端的析构日志。
    return 0;
}
