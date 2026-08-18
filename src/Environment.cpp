#include "Environment.h"
#include <stdexcept>

// ============================================================================
// Environment.cpp — RL 环境包装层实现
// 职责：把 Character 的物理仿真包装成 Gym 风格接口。
// 关键流程（见 README.md「数据流」）：
//   Step(action) → SetMuscleActivations → Character::Step → GetReward
// ============================================================================

namespace MiniSim{
    Environment::Environment(std::shared_ptr<Character> character, int max_steps, double dt)
        : mCharacter(character),
          mStepCount(0),
          mMaxSteps(max_steps),
          mDt(dt)
    {
        if (!mCharacter){
            throw std::invalid_argument("character must not be null");
        }
        mTargetPositions = Eigen::VectorXd::Zero(mCharacter->GetDOF());
    }

    void Environment::Reset()
    {
        mStepCount = 0;
        mCharacter->Reset();
        mRewardMap.clear();
    }

    // 这是 RL action 进入物理仿真的入口——整个数据流的起点。
    double Environment::Step(const Eigen::VectorXd& action)
    {
        // 1. action → 每块肌肉的激活（action[i] = 第 i 块肌肉的激活值）
        mCharacter->SetMuscleActivations(action);
        // 2. 推进一步物理（外部力矩这里设为零，纯肌肉驱动）
        Eigen::VectorXd external_torque = Eigen::VectorXd::Zero(mCharacter->GetDOF());
        mCharacter->Step(mDt, external_torque);
        mStepCount++;
        return GetReward();
    }

    Eigen::VectorXd Environment::GetState() const
    {
        int dof = mCharacter->GetDOF();
        // 状态向量布局：[positions, velocities, phase]
        //   phase = 当前步数 / 最大步数，表示"这一回合进行到哪了"。
        Eigen::VectorXd state(2 * dof + 1);
        state.head(dof) = mCharacter->GetPositions();          // 前 dof 个：位置
        state.segment(dof, dof) = mCharacter->GetVelocities();   // 中间 dof 个：速度
        state(2 * dof) = static_cast<double>(mStepCount) / mMaxSteps;  // 最后 1 个：phase
        return state;
    }

    double Environment::GetReward()
    {
        // 奖励 = -位置跟踪误差² + 存活奖励
        //   越接近目标位置，误差越小，奖励越高（负平方，最大为 0）。
        Eigen::VectorXd error = mCharacter->GetPositions() - mTargetPositions;
        double tracking_reward = -error.squaredNorm();
        double alive_reward = IsEnd() ? 0.0 : 1.0;
        double total_reward = tracking_reward + alive_reward;

        // 分项记录到 map，方便外部拆开看奖励构成。
        mRewardMap["tracking"] = tracking_reward;
        mRewardMap["alive"] = alive_reward;
        mRewardMap["total"] = total_reward;
        return total_reward;
    }

    bool Environment::IsEnd() const
    {
        return mStepCount >= mMaxSteps;
    }

    void Environment::SetTargetPositions(const Eigen::VectorXd& target_positions)
    {
        if (target_positions.size() != mCharacter->GetDOF()){
            throw std::invalid_argument("target position size must match num_dof");
        }
        mTargetPositions = target_positions;
    }
}
