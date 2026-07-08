#include "Environment.h"
#include <stdexcept>

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

    double Environment::Step(const Eigen::VectorXd& action)
    {
        // action 的每个元素对应一块肌肉的激活值。
        mCharacter->SetMuscleActivations(action);
        Eigen::VectorXd external_torque = Eigen::VectorXd::Zero(mCharacter->GetDOF());
        mCharacter->Step(mDt, external_torque);
        mStepCount++;
        return GetReward();
    }

    Eigen::VectorXd Environment::GetState() const
    {
        int dof = mCharacter->GetDOF();
        // 状态向量布局：[positions, velocities, phase]。
        Eigen::VectorXd state(2 * dof + 1);
        state.head(dof) = mCharacter->GetPositions();
        state.segment(dof, dof) = mCharacter->GetVelocities();
        state(2 * dof) = static_cast<double>(mStepCount) / mMaxSteps;
        return state;
    }

    double Environment::GetReward()
    {
        Eigen::VectorXd error = mCharacter->GetPositions() - mTargetPositions;
        double tracking_reward = -error.squaredNorm();
        double alive_reward = IsEnd() ? 0.0 : 1.0;
        double total_reward = tracking_reward + alive_reward;

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
