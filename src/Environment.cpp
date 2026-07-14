#include "Environment.h"
#include <memory>

namespace MiniSim{
    Environment::Environment(std::shared_ptr<Character> Character, int max_steps, double dt)
       :mCharacter(Character),
        mStepCount(0),
        mMaxStep(max_steps),
        mDt(dt)
    {
        if(!mCharacter){
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
        // action 的每一个元素对应一块肌肉的激活值
        mCharacter->SetMuscleActivations(action);
        Eigen::VectorXd external_torque = Eigen::VectorXd::Zero(mCharacter->GetDOF());
        mCharacter->Step(mDt, external_torque);
        mStepCount++;
        return GetReward();        
    }
    Eigen::VectorXd Environment::GetStates() const
    {
        int dof = mCharacter->GetDOF();
        Eigen::VectorXd state(2 * dof + 1);
        state.head(dof) = mCharacter->GetVelocities();
        state.segment(dof, dof) = mCharacter->GetPositions();
        state(2 * dof) = static_cast<double>(mStepCount) / mMaxStep;
        return state;
    }
    double Environment::GetReward()
    {
        Eigen::VectorXd error = mCharacter->GetPositions() - mTargetPositions;
        double tracking_reward = -error.squaredNorm();
        double alive_reward = IsEnd() ? 0.0 : 0.1;
        double total_reward = tracking_reward + alive_reward;

        mRewardMap["tracking"] = tracking_reward;
        mRewardMap["alive"] = alive_reward;
        mRewardMap["total"] = total_reward;
        return total_reward;
    }
    bool Environment::IsEnd() 
    {
        return mStepCount >= mMaxStep;
    }
    void Environment::SetTargetPositions(const Eigen::VectorXd& target_positions)
    {
        if (target_positions.size() != mCharacter->GetDOF()){
            throw std::invalid_argument("target position size must match num_dof");
        }
        mTargetPositions = target_positions;
    }

}