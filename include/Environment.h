#pragma once

#include <map>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "Character.h"

namespace MiniSim{
    // Environment 把 Character 包装成类似强化学习环境的接口：Reset / Step / State / Reward。
    class Environment
    {
    private:
        // shared_ptr 表示共享所有权：只要还有 shared_ptr 指向 Character，对象就不会被销毁。
        std::shared_ptr<Character> mCharacter;
        int mStepCount;
        int mMaxSteps;
        double mDt;
        Eigen::VectorXd mTargetPositions;
        std::map<std::string, double> mRewardMap;

    public:
        Environment(std::shared_ptr<Character> character, int max_steps = 1000, double dt = 0.01);

        void Reset();
        double Step(const Eigen::VectorXd& action);
        Eigen::VectorXd GetState() const;
        double GetReward();
        bool IsEnd() const;

        int GetStepCount() const { return mStepCount; }
        const std::map<std::string, double>& GetRewardMap() const { return mRewardMap; }
        void SetTargetPositions(const Eigen::VectorXd& target_positions);
        std::shared_ptr<Character> GetCharacter() const { return mCharacter; }
    };
}
