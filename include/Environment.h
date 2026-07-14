#pragma once
#include "Character.h"
#include <memory>
#include <map>
#include <Eigen/Dense>
#include <string>

namespace MiniSim{
    class Environment
    {
    private:
        std::shared_ptr<Character> mCharacter;
        int mStepCount;
        int mMaxStep;
        double mDt;
        Eigen::VectorXd mTargetPositions;
        std::map<std::string, double> mRewardMap;
    public:
        Environment(std::shared_ptr<Character> Character, int max_steps = 1000, double dt =0.01);
        
        void Reset();
        double Step(const Eigen::VectorXd& action);
        Eigen::VectorXd GetStates() const;
        double GetReward();
        bool IsEnd();

        int GetStepCount() const {return mStepCount;}
        const std::map<std::string, double> GetRewardMap() const {return mRewardMap;}
        void SetTargetPositions(const Eigen::VectorXd& target_positions);
        std::shared_ptr<Character> GetCharacter() const {return mCharacter;}
        ~Environment();
    };
}