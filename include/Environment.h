#pragma once

#include <map>
#include <memory>
#include <string>
#include <Eigen/Dense>
#include "Character.h"

// ============================================================================
// Environment.h — RL 环境包装层（Gym 风格）
// ----------------------------------------------------------------------------
// 职责：把一个 Character 包装成强化学习可交互的对象。
//       提供 Reset / Step(action) / GetState / GetReward / IsEnd 接口。
// 在架构中的位置：最外层。持有 Character（共享所有权），被 main 使用。
// 依赖：Character，不知道 Muscle 的存在（通过 Character 间接交互）。
// 见 README.md「数据流」——这里就是 RL action 进入物理仿真的入口。
// ============================================================================

namespace MiniSim{
    class Environment
    {
    private:
        // shared_ptr：共享所有权。Environment 和 main 都持有同一个 Character，
        // 只要还有任意一方活着，Character 就不会被销毁。
        std::shared_ptr<Character> mCharacter;
        int mStepCount;
        int mMaxSteps;
        double mDt;
        Eigen::VectorXd mTargetPositions;
        std::map<std::string, double> mRewardMap;  // 各项奖励分项值

    public:
        Environment(std::shared_ptr<Character> character, int max_steps = 1000, double dt = 0.01);

        void Reset();
        // action 的每个元素对应一块肌肉的激活值，长度 = 肌肉数。
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
