#pragma once
#include <functional>
#include <vector>
#include <Eigen/Dense>
#include "Muscle.h"

// ============================================================================
// Character.h — 仿真主体：关节状态 + 肌肉列表 + 物理积分
// ----------------------------------------------------------------------------
// 职责：管理一个"身体"——关节位置/速度/质量，以及它拥有的肌肉列表。
//       对外提供 AddMuscle（装肌肉）、SetMuscleActivations（给输入）、
//       Step（推进一步物理）、ComputeTotalTorque（算当前总力矩）。
// 在架构中的位置：中间层。持有 Muscle（往下），被 Environment 持有（往上）。
// 依赖：只依赖 Muscle.h，不知道 Environment 的存在。
// 见 README.md「架构总览」「数据流」。
// ============================================================================

namespace MiniSim{
    class Character
    {
    private:
        int mNumDOF;
        Eigen::VectorXd mPositions;
        Eigen::VectorXd mVelocities;
        Eigen::VectorXd mMass;
        // 教学点：这里用裸指针演示 RAII——AddMuscle 里 new，析构函数里 delete。
        // 真实项目会用 std::unique_ptr<Muscle>，这里刻意手动管理是为了讲清楚所有权。
        std::vector<Muscle*> mMuscles;
    public:
        Character(int num_dof, const Eigen::VectorXd& mass);
        ~Character();
        void AddMuscle(const MuscleInfo& info);
        int GetNumMuscles() const {return static_cast<int>(mMuscles.size());}
        // virtual：子类以后可以重写这个函数。这里先作为继承语法示例。
        virtual int GetDOF() const {return mNumDOF; }
        // 返回 const 引用：避免拷贝。Eigen 向量内部是堆内存，拷贝很贵。
        const Eigen::VectorXd& GetPositions() const {return mPositions; }
        const Eigen::VectorXd& GetVelocities() const {return mVelocities; }
        void SetPositions(const Eigen::VectorXd& pos) { mPositions = pos; }
        void SetVelocities(const Eigen::VectorXd& vel) { mVelocities = vel; }
        void SetMuscleActivation(int index, double activation);
        void SetMuscleActivations(const Eigen::VectorXd& activations);

        // callback 是一个可选回调：如果调用方传了，每算完一块肌肉就回调一次，
        // 把那块肌肉的力暴露给外部观察/记录。默认 nullptr 表示不传也能用。
        // 这是 std::function 的典型用法——把"一段代码"当参数传进去，见 main.cpp 示例。
        Eigen::VectorXd ComputeTotalTorque(
            std::function<void(const Muscle& muscle, double force)> callback = nullptr
        ) const;
        void Step(double dt, const Eigen::VectorXd& external_torque);
        void Reset();
    };
}
