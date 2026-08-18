#include "Character.h"
#include <iostream>
#include <stdexcept>

// ============================================================================
// Character.cpp — Character 类的实现
// 职责：管理关节状态 + 肌肉列表，用半隐式欧拉积分推进一步物理。
// 物理链条（见 README.md「数据流」）：
//   SetMuscleActivations → ComputeTotalTorque → acceleration → Step 积分
// ============================================================================

namespace MiniSim{
    Character::Character(int num_dof, const Eigen::VectorXd& mass)
        : mNumDOF(num_dof),
          mPositions(Eigen::VectorXd::Zero(num_dof)),
          mVelocities(Eigen::VectorXd::Zero(num_dof)),
          mMass(mass)
    {
        // 维度检查尽早发现不一致，比后面 Eigen 计算时报错更容易定位。
        if (mass.size() != num_dof){
            throw std::invalid_argument("mass size must match num_dof");
        }
        std::cout << "[Character] Created with " << num_dof << " DOF" << std::endl;
    }

    Character::~Character()
    {
        std::cout << "[Character] Destroying... cleaning up "
                  << mMuscles.size() << " muscles" << std::endl;
        // RAII 教学点：Character 拥有这些 Muscle 指针，析构时必须手动释放。
        // 因为 AddMuscle 里用了 new，这里就得配对 delete，否则内存泄漏。
        for (auto muscle : mMuscles){
            delete muscle;
        }
    }

    void Character::AddMuscle(const MuscleInfo& info)
    {
        // 每块肌肉的力矩臂长度必须等于 DOF 数（一个 DOF 一个分量）。
        if (static_cast<int>(info.moment_arms.size()) != mNumDOF){
            std::cerr << "[Character] ERROR: Muscle '" << info.name
                      << "' has " << info.moment_arms.size() << " moment arms,"
                      << " but Character has " << mNumDOF << " DOF!" << std::endl;
            return;
        }
        // new 一块 Muscle，把指针存进列表。所有权归 Character（见析构函数）。
        mMuscles.push_back(new Muscle(info));
    }

    void Character::SetMuscleActivation(int index, double activation)
    {
        if (index < 0 || index >= static_cast<int>(mMuscles.size())){
            throw std::out_of_range("muscle index out of range");
        }
        mMuscles[index]->SetActivation(activation);
    }

    void Character::SetMuscleActivations(const Eigen::VectorXd& activations)
    {
        if (activations.size() != static_cast<int>(mMuscles.size())){
            throw std::invalid_argument("activation size must match muscle count");
        }
        for (int i = 0; i < activations.size(); i++){
            SetMuscleActivation(i, activations(i));
        }
    }

    Eigen::VectorXd Character::ComputeTotalTorque(
        std::function<void(const Muscle& muscle, double force)> callback
    ) const
    {
        // 累加所有肌肉的关节力矩 → 总力矩。这是"肌肉力 → 关节力矩"的汇总步骤。
        Eigen::VectorXd total_torque = Eigen::VectorXd::Zero(mNumDOF);
        for (const auto& muscle : mMuscles){
            double force = muscle->ComputeForce();
            total_torque += muscle->ComputeJointTorque();
            // 如果调用方传了回调，把每块肌肉的力暴露出去，便于观察或记录日志。
            // 这就是 std::function 回调的用法——不侵入计算逻辑，但允许外部观察。
            if(callback){
                callback(*muscle, force);
            }
        }
        return total_torque;
    }

    void Character::Step(double dt, const Eigen::VectorXd& external_torque)
    {
        if (external_torque.size() != mNumDOF){
            throw std::invalid_argument("external_torque size must match num_dof");
        }

        // 1. 肌肉总力矩（依赖当前激活，激活由 SetMuscleActivations 设好）
        Eigen::VectorXd muscle_torque = ComputeTotalTorque();
        // 2. 总力矩 = 肌肉力矩 + 外部力矩
        Eigen::VectorXd total_torque = muscle_torque + external_torque;
        // 3. 角加速度 = 力矩 / 转动惯量（这里把 mass 近似成转动惯量）
        Eigen::VectorXd acceleration(mNumDOF);
        for (int i = 0; i < mNumDOF; i++){
            acceleration(i) = total_torque(i) / mMass(i);
        }
        // 4. 半隐式欧拉积分：先用加速度更新速度，再用【新速度】更新位置。
        //    比显式欧拉稳定，是物理仿真常用做法。
        mVelocities += acceleration * dt;
        mPositions += mVelocities * dt;
        // 5. 阻尼：每步把速度乘 0.99，防止系统能量发散。
        mVelocities *= 0.99;
    }

    void Character::Reset()
    {
        mVelocities = Eigen::VectorXd::Zero(mNumDOF);
        mPositions = Eigen::VectorXd::Zero(mNumDOF);
        for(auto& muscle : mMuscles){
            // muscle 是指针，所以用 ->（等价于 (*muscle).SetActivation）。
            muscle->SetActivation(0.0);
        }
    }
}
