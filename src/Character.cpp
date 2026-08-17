#include "Character.h"
#include <iostream>
#include <stdexcept>

namespace MiniSim{
    Character::Character(int num_dof, const Eigen::VectorXd& mass)
        : mNumDOF(num_dof),
          mPositions(Eigen::VectorXd::Zero(num_dof)),
          mVelocities(Eigen::VectorXd::Zero(num_dof)),
          mMass(mass)
    {
        // 运行时检查能尽早发现维度不一致的问题，比后面 Eigen 计算时报错更容易定位。
        if (mass.size() != num_dof){
            throw std::invalid_argument("mass size must match num_dof");
        }
        std::cout << "[Character] Created with " << num_dof << " DOF" << std::endl;
    }

    Character::~Character()
    {
        std::cout << "[Character] Destroying... cleaning up "
                  << mMuscles.size() << " muscles" << std::endl;
        // Character 拥有这些 Muscle 指针，所以析构时负责释放，避免内存泄漏。
        for (auto muscle : mMuscles){
            delete muscle;
        }
    }

    void Character::AddMuscle(const MuscleInfo& info)
    {
        if (static_cast<int>(info.moment_arms.size()) != mNumDOF){
            std::cerr << "[Character] ERROR: Muscle '" << info.name
                      << "' has " << info.moment_arms.size() << " moment arms,"
                      << " but Character has " << mNumDOF << " DOF!" << std::endl;
            return;
        }
        // 创建一块Muscle，然后把它的地址放到mMuscles这个数组里
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
        Eigen::VectorXd total_torque = Eigen::VectorXd::Zero(mNumDOF);
        for (const auto& muscle : mMuscles){
            double force = muscle->ComputeForce();
            total_torque += muscle->ComputeJointTorque();
            // 如果调用方传了回调，就把每块肌肉的力暴露出去，便于观察或记录日志。
            if(callback){
                callback(*muscle, force);
            }
        }
        return total_torque;
    }

    void Character::Step(double dt, const Eigen::VectorXd& external_torque)
    {
        // 比较操作允许混合类型，所以这里不用转int
        if (external_torque.size() != mNumDOF){
            throw std::invalid_argument("external_torque size must match num_dof");
        }

        Eigen::VectorXd muscle_torque = ComputeTotalTorque();
        Eigen::VectorXd total_torque = muscle_torque + external_torque;
        Eigen::VectorXd acceleration(mNumDOF);
        for (int i = 0; i < mNumDOF; i++){
            // 角加速度T = I·α，I近似成mass
            acceleration(i) = total_torque(i) / mMass(i);
        }
        // 半隐式欧拉：先用加速度更新速度，再用新速度更新位置。
        mVelocities += acceleration * dt;
        mPositions += mVelocities * dt;
        mVelocities *= 0.99;
    }

    void Character::Reset()
    {
        mVelocities = Eigen::VectorXd::Zero(mNumDOF);
        mPositions = Eigen::VectorXd::Zero(mNumDOF);
        for(auto& muscle : mMuscles){
            muscle->SetActivation(0.0);
        }
    }
}
