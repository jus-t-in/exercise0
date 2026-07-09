#include "Character.h"
#include <iostream>
#include <stdexcept>

namespace MiniSim{
    Character::Character(int num_dof, const Eigen::VectorXd& mass)
       :mNumDOF(num_dof),
        mPositions(Eigen::VectorXd::Zero(num_dof)),
        mVelocities(Eigen::VectorXd::Zero(num_dof)),
        mMass(mass)
    {
        // 检查维度对齐
        if(mass.size() != num_dof){
            throw std::invalid_argument("mass size must match num_dof");
        }
        std::cout << "[Character] Created with:" << num_dof << " DOF" << std::endl;
    }
    Character::~Character()
    {
        std::cout << "[Character] Destorying... cleaning up"
                  << mMuscles.size() << "muscles" << std::endl;
        // mMuscles有这些Muscle的指针，析构时需要释放
        for (auto muscle : mMuscles){
            delete muscle;
        }
    }
    void Character::AddMuscle(const MuscleInfo& info)
    {
        if (static_cast<int>(info.moment_arms.size()) != mNumDOF){
            std::cerr << "[Character] ERROR: Mucles '" << info.name
                      << "' has" << info.moment_arms.size() << "moment arms,"
                      << "but Character has" << mNumDOF << " DOF!" << std::endl;
            return;  // 直接结束函数，不执行后续代码
        }
        // 创建一块新的Muscle,然后把它的地址放到mMuscles这个数组里
        mMuscles.push_back(new Muscle(info));  // push_back把这肌肉存入角色的肌肉列表 
    }
    void Character::SetMuscleActivation(int index, double activation)
    {
        if (index < 0 || index >= static_cast<int>(mMuscles.size())){
            throw std::invalid_argument("activation size must match muscle count");
        }
        mMuscles[index]->SetActivation(activation);
    }
    void Character::SetMuscleActivations(const Eigen::VectorXd& activations)
    {
        if (activations.size() != static_cast<int>(mMuscles.size())){
            throw std::invalid_argument("activation size must match muscle count");
        }
        for (int i = 0; i < activations.size(); i++)
        {
            SetMuscleActivation(i, activations(i));
        }
    }
    Eigen::VectorXd Character::ComputeTotalTorque(
            std::function<void(const Muscle& muscle, double force)> callback 
        ) const
    {
        Eigen::VectorXd total_torque = Eigen::VectorXd::Zero(mNumDOF);
        for (const auto& muscle : mMuscles)
        {
            double force = muscle->ComputeForce();
            total_torque += muscle->ComputeJointTorque();
            if (callback){
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
        Eigen::VectorXd muscle_torque = ComputeTotalTorque();
        Eigen::VectorXd total_torque = muscle_torque + external_torque;
        Eigen::VectorXd acceleration(mNumDOF);
        for (int i = 0; i < mNumDOF; i++){
            acceleration(i) = total_torque(i) / mMass(i);
        }
        // 半隐式欧拉，用加速度更新速度，再用新速度更新位置
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
