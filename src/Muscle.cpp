#include "Muscle.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace MiniSim{
    Muscle::Muscle(const MuscleInfo& info)
        : mInfo(info), mActivation(0.0)
    {
        std::cout << "[Muscle] Created: " << mInfo.name
                  << "(Fmax=" << mInfo.max_force << "N)" << std::endl;
    }

    Muscle::~Muscle()
    {
        std::cout << "[Muscle] Destroyed: " << mInfo.name << std::endl;
    }

    void Muscle::SetActivation(double activation)
    {
        // std::clamp 来自 <algorithm>，把值限制在指定范围内。
        mActivation = std::clamp(activation, 0.0, 1.0);
    }

    double Muscle::ComputeForce() const
    {
        return mInfo.max_force * mActivation;
    }

    Eigen::VectorXd Muscle::ComputeJointTorque() const
    {
        double force = ComputeForce();
        // 一个肌肉拉力可能对多个关节自由度产生力矩，所以用数组
        // static_cast<int> 显式类型转换：把 size_t 转成 int
        int num_dof = static_cast<int>(mInfo.moment_arms.size());
        Eigen::VectorXd torque(num_dof);
        for(int i = 0; i < num_dof; i++){
            // C++ 的 [] 运算符只能有一个参数，() 可以有多个
            // Eigen用 () 更灵活            
            torque(i) = force * mInfo.moment_arms[i];
        }
        return torque;
    }
}
