#include "Muscle.h"
#include <algorithm>
#include <cmath>
#include <iostream>

// ============================================================================
// Muscle.cpp — Muscle 类的实现
// 物理链条：activation → force → torque
//   force  = max_force × activation            (ComputeForce)
//   torque[i] = force × moment_arm[i]          (ComputeJointTorque)
// 见 README.md「数据流」第 2 步。
// ============================================================================

namespace MiniSim{
    Muscle::Muscle(const MuscleInfo& info)
        : mInfo(info), mActivation(0.0)
    {
        std::cout << "[Muscle] Created: " << mInfo.name
                  << "(Fmax=" << mInfo.max_force << "N)" << std::endl;
    }

    // 析构打印日志——教学点：验证 RAII 时机（对象生命周期结束时自动调用）。
    Muscle::~Muscle()
    {
        std::cout << "[Muscle] Destroyed: " << mInfo.name << std::endl;
    }

    void Muscle::SetActivation(double activation)
    {
        // std::clamp（<algorithm>）：把值限制到 [0,1]，调用方传非法值也安全。
        mActivation = std::clamp(activation, 0.0, 1.0);
    }

    double Muscle::ComputeForce() const
    {
        // 最简化的肌肉模型：力 = 最大力 × 激活（线性比例）。
        return mInfo.max_force * mActivation;
    }

    Eigen::VectorXd Muscle::ComputeJointTorque() const
    {
        double force = ComputeForce();
        // 一块肌肉的力矩臂是一个向量（每个 DOF 一个分量），所以力矩也是向量。
        // static_cast<int>：显式把 size_t 转成 int，避免有符号/无符号比较警告。
        int num_dof = static_cast<int>(mInfo.moment_arms.size());
        Eigen::VectorXd torque(num_dof);
        for(int i = 0; i < num_dof; i++){
            // Eigen 的 () 运算符可以多参数访问，[] 只能单参数——这里用 () 更统一。
            // 力矩 = 力 × 力矩臂（标量乘标量）。
            torque(i) = force * mInfo.moment_arms[i];
        }
        return torque;
    }
}
