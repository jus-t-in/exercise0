#include <iostream>
#include <cmath>
#include "Muscles.h"

namespace MiniSim{
    Muscle::Muscle(const MuscleInfo& info)
        : mInfo(info), mActivation(0.0)
    {
        std::cout << "[Muscle] Create: " << mInfo.name
                << "(Fmax= " << mInfo.max_force << "N)" << std::endl;
    }
    Muscle::~Muscle()
    {
        std::cout << "[Muscle] Destoryed: " << mInfo.name << std::endl;
    }
    void Muscle::SetActivation(double activation)
    {
        mActivation = std::clamp(activation, 0.0, 1.0);
    }
    double Muscle::ComputeForce() const
    {
        return mInfo.max_force * mActivation;
    }
    Eigen::VectorXd Muscle::ComputeJointTorque() const
    {
        double force = ComputeForce();
        int num_dof = static_cast<int>(mInfo.moment_arms.size());
        Eigen::VectorXd torque(num_dof);
        for (int i = 0; i < num_dof; i++)
        {
            torque[i] = force * mInfo.moment_arms[i];
        }
        return torque;
    }
}