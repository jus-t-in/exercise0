#pragma once
#include "Muscles.h"
#include <vector>
#include <Eigen/Dense>
#include <functional>

namespace MiniSim{
    class Character
    {
    private:
        int mNumDOF;
        Eigen::VectorXd mPositions;
        Eigen::VectorXd mVelocities;
        Eigen::VectorXd mMass;
        std::vector<Muscle*> mMuscles;
    public:
        Character(int num_dof, const Eigen::VectorXd& mass);
        ~Character();
        void AddMuscle(const MuscleInfo& info);
        int GetNumMuscles() const {return static_cast<int>(mMuscles.size());}

        virtual int GetDOF() const {return mNumDOF;}
        const Eigen::VectorXd& GetPositions() const {return mPositions;}
        const Eigen::VectorXd& GetVelocities() const {return mVelocities;}
        void SetPositions(const Eigen::VectorXd& pos) {mPositions = pos;}
        void SetVelosities(const Eigen::VectorXd& vel) {mVelocities = vel;}

        void SetMuscleActivation(int index, double activation);
        void SetMuscleActivations(const Eigen::VectorXd& activation);
        Eigen::VectorXd ComputeTotalTorque(
            std::function<void(const Muscle& muscle, double force)> callback = nullptr
        ) const;
        void Step(double dt, const Eigen::VectorXd& external_torque);
        void Reset();
    };    
}

