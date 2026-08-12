#pragma once
#include <functional>
#include <vector>
#include <Eigen/Dense>
#include "Muscle.h"

namespace MiniSim{
    // Character 管理关节状态和肌肉列表，是仿真主体。
    class Character
    {
    private:
        int mNumDOF;
        Eigen::VectorXd mPositions;
        Eigen::VectorXd mVelocities;
        Eigen::VectorXd mMass;
        // 本练习用裸指针演示 RAII：AddMuscle 里 new，析构函数里 delete。
        std::vector<Muscle*> mMuscles;
    public:
        Character(int num_dof, const Eigen::VectorXd& mass);
        ~Character();
        void AddMuscle(const MuscleInfo& info);
        int GetNumMuscles() const {return static_cast<int>(mMuscles.size());}
        // virtual 表示子类以后可以重写这个函数；这里先作为继承语法示例。
        virtual int GetDOF() const {return mNumDOF; }
        // 这里用'&'，因为Eigen内部是动态分配的堆内存
        // 拷贝 = 重新 new 一块内存 + 逐个元素复制，DOF 大时很贵
        const Eigen::VectorXd& GetPositions() const {return mPositions; }
        const Eigen::VectorXd& GetVelocities() const {return mVelocities; }
        void SetPositions(const Eigen::VectorXd& pos) { mPositions = pos; }
        void SetVelocities(const Eigen::VectorXd& vel) { mVelocities = vel; }
        void SetMuscleActivation(int index, double activation);
        // 为了避免类型转换，没用vector类型
        void SetMuscleActivations(const Eigen::VectorXd& activations);
        // 这个function就是个回调写法，不侵入计算逻辑，但允许外部观察（可观察每块肌肉的力）
        // std::function 可以保存普通函数、lambda、函数对象；默认 nullptr 表示不传回调也能调用。
        Eigen::VectorXd ComputeTotalTorque(
            std::function<void(const Muscle& muscle, double force)> callback = nullptr
        ) const;
        void Step(double dt, const Eigen::VectorXd& external_torque);
        void Reset();
    };
}
