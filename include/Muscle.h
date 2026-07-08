#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

// namespace 用来避免类名和其他库/文件里的名字冲突。
namespace MiniSim{

    // struct 默认成员是 public，适合放纯数据；class 默认成员是 private。
    struct MuscleInfo
    {
        std::string name;
        double max_force;
        std::vector<double> moment_arms;

        // const T& 表示只读引用，避免复制 vector/string 这类可能较大的对象。
        // 冒号后面是成员初始化列表，构造对象时直接初始化成员，通常比先默认构造再赋值更好。
        MuscleInfo(const std::string& name, double max_force,
                   const std::vector<double>& moment_arms)
            : name(name), max_force(max_force), moment_arms(moment_arms) {}
    };

    class Muscle
    {
    private:
        MuscleInfo mInfo;
        double mActivation;
    public:
        Muscle(const MuscleInfo& info);
        ~Muscle();

        // activation 会在实现里限制到 [0, 1]，避免调用者传入非法激活值。
        void SetActivation(double activation);

        // 返回 const 引用避免复制字符串；函数后的 const 表示这个函数不会修改对象。
        const std::string& GetName() const {return mInfo.name; }
        double GetActivation() const {return mActivation; }
        double GetMaxForce() const {return mInfo.max_force; }

        double ComputeForce() const;
        Eigen::VectorXd ComputeJointTorque() const;
    };
}
