#pragma once
'''
如果报错，是因为vscode把和.h文件当成C语言头文件，<functional> 是 C++ 标准库头文件。
在 .vscode/settings.json 中添加：
{
    "C_Cpp.default.cppStandard": "c++17",
    "C_Cpp.default.compilerPath": "/usr/bin/g++",
    "C_Cpp.default.intelliSenseMode": "linux-gcc-x64",
    "files.associations": {
        "*.h": "cpp"
    }
}
'''
#include <string>
#include <vector>
/*
<Eigen/Dense>报错，就在.vscode/setting.json里加一句：
    "C_Cpp.default.includePath": [
        "/usr/include/eigen3"
    ],
*/
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
        // const引用：不复制，同时保证不会通过 info 修改原对象
        Muscle(const MuscleInfo& info);
        ~Muscle();

        // activation 会在实现里限制到 [0, 1]，避免调用者传入非法激活值。
        void SetActivation(double activation);

        // 函数前的 const 表示返回的东西只能读不能改；函数后的 const 表示这个函数不会修改成员变量
        const std::string& GetName() const {return mInfo.name; }
        double GetActivation() const {return mActivation; }
        double GetMaxForce() const {return mInfo.max_force; }

        double ComputeForce() const;
        Eigen::VectorXd ComputeJointTorque() const;
    };
}
