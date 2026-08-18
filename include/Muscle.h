#pragma once
// ============================================================================
// Muscle.h — 最底层模块：一块肌肉
// ----------------------------------------------------------------------------
// 职责：把"激活水平"转成"肌肉力"再转成"关节力矩"。
//       力 = 最大力 × 激活；力矩 = 力 × 力矩臂。
// 在架构中的位置：最底层。不依赖 Character/Environment，被 Character 持有。
// 见 README.md「架构总览」。
// ----------------------------------------------------------------------------
// IDE 配置提示（首次配置时看一次即可）：
//   如果 VSCode 把 .h 当 C 头文件导致 <functional> 报错，在 .vscode/settings.json 设：
//     "C_Cpp.default.cppStandard": "c++17",
//     "files.associations": { "*.h": "cpp" }
//   如果 <Eigen/Dense> 找不到，加 include 路径：
//     "C_Cpp.default.includePath": ["/usr/include/eigen3"]
// ============================================================================

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace MiniSim{

    // struct vs class：struct 默认成员 public，适合放纯数据；class 默认 private。
    // MuscleInfo 只是数据包（名称+最大力+力矩臂），没有任何行为，所以用 struct。
    struct MuscleInfo
    {
        std::string name;
        double max_force;
        std::vector<double> moment_arms;

        // const T& 传参：不复制对象（string/vector 可能很大），同时保证不修改原对象。
        // 冒号后是成员初始化列表，直接构造成员，比先默认构造再赋值更高效。
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

        // activation 会被 clamp 到 [0,1]，调用方传非法值也安全。
        void SetActivation(double activation);

        // 函数后的 const：这个函数不会修改成员变量。
        // 返回 const&：不复制（string 较大），且禁止调用方通过返回值改成员。
        const std::string& GetName() const {return mInfo.name; }
        double GetActivation() const {return mActivation; }
        double GetMaxForce() const {return mInfo.max_force; }

        double ComputeForce() const;
        Eigen::VectorXd ComputeJointTorque() const;
    };
}
