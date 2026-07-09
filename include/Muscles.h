#include <string>
#include <vector>
#include <Eigen/Dense>


namespace MiniSim{
    struct MuscleInfo
    {
        double max_force;
        std::string name;
        std::vector<double> moment_arms;
        MuscleInfo(const std::string& name, 
                const double& max_force,
                std::vector<double> moment_arms)
                : name(name), max_force(max_force), moment_arms(moment_arms){}
    };
    class Muscle
    {
    private:
        MuscleInfo mInfo;
        double mActivation;
    public:
        Muscle(const MuscleInfo& info);
        ~Muscle();
        void SetActivation(double activation);
        const std::string& GetName() const {return mInfo.name; }
        double GetActivation() const {return mActivation; }
        double GetMaxForce() const {return mInfo.max_force; }
        double ComputeForce() const;
        Eigen::VectorXd ComputeJointTorque() const;  
    };
    
}