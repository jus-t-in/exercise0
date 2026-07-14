#include "Environment.h"
#include <iostream>
#include <cmath>

int main()
{
    using namespace MiniSim;  // 
    Eigen::VectorXd mass(2);
    mass << 1.0, 1.0;

    auto character = std::make_shared<Character>(2, mass);
    character->AddMuscle(MuscleInfo("hip_flexor", 120.0, {0.05, 0.00}));
    character->AddMuscle(MuscleInfo("hip_extensor", 120.0, {-0.05, 0.00}));
    character->AddMuscle(MuscleInfo("knee_flexor", 80.0, {0.00, 0.04}));
    character->AddMuscle(MuscleInfo("knee_extensor", 80.0, {0.00, -0.04}));

    Environment env(character, 100, 0.01);
    Eigen::VectorXd target(2);
    target << 0.2, -0.1;
    env.SetTargetPositions(target);
    env.Reset();

    std::vector<double> force_log;
    auto observe_force = [&force_log](const Muscle& muscle, double force){
        force_log.push_back(force);
        std::cout << "  " << muscle.GetName() << "force = " << force << std::endl;
    };

    for (int step = 0; step < 100 && !env.IsEnd(); step++){
        Eigen::VectorXd action(character->GetNumMuscles());
        action(0) = 0.5 + 0.5 * std::sin(0.1 * step);
        action(1) = 1.0 - action(0);
        action(2) = 0.5 + 0.5 * std::cos(0.1 * step);
        action(3) = 1.0 -action(2);

        double reward = env.Step(action);
        Eigen::VectorXd state = env.GetStates();

        std::cout << "step = " << step << "state = " << state.transpose() 
                  << "reward = " << reward << std::endl;
        character->ComputeTotalTorque(observe_force);
    }
    std::cout << "logged forces:" << force_log.size() << std::endl;
    return 0;
}
