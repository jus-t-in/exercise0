# CMakeLists.txt — 构建配置
# 产出两个目标：
#   simcore  静态库 = Muscle + Character + Environment
#   mini_sim 可执行文件 = main，链接 simcore
# 见 README.md「文件结构」。

cmake_minimum_required(VERSION 3.16)
project(mini_musculoskeletal_sim LANGUAGES CXX)

# C++17，要求标准合规（不支持就报错，不悄悄降级），关闭 GNU 扩展（便于跨平台）。
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# 导出 compile_commands.json，VSCode C/C++ 扩展用它做准确补全。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Eigen 是头文件库（矩阵/向量计算），找不到就停止配置。
find_package(Eigen3 REQUIRED)

# 核心仿真代码编成 simcore 库，main 单独编译再链接它——库可以被多个可执行文件复用。
add_library(simcore
    src/Muscle.cpp
    src/Character.cpp
    src/Environment.cpp
)

# PUBLIC：include 路径既给 simcore 自己用，也会传递给链接 simcore 的目标（如 mini_sim）。
target_include_directories(simcore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Eigen3::Eigen 是 find_package 提供的 imported target，携带 Eigen 的 include 路径。
# PUBLIC 的理由同上：下游目标也间接依赖 Eigen 的头文件。
target_link_libraries(simcore PUBLIC Eigen3::Eigen)

# 最终可运行程序：只编译 main.cpp，靠链接 simcore 拿到所有仿真实现。
add_executable(mini_sim src/main.cpp)
# PRIVATE：mini_sim 自己链接 simcore，但不再把这个依赖传递给其他目标。
target_link_libraries(mini_sim PRIVATE simcore)
