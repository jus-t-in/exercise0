# CMakeLists.txt 是 CMake 的入口文件，描述“怎么把源码编译成目标”。
cmake_minimum_required(VERSION 3.16)

# project 定义项目名；LANGUAGES CXX 表示这里只启用 C++ 编译器。
project(mini_musculoskeletal_sim LANGUAGES CXX)

# 要求所有目标默认使用 C++17。
set(CMAKE_CXX_STANDARD 17)
# 如果编译器不支持 C++17，直接报错，而不是悄悄降级。
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 关闭 GNU 扩展，尽量使用标准 C++，便于跨平台。
set(CMAKE_CXX_EXTENSIONS OFF)
# 生成 build/compile_commands.json，VSCode C/C++ 扩展会用它做准确补全。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# REQUIRED 表示找不到 Eigen3 就停止配置；Eigen 是头文件库，常用于矩阵/向量计算。
find_package(Eigen3 REQUIRED)

# add_library 创建一个库目标。这里把核心仿真代码编成 simcore，main 再链接它。
add_library(simcore
    src/Muscle.cpp
    src/Character.cpp
    src/Environment.cpp
)

# PUBLIC 表示 include 路径既给 simcore 自己用，也会传递给链接 simcore 的目标。
target_include_directories(simcore PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# Eigen3::Eigen 是 find_package 提供的 imported target，会携带 Eigen 的 include 路径。
target_link_libraries(simcore PUBLIC Eigen3::Eigen)

# add_executable 创建最终可运行程序 mini_sim。
add_executable(mini_sim src/main.cpp)
# PRIVATE 表示 mini_sim 自己链接 simcore，但不再把这个依赖传递给其他目标。
target_link_libraries(mini_sim PRIVATE simcore)
