cmake_minimum_required(VERSION 3.10)
project(7Dof_Kinematics_iiwa_Rokae)

# 使用 C++17
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 打开警告信息
add_compile_options(-Wall -Wextra -Wpedantic)

# 查找 Eigen 库
find_package(Eigen3 REQUIRED)

# 包含头文件目录
include_directories(${EIGEN3_INCLUDE_DIRS} include)

# 源文件
set(SRC
    src/SRSAnalyticalSolver.cpp
    src/KUKAiiwaSolver.cpp
    src/main.cpp
)

# 生成可执行文件
add_executable(kinematics_solver ${SRC})

# 链接 Eigen
target_link_libraries(kinematics_solver Eigen3::Eigen)
