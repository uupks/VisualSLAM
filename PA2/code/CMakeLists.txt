cmake_minimum_required(VERSION 3.10)
project(PA2)

find_package(Eigen3 REQUIRED)

add_executable(pa2_solution main.cpp)
target_link_libraries(pa2_solution
    Eigen3::Eigen
)
