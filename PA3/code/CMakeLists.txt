cmake_minimum_required(VERSION 3.10)

project(trajectory)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

add_executable(${PROJECT_NAME}
    draw_trajectory.cpp
)

target_link_libraries(${PROJECT_NAME}
    pangolin
    Sophus::Sophus
)