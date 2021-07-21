cmake_minimum_required(VERSION 3.2)

project(PA7)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(Pangolin REQUIRED)

add_executable(directBA
    directBA.cpp
)

target_link_libraries(directBA
    PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen Sophus::Sophus pangolin g2o::core g2o::stuff
)