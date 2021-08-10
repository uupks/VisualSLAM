cmake_minimum_required(VERSION 3.2)

project(PA7)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(g2o REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Ceres REQUIRED)

add_executable(directBA
    directBA.cpp
)

target_link_libraries(directBA
    PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen Sophus::Sophus pangolin g2o::core g2o::solver_dense g2o::types_slam3d
)

add_executable(bal_example
    bal_example.cpp
)

target_link_libraries(bal_example
    PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen Sophus::Sophus pangolin g2o::core g2o::solver_eigen g2o::solver_cholmod
)

add_executable(bal_example_ceres.cpp
    bal_example_ceres.cpp
)
target_link_libraries(bal_example_ceres.cpp
    PUBLIC ${OpenCV_LIBRARIES} Eigen3::Eigen pangolin ${CERES_LIBRARIES}
)