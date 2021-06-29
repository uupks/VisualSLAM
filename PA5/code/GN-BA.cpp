//
// Created by xiang on 12/21/17.
//

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

// #include "sophus/se3.h"
#include "sophus/se3.hpp"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "../p3d.txt";
string p2d_file = "../p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    std::ifstream in_3d_file(p3d_file);
    while (!in_3d_file.eof()) {
        double x, y, z;
        in_3d_file>>x>>y>>z;
        Eigen::Vector3d p(x, y, z);
        p3d.emplace_back(p);
    }

    std::ifstream in_2d_file(p2d_file);
    while (!in_2d_file.eof()) {
        double x, y;
        in_2d_file>>x>>y;
        Eigen::Vector2d p(x, y);
        p2d.emplace_back(p);
    }
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    std::cout << "points: " << nPoints << std::endl;

    // Sophus::SE3 T_esti; // estimated pose
    Sophus::SE3d T_esti;
    std::cout<<T_esti.matrix()<<std::endl;
    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE 
            Eigen::Vector2d e;
            Eigen::Vector3d p_c = (T_esti * p3d[i]);
            Eigen::Vector3d p_uv = K * p_c;
            p_uv = p_uv / p_uv.z();
            e = p2d[i] - Eigen::Vector2d(p_uv.x(), p_uv.y());
            // END YOUR CODE HERE

            // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE 
            J.setZero();
            double x = p_c.x();
            double y = p_c.y();
            double z = p_c.z();
            double z2 = p_c.z() * p_c.z();
            J(0, 0) = -fx / z;
            J(0, 2) = fx * x / z2;
            J(0, 3) = fx * x * y / z2;
            J(0, 4) = -(fx + fx * x * x / z2);
            J(0, 5) = fx * y / z;
            J(1, 1) = -fy / z;
            J(1, 2) = fy * y / z2;
            J(1, 3) = fy + fy * y * y / z2;
            J(1, 4) = -fy * x * y / z2;
            J(1, 5) = -fy * x / z;
            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;

            cost += e.transpose() * e;
        }
        cost = cost / nPoints;
        // solve dx 
        Vector6d dx;

        std::cout << "iteration " << iter << " cost=" << cost << endl;
        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        std::cout<<"dx : "<<dx.transpose()<<std::endl;
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            std::cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            std::cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti =  Sophus::SE3d::exp(dx) * T_esti;

        lastCost = cost;
        // END YOUR CODE HERE
    }

    std::cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
