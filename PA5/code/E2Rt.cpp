//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#define PI 3.1415926

#define r2a(x) ((x)*180/PI)

#define a2r(x) ((x)*PI/180)

using namespace Eigen;

#include <sophus/so3.hpp>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(E, Eigen::ComputeThinU | Eigen::ComputeThinV);
    auto U = svd.matrixU();
    cout<<"U\n"<<U<<endl;
    auto V = svd.matrixV();
    cout<<"V\n"<<V<<endl;
    auto A = svd.singularValues();
    cout<<"A\n"<<A<<endl;
    
    double v = (A.x() + A.y()) / 2.0;
    Eigen::Matrix3d diag = Eigen::Vector3d(v, v, 0).asDiagonal();
    cout<<"Diag\n"<<diag<<endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;

    Matrix3d R1;
    Matrix3d R2;
    double angle_z = 90;

    Eigen::AngleAxisd RZ_1 = Eigen::AngleAxisd(a2r(angle_z), Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd RZ_2 = Eigen::AngleAxisd(a2r(-angle_z), Eigen::Vector3d::UnitZ());

    t_wedge1 = U * RZ_1 * diag * U.transpose();
    t_wedge2 = U * RZ_2 * diag * U.transpose();
    R1 = U * RZ_1.toRotationMatrix().transpose() * V.transpose();
    R2 = U * RZ_2.toRotationMatrix().transpose() * V.transpose();
    // END YOUR CODE HERE

    cout << "R1 = \n" << R1 << endl;
    cout << "R2 = \n" << R2 << endl;
    cout << "t1 = \n" << Sophus::SO3d::vee(t_wedge1) << endl;
    cout << "t2 = \n" << Sophus::SO3d::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = \n" << tR << endl;

    return 0;
}