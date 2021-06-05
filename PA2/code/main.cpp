#include <iostream>
#include <Eigen/Eigen>

int PA2_5()
{
    Eigen::MatrixXd A = Eigen::MatrixXd::Random(100, 100);
    Eigen::VectorXd b = Eigen::VectorXd::Random(100);

    Eigen::MatrixXd A_qr = A;
    Eigen::MatrixXd b_qr = b;
    Eigen::HouseholderQR<Eigen::MatrixXd> qr;
    qr.compute(A_qr);
    auto x_qr = qr.solve(b_qr);

    Eigen::MatrixXd A_ldlt = A;
    Eigen::MatrixXd b_ldlt = b;
    Eigen::LDLT<Eigen::MatrixXd> ldlt;
    ldlt.compute(A_ldlt);
    auto x_ldlt = ldlt.solve(b_ldlt);

    std::cout<<"x_qr : "<<x_qr.transpose()<<std::endl;
    std::cout<<"x_ldlt : "<<x_ldlt.transpose()<<std::endl;

    return 0;
}

int PA3()
{
    // 构造函数 q(w, x, y, z)
    Eigen::Quaterniond q1(0.55, 0.3, 0.2, 0.2);
    q1.normalize();
    Eigen::Vector3d t1(0.7, 1.1, 0.2);

    Eigen::Quaterniond q2(-0.1, 0.3, -0.7, 0.2);
    q2.normalize();
    Eigen::Vector3d t2(-0.1, 0.4, 0.8);

    Eigen::Vector3d p1(0.5, -0.1, 0.2);

    Eigen::Matrix4d T_c1_w = Eigen::Matrix4d::Identity();
    T_c1_w.block<3,3>(0,0) = q1.normalized().toRotationMatrix();
    T_c1_w.block<3,1>(0,3) = t1;

    Eigen::Matrix4d T_c2_w = Eigen::Matrix4d::Identity();
    T_c2_w.block<3,3>(0,0) = q2.normalized().toRotationMatrix();
    T_c2_w.block<3,1>(0,3) = t2;

    Eigen::Vector4d p(p1.x(), p1.y(), p1.z(), 1);
    auto p_w = T_c1_w.inverse() * p;
    auto p_c2 = T_c2_w * p_w;

    std::cout<<"P2 : "<<p_c2.block<3,1>(0,0).transpose()<<std::endl;
    return 0;
}

int main(int argc, char *argv[])
{
    std::cout<<"##### PA 2-5 #####"<<std::endl;
    PA2_5();
    std::cout<<std::endl;

    std::cout<<"##### PA 3 #####"<<std::endl;
    PA3();
    std::cout<<std::endl;

    return 0;
}
