#include <fstream>
#include <string>
#include <iostream>
#include <vector>
#include <thread>
#include <unistd.h>

#include <Eigen/Eigen>
#include "pangolin/pangolin.h"
#include "sophus/se3.hpp"


using namespace std;
using namespace pangolin;

using Poses = std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;

void Draw2Trajectory(Poses& pose1, Poses& pose2);


int main(int argc, char **argv)
{
    /* code */
    std::string traj_file;
    if (argc == 2) {
        traj_file = argv[1];
    } else {
        std::cout<<"Usage : "<<argv[0]<<" <trajectory file>\n";
        return -1;
    }

    Poses est_poses;
    Poses gt_poses;

    std::ifstream in_file(traj_file);
    while(!in_file.eof()) {
        double te, tex, tey, tez, qex, qey, qez, qew, tg, tgx, tgy, tgz, qgx, qgy, qgz, qgw;
        in_file>>te>>tex>>tey>>tez>>qex>>qey>>qez>>qew>>tg>>tgx>>tgy>>tgz>>qgx>>qgy>>qgz>>qgw;
        Eigen::Quaterniond q(qew, qex, qey, qez);
        q.normalize();
        Sophus::SE3d est_pose(q, Eigen::Vector3d(tex, tey, tez));
        est_poses.push_back(est_pose);

        Eigen::Quaterniond q_gt(qgw, qgx, qgy, qgz);
        q_gt.normalize();
        Sophus::SE3d gt_pose(q_gt, Eigen::Vector3d(tgx, tgy, tgz));
        gt_poses.push_back(gt_pose);
    }
    assert(est_poses.size() == gt_poses.size());

    std::thread th(Draw2Trajectory, std::ref(est_poses), std::ref(gt_poses));
    // Draw2Trajectory(est_poses, gt_poses);

    int T = 300;
    int N = est_poses.size();
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double last_cost = 0;
    for (size_t i = 0; i < T; i++) {
        // 质心坐标
        Eigen::Vector3d p_est_center, p_gt_center;
        for (size_t i = 0; i < N; i++) {
            p_est_center += est_poses[i].translation();
            p_gt_center += gt_poses[i].translation();
        }
        p_est_center = p_est_center / N;
        p_gt_center = p_gt_center / N;
        cout << "p_est_center : "<<p_est_center.transpose()<<endl;
        cout << "p_gt_center : "<<p_gt_center.transpose()<<endl;
        // 构造W矩阵
        Eigen::Matrix3d W;
        for (size_t i = 0; i < N; i++) {
            Eigen::Vector3d p_est = est_poses[i].translation() - p_est_center;
            Eigen::Vector3d p_gt = gt_poses[i].translation() - p_gt_center;
            W += p_gt * p_est.transpose();
        }
        cout <<"W \n"<<W<<endl;
        // SVD分解
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(W, Eigen::ComputeThinU | Eigen::ComputeThinV);
        auto U = svd.matrixU();
        auto V = svd.matrixV();
        R = U * V.transpose();
        cout <<"R \n"<<R<<endl;
        t = p_gt_center - R * p_est_center;
        cout <<"t : "<<t.transpose()<<endl;

        double error = 0;
        double cost = 0;
        for (size_t i = 0; i < N; i++) {
            Eigen::Vector3d t_icp = R * est_poses[i].translation() + t;
            est_poses[i].translation() = t_icp;
            error += (gt_poses[i].translation() - est_poses[i].translation()).norm();
        }
        cost = error / N;
        cout <<"cost : "<<cost<<endl;
        if ((i > 0 && cost > last_cost) || (cost < 0.05)) {
            std::cout << "cost: " << cost << ", last cost: " << last_cost << endl;
            break;
        }
        last_cost = cost;
    }
    getchar();
    pangolin::Quit();
    return 0;
}


void Draw2Trajectory(Poses& pose1, Poses& pose2)
{
    if (pose1.empty() || pose2.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));


    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glLineWidth(2);
        for (size_t i = 0; i < pose1.size() - 1; i++) {
            glColor3f(0.0, 0.0, 1.0);
            glBegin(GL_LINES);
            auto p1 = pose1[i], p2 = pose1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }

        for (size_t i = 0; i < pose2.size(); i++) {
            glColor3f(0.0, 1.0, 0.0);
            glBegin(GL_LINES);
            auto p1 = pose2[i], p2 = pose2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}