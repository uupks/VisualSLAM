#include <sophus/se3.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <sstream>
// need pangolin for plotting trajectory
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file

using Poses = vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>;
// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(Poses& poses);
void Draw2Trajectory(Poses& pose1, Poses& pose2);
double CalcRMSE(Poses& ground_truth, Poses& estimated);

int main(int argc, char **argv) {

    Poses poses;
    string trajectory_file = "./trajectory.txt";
    string ground_truth_file, estimated_file;
    /// implement pose reading code
    // start your code here (5~10 lines)
    if (argc == 2) {
        trajectory_file = argv[1];
        cout<<"Trajectory file : "<<trajectory_file<<endl;
        ifstream in_file(trajectory_file);
        if(in_file.is_open()) {
            while (!in_file.eof()) {
                double timestamp, tx, ty, tz, qx, qy, qz, qw;
                in_file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
                Eigen::Quaterniond q(qw, qx, qy, qz);
                q.normalize();
                Sophus::SE3d pose(q, Eigen::Vector3d(tx, ty, tz));
                poses.push_back(pose);
            }
        }
        // end your code here
        // draw trajectory in pangolin
        DrawTrajectory(poses);
    } else if (argc == 3) {
        ground_truth_file = argv[1];
        estimated_file = argv[2];
        Poses ground_truth, est_poses;
        cout << "Ground truth file : " << ground_truth_file << endl;
        cout << "Estimated file : " << estimated_file << endl;
        ifstream gt_file(ground_truth_file);
        while(!gt_file.eof()) {
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            gt_file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Eigen::Quaterniond q(qw, qx, qy, qz);
            q.normalize();
            Sophus::SE3d pose(q, Eigen::Vector3d(tx, ty, tz));
            ground_truth.push_back(pose);
        }

        ifstream est_file(estimated_file);
        while(!est_file.eof()) {
            double timestamp, tx, ty, tz, qx, qy, qz, qw;
            est_file >> timestamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Eigen::Quaterniond q(qw, qx, qy, qz);
            q.normalize();
            Sophus::SE3d pose(q, Eigen::Vector3d(tx, ty, tz));
            est_poses.push_back(pose);
        }
        CalcRMSE(ground_truth, est_poses);
        Draw2Trajectory(est_poses, ground_truth);
    }
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>& poses) {
    if (poses.empty()) {
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
        for (size_t i = 0; i < poses.size() - 1; i++) {
            glColor3f(1 - (float) i / poses.size(), 0.0f, (float) i / poses.size());
            glBegin(GL_LINES);
            auto p1 = poses[i], p2 = poses[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
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


double CalcRMSE(Poses& ground_truth, Poses& estimated)
{
    if (ground_truth.empty() || estimated.empty()) {
        cerr << "Trajectory is empty!" << endl;
        return -1.0;
    }
    assert(ground_truth.size() == estimated.size());
    size_t length = ground_truth.size();

    double rmse = 0.0;
    for (size_t i = 0; i < length; i++) {
        auto e = (ground_truth[i].inverse() * estimated[i]).log().norm();
        rmse += e * e / length;
    }
    rmse = sqrt(rmse);
    cout<<"RMSE : "<<rmse<<endl;
    return rmse;
}
