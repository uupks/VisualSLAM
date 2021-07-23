//
// Created by xiang on 1/4/18.
// this program shows how to perform direct bundle adjustment
//
#include <iostream>
#include <unistd.h>
#include <thread>
using namespace std;

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include <pangolin/pangolin.h>

typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> VecSE3;
typedef vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;

// global variables
string pose_file = "../poses.txt";
string points_file = "../points.txt";

// intrinsics
float fx = 277.34;
float fy = 291.402;
float cx = 312.234;
float cy = 239.777;

// bilinear interpolation
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
    );
}

// g2o vertex that use sophus::SE3 as pose
class VertexSophus : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexSophus() {}

    ~VertexSophus() {}

    bool read(std::istream &is) {}

    bool write(std::ostream &os) const {}

    virtual void setToOriginImpl() {
        _estimate = Sophus::SE3d();
    }

    virtual void oplusImpl(const double *update_) {
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> update(update_);
        setEstimate(Sophus::SE3d::exp(update) * estimate());
    }
};

// TODO edge of projection error, implement it
// 16x1 error, which is the errors in patch
typedef Eigen::Matrix<double,16,1> Vector16d;
class EdgeDirectProjection : public g2o::BaseBinaryEdge<16, Vector16d, g2o::VertexPointXYZ, VertexSophus> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeDirectProjection(float *color, cv::Mat &target) {
        this->origColor = color;
        this->targetImg = target;
    }

    ~EdgeDirectProjection() {}

    virtual void computeError() override {
        // TODO START YOUR CODE HERE
        const g2o::VertexPointXYZ* v_pw = static_cast<const g2o::VertexPointXYZ*>(vertex(0));
        const VertexSophus* T_cw = static_cast<const VertexSophus*>(vertex(1));
        Eigen::Vector3d p_c = T_cw->estimate() * v_pw->estimate();
        float x = p_c.x() / p_c.z() * fx + cx;
        float y = p_c.y() / p_c.z() * fy + cy;
        if ((x - 3) < 0 || (x + 2) > targetImg.cols || 
                (y - 3) < 0 || (y + 2) > targetImg.rows) {
            _error(0, 0) = 0.0;
            this->setLevel(1);
        } else {            
            for (int u = -2; u < 2; u++) {
                for (int v = -2; v < 2; v++) {
                    int num = 4 * u + v + 10;
                    _error[num] = origColor[num] - GetPixelValue(targetImg, x + u, y + v);
                }
            }
        }
        // END YOUR CODE HERE
    }

    // Let g2o compute jacobian for you

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}

private:
    cv::Mat targetImg;  // the target image
    float *origColor = nullptr;   // 16 floats, the color of this point
};

// plot the poses and points for you, need pangolin
void Draw(const VecSE3 &poses, const VecVec3d &points, std::string window_name);

int main(int argc, char **argv) {

    // read poses and points
    VecSE3 poses;
    VecVec3d points;
    ifstream fin(pose_file);

    while (!fin.eof()) {
        double timestamp = 0;
        fin >> timestamp;
        if (timestamp == 0) break;
        double data[7];
        for (auto &d: data) fin >> d;
        poses.push_back(Sophus::SE3d(
                Eigen::Quaterniond(data[6], data[3], data[4], data[5]),
                Eigen::Vector3d(data[0], data[1], data[2])
        ));
        if (!fin.good()) break;
    }
    fin.close();


    vector<float *> color;
    fin.open(points_file);
    while (!fin.eof()) {
        double xyz[3] = {0};
        for (int i = 0; i < 3; i++) fin >> xyz[i];
        if (xyz[0] == 0) break;
        points.push_back(Eigen::Vector3d(xyz[0], xyz[1], xyz[2]));
        float *c = new float[16];
        for (int i = 0; i < 16; i++) fin >> c[i];
        color.push_back(c);

        if (fin.good() == false) break;
    }
    fin.close();

    cout << "poses: " << poses.size() << ", points: " << points.size() << endl;

    // read images
    vector<cv::Mat> images;
    for (int i = 0; i < 7; i++) {
        std::string img_name = "../" + std::to_string(i) + ".png";
        std::cout<<"Image Name : "<<img_name<<std::endl;
        images.push_back(cv::imread(img_name, 0));
    }
    // build optimization problem
    using DirectBlock = g2o::BlockSolver<g2o::BlockSolverTraits<6, 3>>;                 // 优化变量维度为6，误差维度为3
    using LinearSolverType = g2o::LinearSolverDense<DirectBlock::PoseMatrixType>;       // 线性求解器类型 

    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<DirectBlock>(g2o::make_unique<LinearSolverType>())); // L-M
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // TODO add vertices, edges into the graph optimizer
    // START YOUR CODE HERE

    for (size_t i = 0; i < poses.size(); i++) {
        VertexSophus *v = new VertexSophus();
        v->setEstimate(poses[i]);
        v->setId(i);
        optimizer.addVertex(v);
    }

    for (size_t i = 0; i < points.size(); i++) {
        g2o::VertexPointXYZ *vertex_p = new g2o::VertexPointXYZ();
        vertex_p->setId(i + poses.size());
        vertex_p->setEstimate(points[i]);
        vertex_p->setMarginalized(true);
        optimizer.addVertex(vertex_p);
    }

    for (size_t i = 0; i < poses.size(); i++) {
        for (size_t j = 0; j < points.size(); j++) {
            EdgeDirectProjection *edge = new EdgeDirectProjection(color[j], images[i]);
            // edge->setId(j);
            edge->setVertex(0, dynamic_cast<g2o::VertexPointXYZ*>(optimizer.vertex(j + poses.size())));
            edge->setVertex(1, dynamic_cast<VertexSophus*>(optimizer.vertex(i)));
            edge->setInformation(Eigen::Matrix<double, 16, 16>::Identity());

            g2o::RobustKernelHuber* huber_kernel = new g2o::RobustKernelHuber();
            huber_kernel->setDelta(1.0);
            edge->setRobustKernel(huber_kernel);
            optimizer.addEdge(edge);
        }
    }
    // END YOUR CODE HERE
    // perform optimization
    optimizer.initializeOptimization(0);
    optimizer.optimize(200);

    // TODO fetch data from the optimizer
    // START YOUR CODE HERE
    VecSE3 optimized_poses;
    VecVec3d optimzed_points;
    optimized_poses.resize(poses.size());
    optimzed_points.resize(points.size());
    for (size_t i = 0; i < poses.size(); i++) {
        Sophus::SE3d T_cw = dynamic_cast<VertexSophus*>(optimizer.vertex(i))->estimate();
        optimized_poses[i] = T_cw;
    }
    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3d p_w = dynamic_cast<g2o::VertexPointXYZ *>(optimizer.vertex(i + poses.size()))->estimate();
        optimzed_points[i] = p_w;
    }
    // END YOUR CODE HERE
    // auto points_error = 
    Eigen::Vector3d points_error;
    for (size_t i = 0; i < points.size(); i++) {
        points_error += optimzed_points[i] - points[i];
    }
    std::cout<<"point error : "<<points_error<<std::endl;
    
    // plot the optimized points and poses
    std::thread th1(&Draw, poses, points, "trajectory viewer");
    std::thread th2(&Draw, optimized_poses, optimzed_points, "optimized trajectory viewer");
    // Draw(poses, points, "trajectory viewer");
    // Draw(poses, points, "optimized trajectory viewer");
    th1.join();
    th2.join();

    // delete color data
    for (auto &c: color) delete[] c;
    return 0;
}

void Draw(const VecSE3 &poses, const VecVec3d &points, std::string window_name) {
    if (poses.empty() || points.empty()) {
        cerr << "parameter is empty!" << endl;
        return;
    }

    // create pangolin window and plot the trajectory
    pangolin::CreateWindowAndBind(window_name, 1024, 768);
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
        glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

        // draw poses
        float sz = 0.1;
        int width = 640, height = 480;
        for (auto &Tcw: poses) {
            glPushMatrix();
            Sophus::Matrix4f m = Tcw.inverse().matrix().cast<float>();
            glMultMatrixf((GLfloat *) m.data());
            glColor3f(1, 0, 0);
            glLineWidth(2);
            glBegin(GL_LINES);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(0, 0, 0);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
            glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
            glEnd();
            glPopMatrix();
        }

        // points
        glPointSize(2);
        glBegin(GL_POINTS);
        for (size_t i = 0; i < points.size(); i++) {
            glColor3f(0.0, points[i][2]/4, 1.0-points[i][2]/4);
            glVertex3d(points[i][0], points[i][1], points[i][2]);
        }
        glEnd();

        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
}

