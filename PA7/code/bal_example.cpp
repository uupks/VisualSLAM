#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <unistd.h>

#include "g2o/EXTERNAL/ceres/autodiff.h"
#include "g2o/core/auto_differentiation.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/solver.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <pangolin/pangolin.h>

using namespace std;

typedef struct Observation_ {
    uint32_t cam_index;
    uint32_t point_index;
    double x;
    double y;
} Observation;

using Vector9 = Eigen::Matrix<double, 9, 1, Eigen::ColMajor>;

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;
typedef std::vector<Observation> VecObs;
typedef std::vector<std::vector<double>> VecCamParam;
// typedef std::map<int, std::vector<double>> MapCamParam;

void Draw(const VecVec3d &points, std::string window_name);


// camera vertex
class VertexCamera : public g2o::BaseVertex<9, Vector9>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    VertexCamera() {}

    virtual bool read(std::istream& /*is*/) {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual void setToOriginImpl() { cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl; }

    virtual void oplusImpl(const double* update) {
        Vector9::ConstMapType v(update, VertexCamera::Dimension);
        _estimate += v;
    }
};

// point vertex
class VertexPoint : public g2o::BaseVertex<3, g2o::Vector3>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPoint() {}

    virtual bool read(std::istream& /*is*/) {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual void setToOriginImpl() { cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl; }

    virtual void oplusImpl(const double* update) {
        g2o::Vector3::ConstMapType v(update);
        _estimate += v;
    }
};

// edge
class EdgeObervation : public g2o::BaseBinaryEdge<2, g2o::Vector2, VertexCamera, VertexPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    EdgeObervation() {}

    virtual bool read(std::istream& /*is*/) {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }


    virtual void computeError() override
    {
        const VertexPoint* point = static_cast<const VertexPoint*>(vertex(1));
        const VertexCamera* camera = static_cast<const VertexCamera*>(vertex(0));

        Eigen::Vector3d rotation_vector = camera->estimate().head(3);
        double angle = rotation_vector.norm();
        
        Eigen::Vector3d axis = rotation_vector.normalized();
        Eigen::AngleAxisd rv(angle, axis);

        Eigen::Vector3d t = camera->estimate().segment(3, 3);
        Eigen::Vector3d p = rv * point->estimate() + t;

        Eigen::Vector2d projected_p = -p.head(2) / p(2);
        double r_sqr = projected_p.squaredNorm();

        double f = camera->estimate()(6);
        double k1 = camera->estimate()(7);
        double k2 = camera->estimate()(8);
        double rq = 1.0 + k1 * r_sqr + k2 * r_sqr * r_sqr;

        Eigen::Vector2d prediction = f * rq * projected_p;

        _error = _measurement - prediction;
    }

    // template <typename T>
    // bool operator()(const T* p_camera, const T* p_point, T* p_error) const 
    // {
    //     typename g2o::VectorN<9, T>::ConstMapType camera(p_camera);
    //     typename g2o::VectorN<3, T>::ConstMapType point(p_point);
    //     typename g2o::VectorN<3, T> p;

    //     // Rodrigues' formula for the rotation
    //     T theta = camera.template head<3>().norm();
    //     if (theta > T(0)) {
    //         g2o::VectorN<3, T> v = camera.template head<3>() / theta;
    //         T cth = cos(theta);
    //         T sth = sin(theta);

    //         g2o::VectorN<3, T> vXp = v.cross(point);
    //         T vDotp = v.dot(point);
    //         T oneMinusCth = T(1) - cth;

    //         p = point * cth + vXp * sth + v * vDotp * oneMinusCth;
    //     } else {
    //         // taylor expansion for theta close to zero
    //         p = point + camera.template head<3>().cross(point);
    //     }

    //         // translation of the camera
    //     p += camera.template segment<3>(3);

    //     // perspective division
    //     g2o::VectorN<2, T> projectedPoint = -p.template head<2>() / p(2);

    //     // conversion to pixel coordinates
    //     T radiusSqr = projectedPoint.squaredNorm();
    //     const T& f = camera(6);
    //     const T& k1 = camera(7);
    //     const T& k2 = camera(8);
    //     T r_p = T(1) + k1 * radiusSqr + k2 * radiusSqr * radiusSqr;
    //     g2o::VectorN<2, T> prediction = f * r_p * projectedPoint;

    //     // compute the error
    //     typename g2o::VectorN<2, T>::MapType error(p_error);
    //     error = prediction - measurement().cast<T>();
    //     (void)error;
    //     return true;
    // }
    // G2O_MAKE_AUTO_AD_FUNCTIONS
};

void load_ba_data(std::string in_file, VecObs &obs, VecCamParam &params, VecVec3d &points)
{
    ifstream fin(in_file);
    uint32_t num_cams = 0;
    uint32_t num_points = 0;
    uint32_t num_obs = 0;

    fin >> num_cams >> num_points >> num_obs;
    std::cout<<"num cams : "<<num_cams<<std::endl;
    std::cout<<"num points : "<<num_points<<std::endl;
    std::cout<<"num obs : "<<num_obs<<std::endl;

    // while (!fin.eof()) {
    for (size_t i = 0; i < num_obs; i++)
    {
        uint32_t cam_index, point_index;
        double x, y;
        fin>>cam_index>>point_index>>x>>y;
        Observation ob = {cam_index, point_index, x, y};
        obs.push_back(ob);
    }

    for (int i = 0; i < num_cams; i++)
    {
        std::vector<double> param;
        for (size_t i = 0; i < 9; i++)
        {
            double d;
            fin >> d;
            param.push_back(d);
        }
        params.push_back(param);
    }
    for (size_t i = 0; i < num_points; i++)
    {
        double x, y, z;
        fin >> x >> y >> z;
        points.push_back(Eigen::Vector3d(x, y, z));
    }
    // }
    fin.close();
}


int main(int argc, char **argv)
{
    if (argc != 2) 
    {
        std::cout<<argv[0]<<" [data_file]"<<std::endl;
        return -1;
    }
    std::string data_file = argv[1];
    VecCamParam cam_params;
    VecObs observations;
    VecVec3d points;
    load_ba_data(data_file, observations, cam_params, points);

    std::cout<<"cam_params : "<<cam_params.size()<<std::endl;
    std::cout<<"points : "<<points.size()<<std::endl;
    std::cout<<"observations : "<<observations.size()<<std::endl;


    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 3>> BalBlockSolver;
    // typedef g2o::LinearSolverEigen<BalBlockSolver::PoseMatrixType> BalLinearSolver;
    typedef g2o::LinearSolverCholmod<BalBlockSolver::PoseMatrixType> BalLinearSolver;

    g2o::SparseOptimizer optimizer;

    std::unique_ptr<g2o::LinearSolver<BalBlockSolver::PoseMatrixType>> linearSolver;
    auto cholesky = g2o::make_unique<BalLinearSolver>();
    cholesky->setBlockOrdering(true);
    linearSolver = std::move(cholesky);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
    g2o::make_unique<BalBlockSolver>(std::move(linearSolver)));

    optimizer.setAlgorithm(solver);

    // add camera vertex
    for (size_t i = 0; i < cam_params.size(); i++)
    {
        Vector9 param(cam_params.at(i).data());
        VertexCamera* v = new VertexCamera();
        v->setEstimate(param);
        v->setId(i);
        optimizer.addVertex(v);
    }
    // add points vertex
    for (size_t i = 0; i < points.size(); i++)
    {
        VertexPoint *vertex_p = new VertexPoint();
        vertex_p->setId(i + cam_params.size());
        g2o::Vector3 p;
        p(0) = points[i].x();
        p(1) = points[i].y();
        p(2) = points[i].z();
        vertex_p->setEstimate(p);
        vertex_p->setMarginalized(true);
        optimizer.addVertex(vertex_p);
    }
    // add observation edge
    for (size_t i = 0; i < observations.size(); i++)
    {
        uint32_t cam_index = observations[i].cam_index;
        uint32_t point_index = observations[i].point_index;
        double x = observations[i].x;
        double y = observations[i].y;

        auto p = dynamic_cast<VertexPoint*>(optimizer.vertex(point_index + cam_params.size()));
        EdgeObervation *edge = new EdgeObervation;
        edge->setVertex(1, dynamic_cast<VertexPoint*>(optimizer.vertex(point_index + cam_params.size())));
        edge->setVertex(0, dynamic_cast<VertexCamera*>(optimizer.vertex(cam_index)));
        edge->setInformation(Eigen::Matrix2d::Identity());
        
        edge->setMeasurement(g2o::Vector2(x, y));

        // g2o::RobustKernelHuber* huber_kernel = new g2o::RobustKernelHuber();
        // huber_kernel->setDelta(1.0);
        // edge->setRobustKernel(huber_kernel);
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization(0);
    optimizer.setVerbose(true);
    optimizer.optimize(30);

    VecVec3d optimzed_points;
    optimzed_points.resize(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        Eigen::Vector3d p_w = dynamic_cast<VertexPoint *>(optimizer.vertex(i + cam_params.size()))->estimate();
        optimzed_points[i] = p_w;
    }

    Draw(optimzed_points, "Optimized");
    return 0;
}


void Draw(const VecVec3d &points, std::string window_name) {

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
