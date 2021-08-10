#include <iostream>
#include <vector>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <thread>
#include <ceres/ceres.h>
#include "ceres/rotation.h"

using namespace std;

typedef struct Observation_ {
    uint32_t cam_index;
    uint32_t point_index;
    double x;
    double y;
} Observation;

typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> VecVec3d;
typedef std::vector<Observation> VecObs;
typedef std::vector<std::vector<double>> VecCamParam;

void Draw(const VecVec3d &points, std::string window_name);

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

struct ReprojectionError {
    ReprojectionError(double obs_x, double obs_y) : x(obs_x), y(obs_y) {}

    template<typename T>
    bool operator()(const T* const camera, 
                    const T* const point,
                    T* residuals) const
    {
        // rotation
        T p[3];
        ceres::AngleAxisRotatePoint(camera, point, p);
        // translation
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];
        // perspective
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];
        // distortion
        const T& k1 = camera[7];
        const T& k2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = 1.0 + r2 * (k1 + k2 * r2);

        const T& focal = camera[6];
        T predict_x = focal * distortion * xp;
        T predict_y = focal * distortion * yp;

        residuals[0] = predict_x - x;
        residuals[1] = predict_y - y;
        return true;
    }

    static ceres::CostFunction* Create(const double obs_x, const double obs_y)
    {
        return (new ceres::AutoDiffCostFunction<ReprojectionError, 2, 9, 3>(
            new ReprojectionError(obs_x, obs_y)
        ));
    }

    double x;
    double y;
};


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

    // std::thread th1(&Draw, points, "trajectory viewer"); 
    for (size_t i = 0; i < cam_params.size(); i++)
    {
        printf("%d : [%f, %f, %f, %f, %f, %f, %f, %f, %f]\n",
            i, 
            cam_params[i][0], 
            cam_params[i][1], 
            cam_params[i][2], 
            cam_params[i][3], 
            cam_params[i][4], 
            cam_params[i][5], 
            cam_params[i][6], 
            cam_params[i][7], 
            cam_params[i][8]
        );
    }
    

    ceres::Problem problem;
    for (size_t i = 0; i < observations.size(); i++)
    {
        double x = observations[i].x;
        double y = observations[i].y;
        ceres::CostFunction* cost_function = ReprojectionError::Create(x, y);
        problem.AddResidualBlock(cost_function, NULL, cam_params[observations[i].cam_index].data(), points[observations[i].point_index].data());
    }
    
    ceres::Solver::Options options;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    options.minimizer_progress_to_stdout = true;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout<<summary.FullReport() <<"\n";

    for (size_t i = 0; i < cam_params.size(); i++)
    {
        printf("%d : [%f, %f, %f, %f, %f, %f, %f, %f, %f]\n",
            i, 
            cam_params[i][0], 
            cam_params[i][1], 
            cam_params[i][2], 
            cam_params[i][3], 
            cam_params[i][4], 
            cam_params[i][5], 
            cam_params[i][6], 
            cam_params[i][7], 
            cam_params[i][8]
        );
    }

    // th1.join();
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