#include <jni.h>
#include <string>

#include <iostream>
#include <cstdint>

#include <unordered_set>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
//#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/types/sba/types_six_dof_expmap.h"
//#include "g2o/math_groups/se3quat.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/stuff/sampler.h"

using namespace Eigen;
using namespace std;

class Sample {
public:
    static int uniform(int from, int to) { return static_cast<int>(g2o::Sampler::uniformRand(from, to)); }
};

/*http://cncc.bingj.com/cache.aspx?q=use+std+cout+in+android+ndk&d=4687377020238001&mkt=en-US&setlang=en-US&w=ngI5Ze_Z6VlTktzu3WAM95_46h9ZgL4T
 * Redirect std::cout to android log output*/
#include <iostream>
#include <unistd.h>
#include <pthread.h>
#include <android/log.h>

static int pfd[2];
static pthread_t thr;
static const char *tag = "from-std-cout";

static void *thread_func(void*)
{
    ssize_t rdsz;
    char buf[128];
    while((rdsz = read(pfd[0], buf, sizeof buf - 1)) > 0) {
        if(buf[rdsz - 1] == '\n') --rdsz;
        buf[rdsz] = 0;  /* add null-terminator */
        __android_log_write(ANDROID_LOG_DEBUG, tag, buf);
    }
    return 0;
}

int start_logger(const char *app_name)
{
    tag = app_name;

    /* make stdout line-buffered and stderr unbuffered */
    setvbuf(stdout, 0, _IOLBF, 0);
    setvbuf(stderr, 0, _IONBF, 0);

    /* create the pipe and redirect stdout and stderr */
    pipe(pfd);
    dup2(pfd[1], 1);
    dup2(pfd[1], 2);

    /* spawn the logging thread */
    if(pthread_create(&thr, 0, thread_func, 0) == -1)
        return -1;
    pthread_detach(thr);
    return 0;
}



extern "C" JNIEXPORT jstring JNICALL
Java_com_example_myapplication_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {

    start_logger("g2otest");

    double PIXEL_NOISE = 1;
    double OUTLIER_RATIO = 0.0;
    bool ROBUST_KERNEL = true;
    bool STRUCTURE_ONLY = false;
    bool DENSE = true;

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver;
    if (DENSE) {
        linearSolver = g2o::make_unique<g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>>();
    } /*else {
        linearSolver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>>();
    }*/

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
            g2o::make_unique<g2o::BlockSolver_6_3>(std::move(linearSolver))
    );
    optimizer.setAlgorithm(solver);


    vector<Vector3d> true_points;
    for (size_t i=0;i<500; ++i)
    {
        true_points.push_back(Vector3d((g2o::Sampler::uniformRand(0., 1.)-0.5)*3,
                                       g2o::Sampler::uniformRand(0., 1.)-0.5,
                                       g2o::Sampler::uniformRand(0., 1.)+3));
    }

    double focal_length= 1000.;
    Vector2d principal_point(320., 240.);

    vector<g2o::SE3Quat,
            aligned_allocator<g2o::SE3Quat> > true_poses;
    g2o::CameraParameters * cam_params
            = new g2o::CameraParameters (focal_length, principal_point, 0.);
    cam_params->setId(0);

    if (!optimizer.addParameter(cam_params)) {
        assert(false);
    }

    int vertex_id = 0;
    for (size_t i=0; i<15; ++i) {
        Vector3d trans(i*0.04-1.,0,0);

        Eigen:: Quaterniond q;
        q.setIdentity();
        g2o::SE3Quat pose(q,trans);
        g2o::VertexSE3Expmap * v_se3
                = new g2o::VertexSE3Expmap();
        v_se3->setId(vertex_id);
        if (i<2){
            v_se3->setFixed(true);
        }
        v_se3->setEstimate(pose);
        optimizer.addVertex(v_se3);
        true_poses.push_back(pose);
        vertex_id++;
    }
    int point_id=vertex_id;
    int point_num = 0;
    double sum_diff2 = 0;

    cout << endl;
    unordered_map<int,int> pointid_2_trueid;
    unordered_set<int> inliers;

    for (size_t i=0; i<true_points.size(); ++i){
        g2o::VertexSBAPointXYZ * v_p
                = new g2o::VertexSBAPointXYZ();
        v_p->setId(point_id);
        v_p->setMarginalized(true);
        v_p->setEstimate(true_points.at(i)
                         + Vector3d(g2o::Sampler::gaussRand(0., 1),
                                    g2o::Sampler::gaussRand(0., 1),
                                    g2o::Sampler::gaussRand(0., 1)));
        int num_obs = 0;
        for (size_t j=0; j<true_poses.size(); ++j){
            Vector2d z = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));
            if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
                ++num_obs;
            }
        }
        if (num_obs>=2){
            optimizer.addVertex(v_p);
            bool inlier = true;
            for (size_t j=0; j<true_poses.size(); ++j){
                Vector2d z
                        = cam_params->cam_map(true_poses.at(j).map(true_points.at(i)));

                if (z[0]>=0 && z[1]>=0 && z[0]<640 && z[1]<480){
                    double sam = g2o::Sampler::uniformRand(0., 1.);
                    if (sam<OUTLIER_RATIO){
                        z = Vector2d(Sample::uniform(0,640),
                                     Sample::uniform(0,480));
                        inlier= false;
                    }
                    z += Vector2d(g2o::Sampler::gaussRand(0., PIXEL_NOISE),
                                  g2o::Sampler::gaussRand(0., PIXEL_NOISE));
                    g2o::EdgeProjectXYZ2UV * e
                            = new g2o::EdgeProjectXYZ2UV();
                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(v_p));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>
                    (optimizer.vertices().find(j)->second));
                    e->setMeasurement(z);
                    e->information() = Matrix2d::Identity();
                    if (ROBUST_KERNEL) {
                        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                        e->setRobustKernel(rk);
                    }
                    e->setParameterId(0, 0);
                    optimizer.addEdge(e);
                }
            }

            if (inlier){
                inliers.insert(point_id);
                Vector3d diff = v_p->estimate() - true_points[i];

                sum_diff2 += diff.dot(diff);
            }
            pointid_2_trueid.insert(make_pair(point_id,i));
            ++point_id;
            ++point_num;
        }
    }
    cout << endl;
    optimizer.initializeOptimization();
    optimizer.setVerbose(true);
    if (STRUCTURE_ONLY){
        g2o::StructureOnlySolver<3> structure_only_ba;
        cout << "Performing structure-only BA:"   << endl;
        g2o::OptimizableGraph::VertexContainer points;
        for (g2o::OptimizableGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it) {
            g2o::OptimizableGraph::Vertex* v = static_cast<g2o::OptimizableGraph::Vertex*>(it->second);
            if (v->dimension() == 3)
                points.push_back(v);
        }
        structure_only_ba.calc(points, 10);
    }
    //optimizer.save("test.g2o");
    cout << endl;
    cout << "Performing full BA:" << endl;
    optimizer.optimize(10);
    cout << endl;
    cout << "Point error before optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;
    point_num = 0;
    sum_diff2 = 0;
    for (unordered_map<int,int>::iterator it=pointid_2_trueid.begin();
         it!=pointid_2_trueid.end(); ++it){
        g2o::HyperGraph::VertexIDMap::iterator v_it
                = optimizer.vertices().find(it->first);
        if (v_it==optimizer.vertices().end()){
            cerr << "Vertex " << it->first << " not in graph!" << endl;
            exit(-1);
        }
        g2o::VertexSBAPointXYZ * v_p
                = dynamic_cast< g2o::VertexSBAPointXYZ * > (v_it->second);
        if (v_p==0){
            cerr << "Vertex " << it->first << "is not a PointXYZ!" << endl;
            exit(-1);
        }
        Vector3d diff = v_p->estimate()-true_points[it->second];
        if (inliers.find(it->first)==inliers.end())
            continue;
        sum_diff2 += diff.dot(diff);
        ++point_num;
    }
    cout << "Point error after optimisation (inliers only): " << sqrt(sum_diff2/inliers.size()) << endl;

    cout << endl;

    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
