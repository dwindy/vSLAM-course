//
// Created by xin on 12/07/2020.
//

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/core/robust_kernel_impl.h>
#include <iostream>

#include "common.h"
#include "sophus/se3.h"

using namespace Sophus;
using namespace Eigen;
using namespace std;

//参考示例代码定义相机模型和投影关系
struct PoseAndIntrinsics {
    PoseAndIntrinsics(){}
    SO3 rotation;
    Vector3d translation = Vector3d::Zero();
    double focal = 0;
    double k1=0, k2 = 0;

    explicit PoseAndIntrinsics(double *data_input) {
        rotation = SO3::exp(Vector3d(data_input[0], data_input[1], data_input[2]));
        translation = Vector3d(data_input[3], data_input[4], data_input[5]);
        focal = data_input[6];
        k1 = data_input[7];
        k2 = data_input[8];
    }

    void set_data(double *data_output) {
        auto r = rotation.log();
        for (int i = 0; i < 3; i++)
            data_output[i] = r[i];
        for (int i = 0; i < 3; i++)
            data_output[i + 3] = translation[i];
        data_output[6] = focal;
        data_output[7] = k1;
        data_output[8] = k2;
    }
};

//继承顶点类  <9维(相机位姿和内参)，自定义数据类型>
class VertexCamera : public g2o ::BaseVertex<9, PoseAndIntrinsics>
{
public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexCamera(){}
    //实现重置函数
    virtual void setToOriginImpl() override {
        _estimate = PoseAndIntrinsics();
    }
    //实现更新函数
    virtual void oplusImpl(const double *update) override  {
        //左乘更新
        _estimate.rotation = SO3::exp(Vector3d(update[0],update[1],update[2])) * _estimate.rotation;
        _estimate.translation += Vector3d(update[3],update[4],update[5]);
        _estimate.focal +=update[6];
        _estimate.k1 +=update[7];
        _estimate.k2 +=update[8];
    }
    //投影
    Vector2d project(const Vector3d &point)
    {
        Vector3d p_undis = _estimate.rotation * point + _estimate.translation;
        p_undis = - p_undis/p_undis[2];
        double r2 = p_undis.squaredNorm();
        double distortion = 1.0 + r2*(_estimate.k1+_estimate.k2*r2);
        Vector2d p_distor = Vector2d(_estimate.focal * distortion * p_undis[0],
                                     _estimate.focal * distortion * p_undis[1]);
        return p_distor;
    }

    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

//继承顶点类 (点云) <3维，vector3d数据类型>
class VertexPoint : public g2o::BaseVertex<3, Vector3d>
{
public :
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPoint(){}

    //实现重置
    virtual void setToOriginImpl() override
    {
        _estimate = Vector3d(0,0,0);
    }
    //实现更新
    virtual void oplusImpl(const double *update) override
    {
        _estimate +=Vector3d(update[0], update[1], update[2]);
    }
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

//继承边 <2维（观测为像素坐标），数据类型，相机顶点，特征点顶点>
class Edge: public g2o::BaseBinaryEdge<2, Vector2d, VertexCamera, VertexPoint>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    //实现误差计算函数
    virtual void computeError() override
    {
        //两个顶点的指针
        VertexCamera* vertex0 = (VertexCamera *) _vertices[0];
        VertexPoint* vertex1 = (VertexPoint *) _vertices[1];
        Vector2d projpoint = vertex0->project(vertex1->estimate());
        _error = projpoint - _measurement;
    }
    // use numeric derivatives
    virtual bool read(istream &in) {}

    virtual bool write(ostream &out) const {}
};

void SolveBA(BALProblem &bal_problem)
{
    const int point_block_size = bal_problem.point_block_size();
    const int camera_block_size = bal_problem.camera_block_size();
    //指针 拿数据用
    double *points = bal_problem.mutable_points();
    double *cameras = bal_problem.mutable_cameras();

    //边链接的两个顶点的维度分别是9和3
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<9,3>> BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    const double *observations = bal_problem.observations();
    //定义两个结构体 往里面塞数据
    vector<VertexCamera *> vertex_cameras;
    vector<VertexPoint *> vertex_points;
    //往optimizer里面增加顶点
    for(int i=0;i<bal_problem.num_cameras();i++)
    {
        VertexCamera *vc = new VertexCamera();
        double *camera = cameras + camera_block_size * i;
        vc->setId(i);
        //设置初始值
        vc->setEstimate(PoseAndIntrinsics(camera));
        optimizer.addVertex(vc);
        vertex_cameras.push_back(vc);
    }
    for(int i=0;i<bal_problem.num_points();i++)
    {
        VertexPoint *vp = new VertexPoint();
        double *point = points + point_block_size*i;
        vp->setId(i+bal_problem.num_cameras());
        vp->setEstimate(Vector3d(point[0],point[1],point[2]));
        vp->setMarginalized(true);
        optimizer.addVertex(vp);
        vertex_points.push_back(vp);
    }
    //往opitimizer里面增加边
    for(int i=0;i<bal_problem.num_observations(); i++)
    {
        Edge *edge = new Edge;
        //第i个observation的camera id
        edge->setVertex(0,vertex_cameras[bal_problem.camera_index()[i]]);
        edge->setVertex(1,vertex_points[bal_problem.point_index()[i]]);
        edge->setMeasurement(Vector2d(observations[2*i+0],observations[2*i+1]));
        edge->setInformation(Matrix2d::Identity());
        //防outlier
        edge->setRobustKernel(new g2o::RobustKernelHuber());
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    optimizer.optimize(40);
    //结果输出到两个vector里面
    for (int i = 0; i < bal_problem.num_cameras(); i++) {
        double *camera = cameras + camera_block_size * i;
        VertexCamera *vertexC = vertex_cameras[i];
        auto estimate = vertexC->estimate();
        estimate.set_data(camera);
    }
    for (int i = 0; i < bal_problem.num_points(); ++i) {
        double *point = points + point_block_size * i;
        auto vertex = vertex_points[i];
        for (int k = 0; k < 3; ++k) point[k] = vertex->estimate()[k];
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        cout << "usage: bundle_adjustment_g2o bal_data.txt" <<
             endl;
        return 1;
    }
    //借用示例代码提供的BAL数据集的BAL类
    BALProblem bal_problem(argv[1]);
    bal_problem.Normalize();
    bal_problem.Perturb(0.1, 0.5, 0.5);
    bal_problem.WriteToPLYFile("initial.ply");
    SolveBA(bal_problem);
    bal_problem.WriteToPLYFile("final.ply");
    cout<<" function end"<<endl;
    return 0;
}



