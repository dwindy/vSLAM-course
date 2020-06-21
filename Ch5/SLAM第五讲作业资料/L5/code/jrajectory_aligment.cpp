#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/SVD>
#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.h>

using namespace Eigen;
using namespace std;
using namespace cv;

string trajectory_file = "./compare.txt";

//Draw function
typedef vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> TrajectoryType;
void DrawTrajectory(TrajectoryType poses1, TrajectoryType poses2);

int main()
{
    //load trajectory
    vector<Point3f>points1;
    vector<Point3f>points2;
    ifstream fin(trajectory_file);
    vector<Quaterniond, Eigen::aligned_allocator<Quaterniond>> oritations1;
    vector<Quaterniond, Eigen::aligned_allocator<Quaterniond>> oritations2;
    while (!fin.eof()) {
        double timestamp1, tx1,ty1,tz1,qx1,qy1,qz1,qw1,timestamp2,tx2,ty2,tz2,qx2,qy2,qz2,qw2;
        fin >> timestamp1>>tx1>>ty1>>tz1>>qx1>>qy1>>qz1>>qw1>>timestamp2>>tx2>>ty2>>tz2>>qx2>>qy2>>qz2>>qw2;
        Point3f thisPoint1(tx1,ty1,tz1);
        Point3f thisPoint2(tx2,ty2,tz2);
        points1.push_back(thisPoint1);
        points2.push_back(thisPoint2);
        Quaterniond thisOritation1(qx1,qy1,qz1,qw1);
        Quaterniond thisOritation2(qx2,qy2,qz2,qw2);
        oritations1.push_back(thisOritation1);
        oritations2.push_back(thisOritation2);
    }
    assert(points1.size()==points2.size());
    cout<<"loaded "<<points1.size()<<" point pairs"<<endl;

    //-----ICP SVD Approach
    //remove centre
    int pairNum = points1.size();
    Point3f point_sum1;
    Point3f point_sum2;
    for(int i=0; i< pairNum;i++)
    {
        point_sum1 +=points1[i];
        point_sum2 +=points2[i];
    }
    Point3f point_mean1 = point_sum1 /pairNum;
    Point3f point_mean2 = point_sum2 /pairNum;
    vector<Point3f>points_uniform1;
    vector<Point3f>points_uniform2;
    for(int i=0; i< pairNum;i++)
    {
        points_uniform1.push_back(points1[i]-point_mean1);
        points_uniform2.push_back(points2[i]-point_mean2);
    }
    //define W
    Matrix3d W = Matrix3d::Zero();
    for(int i=0;i<pairNum;i++)
    {
        Vector3d V1(points_uniform1[i].x, points_uniform1[i].y, points_uniform1[i].z);
        Vector3d V2(points_uniform2[i].x, points_uniform2[i].y, points_uniform2[i].z);
        W +=V1 * V2.transpose();
    }
    cout<<"W="<<endl<<W<<endl;
    //SVD
    JacobiSVD<Matrix3d> svd_w(W, ComputeFullU | ComputeFullV);
    cout<<"U="<<endl<<svd_w.matrixU()<<endl;
    cout<<"V="<<endl<<svd_w.matrixV()<<endl;
    Matrix3d R = Matrix3d::Zero();
    R = svd_w.matrixU() * svd_w.matrixV().transpose();
    if(R.determinant()<0)
        R = -R;
    Vector3d t = Vector3d(point_mean1.x, point_mean1.y, point_mean1.z) - R * Vector3d(point_mean2.x, point_mean2.y, point_mean2.z);
    cout<<"R "<<endl<<R<<endl;
    cout<<"t "<<endl<<t<<endl;
    //project all points
    //and construct SE3
    vector<Vector3d> projectedPoint2;
    TrajectoryType trajectory1;
    TrajectoryType trajectory2;
    for(int i =0;i<pairNum;i++)
    {
        Vector3d thisPoint2(points2[i].x, points2[i].y, points2[i].z);
        Vector3d thisProj =R*thisPoint2 + t;
        projectedPoint2.push_back(thisProj);
        Sophus::SE3 p1(oritations1[i], Vector3d(points1[i].x,points1[i].y,points1[i].z));
        Sophus::SE3 p2(oritations2[i], thisProj);
        trajectory1.push_back(p1);
        trajectory2.push_back(p2);
    }
    DrawTrajectory(trajectory1,trajectory2);
    return 0;
}

void DrawTrajectory(TrajectoryType poses1, TrajectoryType poses2)
{
    if (poses1.empty())
    {
        cerr << "Trajectory 1 is empty!" << endl;
        return;
    }
    if (poses2.empty())
    {
        cerr << "Trajectory 2 is empty!" << endl;
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
        for (size_t i = 0; i < poses1.size() - 1; i++)
        {
            glColor3f(0.0f, 0.0f, 1.0f);
            glBegin(GL_LINES);
            auto p1 = poses1[i], p2 = poses1[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        for (size_t i = 0; i < poses2.size() - 1; i++)
        {
            glColor3f(1.0f, 0.0f, 0.0f);
            glBegin(GL_LINES);
            auto p1 = poses2[i], p2 = poses2[i + 1];
            glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
            glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
            glEnd();
        }
        pangolin::FinishFrame();
        usleep(3000);   // sleep 5 ms
    }
}