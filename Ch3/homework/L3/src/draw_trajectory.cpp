#include <sophus/se3.h>
#include <string>
#include <iostream>
#include <fstream>

// need pangolin for plotting trajectory
#include <eigen3/Eigen/Core>
#include <pangolin/pangolin.h>

using namespace std;

// path to trajectory file
string trajectory_file = "./trajectory.txt";
string gt_file = "./groundtruth.txt";
string est_file = "./estimated.txt";

// function for plotting trajectory, don't edit this code
// start point is red and end point is blue
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>>);
int main(int argc, char **argv) {

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses;

    /// implement pose reading code
    // start your code here (5~10 lines)
    ifstream fin(trajectory_file);
    if(!fin)
    {
        cout<<"trajectory_file not found : "<<trajectory_file<<endl;
        return -1;
    }
    while (!fin.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3 pose_tmp(Eigen::Quaterniond(qx, qy, qz, qw), Eigen::Vector3d(tx, ty, tz));
        poses.push_back(pose_tmp);
    }
    cout<<"read "<<poses.size()<<" poses"<<endl;
    fin.close();
    // end your code here

    // // draw trajectory in pangolin
    // DrawTrajectory(poses);

    //reading ground truth and estmate
    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> gt_pose;
    ifstream gtin(gt_file);
    if(!gtin)
    {
        cout<<"trajectory_file not found : "<<gt_file<<endl;
        return -1;
    }
    while(!gtin.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        gtin >> time >> tx >> ty >> tz >> qx >> qy >>qz >>qw;
        Sophus::SE3 pose_tmp(Eigen::Quaterniond(qx,qy,qz,qw), Eigen::Vector3d(tx,ty,tz));
        gt_pose.push_back(pose_tmp);
    }
    cout<<"read "<<gt_pose.size()<<" gt poses"<<endl;
    gtin.close();

    vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> est_pose;
    ifstream estin(est_file);
    if(!estin)
    {
        cout<<"trajectory_file not found : "<<est_file<<endl;
        return -1;
    }
    while (!estin.eof())
    {
        double time, tx, ty, tz, qx, qy, qz, qw;
        estin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Sophus::SE3 pose_tmp(Eigen::Quaterniond(qx, qy, qz, qw), Eigen::Vector3d(tx, ty, tz));
        est_pose.push_back(pose_tmp);
    }
    cout << "read " << est_pose.size() << " est poses" << endl;
    estin.close();
    // calculated error
    double rmse = 0;
    for(int i = 0 ; i < est_pose.size();i++)
    {
        Sophus::SE3 gt = gt_pose[i];
        Sophus::SE3 est = est_pose[i];
        double errortmp = (gt.inverse()*est).log().norm();
        //cout<<"i "<<i<<" "<<errortmp<<endl;
        rmse = rmse + errortmp*errortmp;
    }
    cout <<"rmse "<<rmse <<endl;
    rmse = sqrt(rmse / est_pose.size());
    cout <<"rmse "<<rmse <<endl;
    DrawTrajectory(gt_pose,est_pose);
    return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses) {
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

/*******************************************************************************************/
void DrawTrajectory(vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses1, vector<Sophus::SE3, Eigen::aligned_allocator<Sophus::SE3>> poses2)
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