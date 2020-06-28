//
// Created by 高翔 on 2017/12/19.
// 本程序演示如何从Essential矩阵计算R,t
//

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

using namespace Eigen;

//#include <sophus/so3.hpp>
#include <sophus/so3.h>

#include <iostream>

using namespace std;

int main(int argc, char **argv) {

    // 给定Essential矩阵
    Matrix3d E;
    E << -0.0203618550523477, -0.4007110038118445, -0.03324074249824097,
            0.3939270778216369, -0.03506401846698079, 0.5857110303721015,
            -0.006788487241438284, -0.5815434272915686, -0.01438258684486258;

    // 待计算的R,t
    Matrix3d R;
    Vector3d t;

    // SVD and fix sigular values
    // START YOUR CODE HERE
    JacobiSVD<Eigen::MatrixXd> svd(E, ComputeFullU | ComputeFullV);
    Vector3d oldsingular = svd.singularValues();
    Matrix3d singular_matrix = Matrix3d::Identity();
    singular_matrix(0, 0) = (oldsingular(0) + oldsingular(1)) / 2;
    singular_matrix(1, 1) = singular_matrix(0, 0);
    singular_matrix(2, 2) = 0;
    cout << "new singular " << endl << singular_matrix << endl;
    // END YOUR CODE HERE

    // set t1, t2, R1, R2 
    // START YOUR CODE HERE
    Matrix3d t_wedge1;
    Matrix3d t_wedge2;
    Matrix3d R_z90 = AngleAxisd(3.141592653 / 2, Vector3d(0, 0, 1)).toRotationMatrix();
    Matrix3d R_z_90 = AngleAxisd(3.141592653 / -2, Vector3d(0, 0, 1)).toRotationMatrix();
    t_wedge1 = svd.matrixU() * R_z90 * singular_matrix * svd.matrixU().transpose();
    t_wedge2 = svd.matrixU() * R_z_90 * singular_matrix * svd.matrixU().transpose();
    Matrix3d R1;
    Matrix3d R2;
    R1 = svd.matrixU() * R_z90.transpose() * svd.matrixV().transpose();
    R2 = svd.matrixU() * R_z_90.transpose() * svd.matrixV().transpose();
    // END YOUR CODE HERE

    cout << "R1 = " << endl << R1 << endl;
    cout << "R2 = " << endl << R2 << endl;
    cout << "t1 = " << Sophus::SO3::vee(t_wedge1) << endl;
    cout << "t2 = " << Sophus::SO3::vee(t_wedge2) << endl;

    // check t^R=E up to scale
    Matrix3d tR = t_wedge1 * R1;
    cout << "t^R = " << endl << tR << endl;

    return 0;
}