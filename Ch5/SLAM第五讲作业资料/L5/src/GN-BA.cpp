//
// Created by xiang on 12/21/17.
//

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

#include <vector>
#include <fstream>
#include <iostream>
#include <iomanip>

#include "sophus/se3.h"

using namespace std;

typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

string p3d_file = "./p3d.txt";
string p2d_file = "./p2d.txt";

int main(int argc, char **argv) {

    VecVector2d p2d;
    VecVector3d p3d;
    Matrix3d K;
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;

    // load points in to p3d and p2d 
    // START YOUR CODE HERE
    ifstream f2din(p2d_file);
    while (!f2din.eof()) {
        double x, y;
        f2din >> x >> y;
        Vector2d newP2d(x, y);
        p2d.push_back(newP2d);
    }
    f2din.close();
    ifstream f3din(p3d_file);
    while (!f3din.eof()) {
        double x, y, z;
        f3din >> x >> y >> z;
        Vector3d newP3d(x, y, z);
        p3d.push_back(newP3d);
    }
    f3din.close();
    // END YOUR CODE HERE
    assert(p3d.size() == p2d.size());

    int iterations = 100;
    double cost = 0, lastCost = 0;
    int nPoints = p3d.size();
    cout << "points: " << nPoints << endl;

    Sophus::SE3 T_esti; // estimated pose

    for (int iter = 0; iter < iterations; iter++) {

        Matrix<double, 6, 6> H = Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();

        cost = 0;
        // compute cost
        for (int i = 0; i < nPoints; i++) {
            // compute cost for p3d[I] and p2d[I]
            // START YOUR CODE HERE
            Vector3d thisPoint = p3d[i];
            Vector3d projPoint = T_esti * thisPoint;
            Vector2d projUV(fx * projPoint[0] / projPoint[2] + cx, fy * projPoint[1] / projPoint[2] + cy);
            Vector2d e = p2d[i] - projUV;
            cost += e.squaredNorm();
            // END YOUR CODE HERE

            // compute jacobian
            Matrix<double, 2, 6> J;
            // START YOUR CODE HERE
            double X = projPoint[0];
            double Y = projPoint[1];
            double Z = projPoint[2];
            J(0, 0) = fx / Z;
            J(0, 1) = 0;
            J(0, 2) = -fx * X / (Z * Z);
            J(0, 3) = -fx * X * Y / (Z * Z);
            J(0, 4) = fx + fx * X * X / (Z * Z);
            J(0, 5) = -fx * Y / Z;
            J(1, 0) = 0;
            J(1, 1) = fy / Z;
            J(1, 2) = -fy * Y / (Z * Z);
            J(1, 3) = -fy - fy * Y * Y / (Z * Z);
            J(1, 4) = fy * X * Y / (Z * Z);
            J(1, 5) = fy * X / Z;
            J = -J;
            // END YOUR CODE HERE

            H += J.transpose() * J;
            b += -J.transpose() * e;
        }

        // solve dx
        Vector6d dx;

        // START YOUR CODE HERE 
        dx = H.ldlt().solve(b);
        // END YOUR CODE HERE

        if (isnan(dx[0])) {
            cout << "result is nan!" << endl;
            break;
        }

        if (iter > 0 && cost >= lastCost) {
            // cost increase, update is not good
            cout << "cost: " << cost << ", last cost: " << lastCost << endl;
            break;
        }

        // update your estimation
        // START YOUR CODE HERE 
        T_esti = Sophus::SE3::exp(dx) * T_esti;
        // END YOUR CODE HERE

        lastCost = cost;

        cout << "iteration " << iter << " cost=" << cout.precision(12) << cost << endl;
    }

    cout << "estimated pose: \n" << T_esti.matrix() << endl;
    return 0;
}
