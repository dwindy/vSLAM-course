#include <iostream>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
int main(int argc, char **argv)
{
    Matrix<double, 100, 100> A = MatrixXd::Random(100, 100);
    Matrix<double, 100, 1> B = MatrixXd::Random(100, 1);
    A = A * A.transpose();

    clock_t time_start = clock();
    //directly solve A^-1
    Matrix<double, 100, 1> X = A.inverse() * B;
    cout << "time for direct solution is : " << 1000 * (clock() - time_start) / (double)CLOCKS_PER_SEC << " ms" << endl;
    cout << "X.transpose = " << X.transpose()<<endl;

    //QR solution
    time_start = clock();
    X = A.colPivHouseholderQr().solve(B);
    cout << "time for QR solution is : " << 1000 * (clock() - time_start) / (double)CLOCKS_PER_SEC << " ms" << endl;
    cout << "X.transpose = " << X.transpose()<<endl;

    //Cholesky solution
    time_start = clock();
    X = A.ldlt().solve(B);
    cout << "time for Cholesky solution is : " << 1000 * (clock() - time_start) / (double)CLOCKS_PER_SEC << " ms" << endl;
    cout << "X.transpose = " << X.transpose()<<endl;

    return 0;
}