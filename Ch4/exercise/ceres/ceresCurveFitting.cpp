#include <iostream>
#include <opencv2/core/core.hpp>
#include <ceres/ceres.h>
#include <chrono>

using namespace std;

struct CURVE_FITTING_COST
{
    //struct initial function
    CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}

    //template function 模板形参不能为空，一旦申明了模板形参就可以用它在声明类中的成员/函数
    template <typename T>
    //return type / function name / parameter list
    //定义了运算符(),带两个模板参数
    bool operator()(const T *const abc, T *residual)
        //function bodyconst
        const
    {
        //y-exp(ax^2+bx+c)
        residual[0] = T(_y) - ceres::exp(abc[0] * T(_x) * T(_x) + abc[1] * T(_x) + abc[2]);
        return true;
    }
    const double _x, _y;
};

int main(int argc, char **argv)
{
    double ar = 1.0, br = 2.0, cr = 1.0;
    double ae = 2.0, be = -1.0, ce = 5.0;
    int N = 100;
    double w_sigma = 1.0;
    double inv_sigma = 1.0/w_sigma;
    cv::RNG rng;

    vector<double> x_data, y_data;
    for(int i = 0; i < N; i++)
    {
        double x = i / 100.0;
        x_data.push_back(x);
        y_data.push_back(exp(ar*x*x+br*x+cr) + rng.gaussian(w_sigma*w_sigma));
    }

    double abc[3] = {ae,be,ce};

    ceres::Problem problem;
    for (int i = 0; i < N; i++)
    {
        //add residualblock to my problem
        problem.AddResidualBlock(
            //use auto-diff  //error type the template - CURVE_FITTING_COST, output dimension 1, input dimension 3 误差项为1 优化的维3
            new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 3>(new CURVE_FITTING_COST(x_data[i], y_data[i])), //每组数据都构造一个我们之前定义的对象
            //kernel function
            nullptr,
            //parameter to solve
            abc);
    }

    ceres::Solver::Options options; //huh, setting options
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY; //增量方程如何求解????? 
    options.minimizer_progress_to_stdout = true; //output to cout
    
    ceres::Solver::Summary summary;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    ceres::Solve(options, &problem, &summary); //here we go
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2-t1);
    cout<<"time used "<<time_used.count()<<endl;

    cout<<summary.BriefReport()<<endl;
    cout<<"estimated a b c : "<<endl;
    for(auto a:abc) cout<<a<<" ";
    cout<<endl;
    return 0;
}