#include <iostream>
#include <ctime>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;
int main(int argc, char **argv)
{
    //T_c1w
    Quaterniond q1 = Quaterniond(0.55, 0.3, 0.2, 0.2);
    q1.normalize();
    Vector3d t1 = Vector3d(0.7, 1.1, 0.2);
    Matrix4d T_c1w = Matrix4d::Identity();
    T_c1w.block(0, 0, 3, 3) = q1.matrix();
    T_c1w.block(0, 3, 3, 1) = t1;
    //T_c2w
    Quaterniond q2 = Quaterniond(-0.1, 0.3, -0.7, 0.2);
    q2.normalize();
    Vector3d t2 = Vector3d(-0.1, 0.4, 0.8);
    Matrix4d T_c2w = Matrix4d::Identity();
    T_c2w.block(0, 0, 3, 3) = q2.matrix();
    T_c2w.block(0, 3, 3, 1) = t2;
    //V_c1
    Vector4d p1 = Vector4d(0.5, -0.1, 0.2, 1);
    //V_w = T_c1w^-1 * V_c1
    Vector4d V_w = T_c1w.inverse() * p1;
    //V_c2 = T_c2w * V_w
    Vector4d V_c2 = T_c2w * V_w;
    cout <<"V_c2"<<endl<< V_c2 << endl;

    return 0;

    // //ref: WORKED
    // Quaterniond q1_ref(0.35, 0.2, 0.3, 0.1), q2_ref(-0.5,0.4,-0.1,0.2);
    // q1_ref.normalize(); q2_ref.normalize();
    // Vector3d t1_ref(0.3,0.1,0.1), t2_ref(-0.1,0.5,0.3);
    // Vector3d p1_ref(0.5,0,0.2);

    // Isometry3d T1w_ref(q1_ref), T2w_ref(q2_ref);
    // T1w_ref.pretranslate(t1_ref);
    // T2w_ref.pretranslate(t2_ref);
    // cout << "T1w_ref" << endl;
    // cout << T1w_ref.matrix() << endl;
    // cout << "T1w_ref inverse" << endl;
    // cout << T1w_ref.matrix().inverse() << endl;
    // cout << "T2w_ref" << endl;
    // cout << T2w_ref.matrix() << endl;
    // cout << "p_mid" << endl
    //      << T1w_ref.inverse() * p1_ref;
    // Vector3d p2_ref = T2w_ref * T1w_ref.inverse() * p1_ref;
    // cout << "p2_ref" << endl
    //      << p2_ref << endl;

}