#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
using namespace Eigen;
using namespace std;
class Com
{
protected:
    double walkfreq;
    double walktime;
    double stride;
    double freq;
    double del_t;
    double z_c = 0.29507;
    double g;
    double T_prev;
    int NL;
    Matrix3d A;
    Vector3d B;
    RowVector3d C;
    double Qe;
    Matrix3d Qx;
    Matrix4d Q_p;
    Matrix<double, 1, 1> R;
    Vector4d I_p;
    Vector4d B_p;
    Matrix<double, 4, 3> F_p;
    Matrix4d A_p;
    Matrix4d K_p;
    double Gi;
    Matrix<double, 1, 3> Gx;
    Matrix4d Ac_p;
    MatrixXd Gd;

public:
    Com();
    MatrixXd PreviewGd();
    void ShowGd();
};
class X_Com : public Com
{
private:
    double sim_time;
    int sim_n;
    MatrixXd XCom;
    RowVectorXd zmp_ref;
    RowVectorXd zmp_ref_fifo;
    RowVectorXd u;
    RowVectorXd zmp;
    RowVectorXd zmp_ref_final;
    RowVectorXd CP;
    double zmp_err_int;
    double u_prev;
    double Ref_Xpos[6] = { 0 };
public:
    void Change_Ref_Xpos(double a, double b, double c, double d, double e, double f);
    X_Com();
    MatrixXd XComSimulation();
};
class Y_Com : public Com
{
private:
    double sim_time;
    int sim_n;
    MatrixXd YCom;
    RowVectorXd zmp_ref;
    RowVectorXd zmp_ref_fifo;
    RowVectorXd u;
    RowVectorXd zmp;
    RowVectorXd zmp_ref_final;
    RowVectorXd CP;
    double zmp_err_int;
    double u_prev;
    double Ref_Ypos[6] = { 0 };
public:
    Y_Com();
    void Change_Ref_Ypos(double a, double b, double c, double d, double e, double f);
    MatrixXd YComSimulation();


};
class Foot
{
private:
    double walkfreq;
    double walktime;
    double step;
    double freq;
    Matrix<double, 6, 1> XStep;
    Matrix<double, 6, 1> XStride;
public:
    Foot();
    void Change_step(double a);
    MatrixXd Equation_solver(double t0, double t1, double start, double end);
    double Step(double t);
    double Stride(double t);
    MatrixXd RF_xsimulation_straightwalk();
    MatrixXd LF_xsimulation_straightwalk();
    MatrixXd RF_zsimulation_straightwalk();
    MatrixXd LF_zsimulation_straightwalk();
    MatrixXd RF_ysimulation_leftwalk();
    MatrixXd LF_ysimulation_leftwalk();
    MatrixXd RF_zsimulation_leftwalk();
    MatrixXd LF_zsimulation_leftwalk();
    MatrixXd RF_ysimulation_rightwalk();
    MatrixXd LF_ysimulation_rightwalk();
    MatrixXd RF_zsimulation_rightwalk();
    MatrixXd LF_zsimulation_rightwalk();

};
class BRP_Inverse_Kinematics
{
private:
    double L0 = 45;
    double L1 = 35.64;
    double L2 = 36.07;
    double L3 = 136.29;
    double L4 = 111.76;
    double L5 = 36.10;
    double L6 = 29.79;

    double FW = 92.8;
    double FL = 137.8;
    double RL_th[6] = { 0.,0.,0.,0.,0.,0. }, LL_th[6] = { 0.,0.,0.,0.,0.,0. };
    double RL_th_IK[6] = { 0.,0.,0.,0.,0.,0. }, LL_th_IK[6] = { 0.,0.,0.,0.,0.,0. };
    double Ref_RL_PR[6] = { 0.,0.,0.,0.,0.,0. }, Ref_LL_PR[6] = { 0.,0.,0.,0.,0.,0. };
    double RL_th_FK[6] = { 0.,0.,0.,0.,0.,0. }, RL_PR_FK[6] = { 0.,0.,0.,0.,0.,0. };
    double LL_th_FK[6] = { 0.,0.,0.,0.,0.,0. }, LL_PR_FK[6] = { 0.,0.,0.,0.,0.,0. };
    double Foot_Height = 0;

public:
    BRP_Inverse_Kinematics();
    void BRP_RL_FK(double th[6], double PR[6]);
    void BRP_LL_FK(double th[6], double PR[6]);
    void BRP_RL_IK(double REF_RL_RP[6], double Init_th[6], double IK_th[6]);
    void BRP_LL_IK(double Ref_LL_RP[6], double Init_th[6], double IK_th[6]);
    void inv_mat6(int m, int n, double Mat4[][4], double Mat6[][6], double c_inv4[4][4], double c_inv[6][6]);
    MatrixXd BRP_RL_Simulation(MatrixXd relRFx, MatrixXd RFy, MatrixXd RFz);
    MatrixXd BRP_LL_Simulation(MatrixXd relLFx, MatrixXd LFy, MatrixXd LFz);
};
class Motions {
private:

    MatrixXd Motion0_RL;
    MatrixXd Motion0_LL;
    MatrixXd Motion1_RL;
    MatrixXd Motion1_LL;
    MatrixXd Motion2_RL;
    MatrixXd Motion2_LL;
    MatrixXd Motion3_RL;
    MatrixXd Motion3_LL;
    MatrixXd Motion4_RL;
    MatrixXd Motion4_LL;
    MatrixXd Motion5_RL;
    MatrixXd Motion5_LL;
    MatrixXd Motion6_RL;
    MatrixXd Motion6_LL;
    double L0 = 0.045;

public:

    Motions();
    void Motion0();//inital motion
    void Motion1();//striaght walk 4 step
    void Motion2();//leftwalk 2step
    void Motion3();//step in place
    void Motion4();//rightwalk 2step
    void Motion5();//go back 4step
    void Motion6();//view com movement
    MatrixXd Return_Motion0_RL();
    MatrixXd Return_Motion0_LL();
    MatrixXd Return_Motion1_RL();
    MatrixXd Return_Motion1_LL();
    MatrixXd Return_Motion2_RL();
    MatrixXd Return_Motion2_LL();
    MatrixXd Return_Motion3_LL();
    MatrixXd Return_Motion3_RL();
    MatrixXd Return_Motion4_LL();
    MatrixXd Return_Motion4_RL();
    MatrixXd Return_Motion5_LL();
    MatrixXd Return_Motion5_RL();
    MatrixXd Return_Motion6_LL();
    MatrixXd Return_Motion6_RL();
};