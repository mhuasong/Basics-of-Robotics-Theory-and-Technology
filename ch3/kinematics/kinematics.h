/**
 * @file   kinematics.h
 * @author Huasong Min
 * @date   1 April 2017
 * @version 0.1
 * @brief   Some code of Kinematics of Robot 
 * @        Used in the textboook:The Basics of Robotics Theory and Technology written by Huasong Min.
 * @see https://github.com/mhuasong/ for a full description and follow-up descriptions.
 */

#ifndef KINEMATICS_H
#define KINEMATICS_H

//#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/QR>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>
#include <unsupported/Eigen/Splines>
//#include <Eigen/Geometry>
//#include <Eigen/Eigenvalues>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <time.h>

#define random(x)(rand()%x)
#define MOVLNUM 20
#define MOVPNUM 50
//Modified DH parameters,please note that initial POSE of our MDH method is different from zero POSE defined by AUBO 
static double a0=0,a1=0,a2=-0.408,a3=-0.376,a4=0,a5=0, a6=0;
static double alpha0=0,alpha1=M_PI/2,alpha2=0,alpha3=0,alpha4=M_PI/2,alpha5=-M_PI/2,alpha6=0;
static double d1=0.0985,d2=0.0,d3=0.0,d4=0.1215,d5=0.1025,d6=0.094;

//D-H kinematics
Eigen::Matrix<double,4,4> TransMatrixSDH(double a, double alpha, double d, double theta);
Eigen::Matrix<double,4,4> TransMatrixMDH(double a, double alpha, double d, double theta);
//Forward Kinematics: MDH method 
Eigen::Matrix<double,4,4> ForwardKinematics(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6);
//Forward Kinematics: SDH method, if use this method, please redefine  DH parameters again.
Eigen::Matrix<double,4,4> SDHFwd(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6);
//Forward Kinematics: Screw thoery
Eigen::Matrix<double,4,4> ForwardPoE(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6);
//Forward Kinematics: quaternion method
Eigen::Matrix<double,4,4> ForwardQuat(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6);
//Forward Kinematics: dual quaternion method
Eigen::Matrix<double,4,4> ForwardDualQuat(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6);

//Inverse Kinematics
//Mixed Solution of Analytic and Geometric Vector Method
int InverseKinematics(Eigen::Matrix<double,4,4> TcpPos, double **q , double q6Des);
Eigen::Matrix<double,1,6> getOptimalIK(Eigen::Matrix<double,4,4> Pos , double qf[6]);
Eigen::Matrix<double,1,6> getFirstIK(Eigen::Matrix<double,4,4> Pos , double qf[6]);
//Newton_Raphson Iteration Inverse Kinematics
Eigen::Matrix<double,1,6> Newton_Raphson(Eigen::Matrix<double,6,6> T, Eigen::Matrix<double,1,6> theta, Eigen::Matrix4d gz, Eigen::Matrix4d gd);
Eigen::Matrix<double,6,6> jacbobian(Eigen::Matrix<double,6,6> T, Eigen::Matrix<double,1,6> theta);
Eigen::Matrix<double,6,1> Psi(Eigen::Matrix<double,6,6> T, Eigen::Matrix<double,1,6> theta, Eigen::Matrix4d gz, Eigen::Matrix4d gd);
double norm(Eigen::Matrix<double,6,1> T);
Eigen::Matrix<double,6,6> adj(Eigen::Matrix4d X);
Eigen::Matrix4d Exp_twist(Eigen::Matrix<double,1,6> T, double theta);
Eigen::Matrix<double,6,1> Vee(Eigen::Matrix4d r);
Eigen::Matrix<double,3,3> w_hat( double *p);
int sum3v(Eigen::Matrix<double,3,1> w, Eigen::Matrix<double,3,1> z);
Eigen::Matrix3d exp_rot(double theta, Eigen::Matrix3d what);
//jacobian Iteration Inverse Kinematics
Eigen::Matrix<double,1,6> jacob_Iteration(Eigen::Matrix<double,4,4> Tobj, Eigen::Matrix<double,1,6> q);
Eigen::Matrix<double,6,6> jacob0(Eigen::Matrix<double,1,6> q);
Eigen::Matrix<double,1,6> tr2diff(Eigen::Matrix<double,4,4> T1, Eigen::Matrix<double,4,4> T2);
void pinv(Eigen::MatrixXd &input_matrix, Eigen::MatrixXd &output_matrix);

//trajactory planning function
Eigen::Matrix<double,MOVLNUM,6> ctrajMat(Eigen::Matrix<double,4,4> startPos, Eigen::Matrix<double,4,4> endPos);
Eigen::Matrix<double,MOVPNUM,6> jtrajCubic(Eigen::MatrixXd qStart, Eigen::MatrixXd qEnd); 
Eigen::Matrix<double,MOVPNUM,6> jtrajQuintic(Eigen::MatrixXd qStart, Eigen::MatrixXd qEnd);

//functions
Eigen::MatrixXd EigenTransl(double theta,double px,double py,double pz);
Eigen::Matrix3d RPY2R(double roll, double pitch, double yaw);
Eigen::Matrix<double,3,3> Quat_Rot(Eigen::Quaterniond  qd);

double limit2pi(double rad);
double regularAngle(double theta);
double AngletoRad(double angle);
double RadtoAngle(double rad);

#endif
