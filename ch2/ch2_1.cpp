/***************************
c++ source code for chapter2 written by Huasong Min
run under linux with g++ compiler
g++ -o ch2_1 ch2_1.cpp
./ch2_1
***************************/

#include <iostream> 
#include <iomanip>
#include <cmath>
#include <Eigen/Eigen>

#define PI  M_PI

using namespace std;

//function exp_rot for formula (2-11)
Eigen::Matrix3d exp_rot(double theta, Eigen::Matrix3d what){
	Eigen::Matrix3d R;
	Eigen::Matrix3d eye3;
	eye3.setIdentity(3,3);
	R = eye3+what*sin(theta)+what*what*(1-cos(theta));//formula (2-11)

	return R;
}

//function sum3v:use for calculate v in formula (2-10)
int sum3v(Eigen::Matrix<double,3,1> w, Eigen::Matrix<double,3,1> z){
	int same=0;
	if(w(0)==z(0))same=1;
	if(w(1)==z(1))same=same+1;
	if(w(2)==z(2))same=same+1;
	return same;
}

//w_hat: use to calculate the 3x3 matrix(w with a hat in formula 2-11) 
Eigen::Matrix<double,3,3> w_hat( double *p){
	Eigen::Matrix3d what;
	what = Eigen::Matrix3d::Zero();
	what(0,1) = - p[2];
	what(0,2) = p[1];
	what(1,2) = -p[0];
	what(1,0) = p[2];
	what(2,0) = -p[1];
	what(2,1) = p[0];

	return what;
}

//calculate the whole 4x4 matrix for formula 2-10
Eigen::Matrix4d Exp_twist(Eigen::Matrix<double,1,6> T, double theta){
	Eigen::Matrix4d R;
	Eigen::Matrix3d what,eye3;
	eye3.setIdentity(3,3);
	double p[3];
	Eigen::Matrix<double,3,1> v, w,z;
		
	v.block<3,1>(0,0) = T.block<1,3>(0,0);
	w.block<3,1>(0,0) = T.block<1,3>(0,3);

	p[0] = w(0,0);
	p[1] = w(1,0);
	p[2] = w(2,0);
	what = w_hat(p);
	z = Eigen::Matrix<double,3,1>::Zero();

	if(sum3v(w,z)!=3){
		R.block<3,1>(0,3) = ((eye3-exp_rot(theta,what))*(w.cross(v))+(w*(w.transpose())*v*theta));
	}else{
		R.block<3,1>(0,3) = theta*v;
	}
	
	R.block<3,3>(0,0) = exp_rot(theta,what);
	R.block<1,3>(3,0) = z.transpose();
	R(3,3) = 1;

	return R;
}

//formula 2-15
void rotate_vector_by_quaternion(Eigen::Vector4d& p, const Eigen::Vector4d& q, const Eigen::Vector4d& prime)
{
	Eigen::Vector4d  qstar;
	qstar << q(0), -q(1), -q(2), -q(3);
	Eigen::Matrix<double,4,4> MP;
	//MP:stands for M(P); two quaternion multiplication: p(X)q = M(p)q
	MP << prime(0),-prime(1),-prime(2), -prime(3),
	      prime(1),prime(0),-prime(3),prime(2),
	      prime(2),prime(3),prime(0),-prime(1),
	      prime(3),-prime(2),prime(1),prime(0);
	Eigen::Vector4d  tempV;
	tempV = MP*qstar;
	MP << q(0),-q(1),-q(2), -q(3),
	      q(1),q(0),-q(3),q(2),
	      q(2),q(3),q(0),-q(1),
	      q(3),-q(2),q(1),q(0);
	p = MP*tempV;
}

//example Demos for chapter2(ex2.3)
int main(int argc, char** argv)
{
	Eigen::Matrix<double,4,4> target_pos = Eigen::Matrix4d::Identity();
	Eigen::Matrix<double,4,4> init_pos;
	//ch2 ex2.3 if the matrix(init_pos) stands for the initial POSE
	init_pos << 1, 0, 0, 0,
		    0, 0, 1, 0,
		    0, -1, 0, 0,
		    0, 0, 0, 1;
	std::cout<<"Initial POSE Matrix:\n"<<init_pos<<std::endl;
	//step by step demo for RPY method (Step1 to Step3)
	//Step1: rotate by 90 degrees about the x axis
	Eigen::Matrix<double,4,4> first_rot;
	first_rot << 1, 0, 0,  0,
		     0, 0, -1, 0,
		     0, 1, 0,  0,
		     0, 0, 0,  1;
	target_pos = first_rot*init_pos;
	std::cout<<"Target POSE Matrix:\n"<<target_pos<<std::endl;
	//Step2: translate by 10 mm along the y axis
	Eigen::Matrix<double,4,4> second_trans;
	second_trans << 1, 0, 0,  0,
		        0, 1, 0, 10,
		        0, 0, 1,  0,
		        0, 0, 0,  1;
	target_pos = second_trans*target_pos;
	std::cout<<"Target POSE Matrix:\n"<<target_pos<<std::endl;
	//Step3: translate by 10 mm along the z axis
	Eigen::Matrix<double,4,4> third_trans;
	third_trans << 1, 0, 0,  0,
		       0, 1, 0,  0,
		       0, 0, 1, 10,
		       0, 0, 0,  1;
	target_pos = third_trans*target_pos;
	std::cout<<"Target POSE Matrix:\n"<<target_pos<<std::endl;

	//demo for screw thoery(Rodrigues)
	Eigen::Matrix<double,1,6> Twist;
	Twist<<0, 0, 0, 1, 0, 0;
	Eigen::Matrix<double,4,4> screw_pos = Eigen::Matrix4d::Identity();
	Eigen::Matrix<double,4,4> init_screw;
	init_screw << 1, 0, 0, 0,
		      0, 0, 1, 0,
		      0, -1, 0, -10,
		      0, 0, 0, 1;

	//screw_pos = Exp_twist(Twist, PI/2.0)*init_screw;//formula 2-8
	screw_pos = third_trans*Exp_twist(Twist, PI/2.0)*init_screw;
	std::cout<<"Screw POSE Matrix:\n"<<screw_pos<<std::endl;
	
	//demo for quaternion
	Eigen::Vector4d  q;
	q << cos(PI/4), sin(PI/4), 0, 0;//quaternion for the bottom center point of cone rotates 90 degrees around the screw axis S=(1,0,0)
	Eigen::Vector4d p,prime;
	prime << 0, 0, 0, -10;//The initial point P is represented by a virtual quaternion
	rotate_vector_by_quaternion(p,q,prime);//formula 2-15
	std::cout<<"rotate vector by quaternion:\n"<<p<<std::endl;
	
	//convert Quaternion to Attitude Matrix
	Eigen::Matrix<double,3,3> init_attitude, target_attitude,quat_w_theta;
	init_attitude << 1, 0, 0,
		      	 0, 1, 0,
		      	 0, 0, 1;
	//formula 2-17
	quat_w_theta << q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)-q(0)*q(3)),2*(q(1)*q(3)+q(0)*q(2)),
			2*(q(1)*q(2)+q(0)*q(3)),q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3), 2*(q(2)*q(3)-q(0)*q(1)),
			2*(q(1)*q(3)-q(0)*q(2)), 2*(q(2)*q(3)+q(0)*q(1)),q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);
	std::cout<<"rotate attitude by quaternion:\n"<<quat_w_theta<<std::endl;	


	//Rodrigues parameter method
	Eigen::Vector3d  Rho;
	Rho << q(1)/q(0),q(2)/q(0),q(3)/q(0);
	Eigen::Matrix<double,3,3> Rodrigues_attitude;
	//formula 2-20
	Rodrigues_attitude << 1+Rho(0)*Rho(0)-Rho(1)*Rho(1)-Rho(2)*Rho(2), 2*(Rho(0)*Rho(1)-Rho(2)),2*(Rho(0)*Rho(2)+Rho(1)),
			   2*(Rho(0)*Rho(1)+Rho(2)),1-Rho(0)*Rho(0)+Rho(1)*Rho(1)-Rho(2)*Rho(2),2*(Rho(1)*Rho(2)-Rho(0)),
			   2*(Rho(0)*Rho(2)-Rho(1)),2*(Rho(1)*Rho(2)+Rho(0)),1-Rho(0)*Rho(0)-Rho(1)*Rho(1)+Rho(2)*Rho(2);

	Rodrigues_attitude = Rodrigues_attitude/(1+Rho(0)*Rho(0)+Rho(1)*Rho(1)+Rho(2)*Rho(2));
	Rodrigues_attitude = Rodrigues_attitude*init_attitude;
	std::cout<<"rotate attitude by Rodrigues:\n"<<Rodrigues_attitude<<std::endl;	

	//use Rodrigues parameter method to calculate POSE Matrix
	Eigen::Matrix<double,4,4> EXP=Eigen::Matrix4d::Zero(4,4);	
	EXP.block<3,3>(0,0) = Rodrigues_attitude;
	Eigen::Vector3d  V;
	V << Twist(0),Twist(1),Twist(2);
	EXP.block<3,1>(0,3) = V.block<3,1>(0,0); 
	EXP(3,3) = 1;
	EXP = EXP*init_screw;
	std::cout<<"POSE Matrix under srew axis by Rodrigues:\n"<<EXP<<std::endl;
	target_pos = third_trans*EXP;
	std::cout<<"POSE Matrix under absolute coordinate by Rodrigues:\n"<<target_pos<<std::endl;
	
	return 0;
}


