/**
 * @file   kinematics.cpp
 * @author Huasong Min
 * @date   1 April 2017
 * @version 0.1
 * @brief   Some code of Kinematics of Robot 
 * @        Used in the textboook:The Basics of Robotics Theory and Technology written by Huasong Min.
 * @see https://github.com/mhuasong/ for a full description and follow-up descriptions.
 */
#include "kinematics.h"
#include "DualQuaternion.hpp"

//#define approxZero 0.00001 
const double ZERO_THRESH = 0.00001;
int SIGN(double x) {
      return (x > 0) - (x < 0);
}
const double PI = M_PI;

//Translation Matrix for SDH notation method
Eigen::Matrix<double,4,4> TransMatrixSDH(double a, double alpha, double d, double theta) {
	Eigen::Matrix<double,4,4> T;
	T.row(0)<<cos(theta),-sin(theta)*cos(alpha), sin(theta)*sin(alpha),a*cos(theta);
	T.row(1)<<sin(theta),cos(theta)*cos(alpha),-cos(theta)*sin(alpha),a*sin(theta);
	T.row(2)<<0,sin(alpha),cos(alpha),d;
	T.row(3)<<0,0,0,1;
	//cout<<"Matrix: "<<endl; 
	return T;
}

//Translation Matrix for MDH notation method
Eigen::Matrix<double,4,4> TransMatrixMDH(double a, double alpha, double d, double theta) {
	Eigen::Matrix<double,4,4> T;
	T.row(0)<<cos(theta),-sin(theta), 0, a;
	T.row(1)<<sin(theta)*cos(alpha),cos(theta)*cos(alpha),-sin(alpha),-sin(alpha)*d;
	T.row(2)<<sin(theta)*sin(alpha),cos(theta)*sin(alpha),cos(alpha),cos(alpha)*d;
	T.row(3)<<0,0,0,1;
	//cout<<"Matrix: "<<endl; 
	return T;
}

//Forward Kinematics: MDH method 
Eigen::Matrix<double,4,4> ForwardKinematics(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6){
	Eigen::Matrix<double,4,4> A1,A2,A3,A4,A5,A6;
	Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();	
	A1=TransMatrixMDH(a0,alpha0,d1,theta1);	
	A2=TransMatrixMDH(a1,alpha1,d2,theta2);
	A3=TransMatrixMDH(a2,alpha2,d3,theta3);
	A4=TransMatrixMDH(a3,alpha3,d4,theta4);
	A5=TransMatrixMDH(a4,alpha4,d5,theta5);
	A6=TransMatrixMDH(a5,alpha5,d6,theta6);

	T=A6;T=A5*T;T=A4*T;T=A3*T;T=A2*T;T=A1*T;
	
	return T;	
}

//Forward Kinematics: SDH method 
Eigen::Matrix<double,4,4> SDHFwd(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6){
	Eigen::Matrix<double,4,4> A[6];
	Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();	
	A[0]=TransMatrixSDH(a1,alpha1,d1,theta1);	
	A[1]=TransMatrixSDH(a2,alpha2,d2,theta2);
	A[2]=TransMatrixSDH(a3,alpha3,d3,theta3);
	A[3]=TransMatrixSDH(a4,alpha4,d4,theta4);
	A[4]=TransMatrixSDH(a5,alpha5,d5,theta5);
	A[5]=TransMatrixSDH(a6,alpha6,d6,theta6);
	for(int i=0;i<6;i++){
	T *=A[i];
	}
	
	return T;	
}

//Forward Kinematics: Screw thoery 
Eigen::Matrix<double,4,4> ForwardPoE(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6){
	Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
	//Table 3.5 Scales for Each Screw Axis
	Eigen::Matrix<double,1,6> T0, T1, T2, T3, T4, T5;
	T0 << 0, 0, 0, 0, 0, 1;
	T1 << 98.5, 0, 0, 0, -1, 0;
	T2 << 98.5, 0, 408,0, -1, 0;
	T3 << 98.5, 0, 784,0, -1, 0;
	T4 << 121.5, -784, 0, 0, 0, -1;
	T5 << -4, 0,784, 0, -1, 0;
	//POSE matrix of TCP at ZERO position, formula 3-38
	Eigen::Matrix4d M;
	M(0,0)=1; M(0,1)=0; M(0,2)=0; M(0,3)= -784;
	M(1,0)=0; M(1,1)=0; M(1,2)=-1; M(1,3)= -215.5;
	M(2,0)=0; M(2,1)=1; M(2,2)=0; M(2,3)= -4;
	M(3,0)=0; M(3,1)=0; M(3,2)=0; M(3,3)=1;
	T = Exp_twist(T0,theta1)*Exp_twist(T1,theta2)*Exp_twist(T2,theta3)*Exp_twist(T3,theta4)*Exp_twist(T4,theta5)*Exp_twist(T5,theta6)*M;
	return T;	
}

//Forward Kinematics: quaternion method
Eigen::Matrix<double,4,4> ForwardQuat(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6){
	Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
	Eigen::Quaterniond q1,q2,q3,q4,q5,q6;
	q1.w()=cos(theta1/2); q1.x()=0; q1.y()=0; q1.z()=sin(theta1/2);
	q2.w()=cos(theta2/2); q2.x()=0; q2.y()=-sin(theta2/2); q2.z()=0;
	q3.w()=cos(theta3/2); q3.x()=0; q3.y()=-sin(theta3/2); q3.z()=0;
	q4.w()=cos(theta4/2); q4.x()=0; q4.y()=-sin(theta4/2); q4.z()=0;
	q5.w()=cos(theta5/2); q5.x()=0; q5.y()=0; q5.z()=-sin(theta5/2);
	q6.w()=cos(theta6/2); q6.x()=0; q6.y()=-sin(theta6/2); q6.z()=0;

	Eigen::Quaterniond p1,p2,p3,p4,p5,p6;

	//p_1=(0,0,0,0), p_2=(0,0,0,H1), p_3=(0,-L1,0,H1), p_4=(0,-L1-L2,0,H1), p_5=(0,-L1-L2,-W1,H1), p_6=(0,-L1-L2,0,H1-H2)。	
	p1.w()=0; p1.x()=0; p1.y()=0; p1.z()=0;
	p2.w()=0; p2.x()=0; p2.y()=0; p2.z()=98.5;
	p3.w()=0; p3.x()=-408; p3.y()=0; p3.z()=98.5;
	p4.w()=0; p4.x()=-784; p4.y()=0; p4.z()=98.5;
	p5.w()=0; p5.x()=-784; p5.y()=-121.5; p5.z()=98.5;
	p6.w()=0; p6.x()=-784; p6.y()=0; p6.z()=-4;
		
	
	Eigen::Quaterniond q1o,q2o,q3o,q4o,q5o,q6o;
	q1o = q1*(p1*(q1.conjugate())); q1o.w()=p1.w()-q1o.w();q1o.x() = p1.x()-q1o.x();q1o.y() = p1.y()-q1o.y();q1o.z() = p1.z()-q1o.z();
	q2o = q2*(p2*(q2.conjugate())); q2o.w()=p2.w()-q2o.w();q2o.x() = p2.x()-q2o.x();q2o.y() = p2.y()-q2o.y();q2o.z() = p2.z()-q2o.z();
	q3o = q3*(p3*(q3.conjugate())); q3o.w()=p3.w()-q3o.w();q3o.x() = p3.x()-q3o.x();q3o.y() = p3.y()-q3o.y();q3o.z() = p3.z()-q3o.z();
	q4o = q4*(p4*(q4.conjugate())); q4o.w()=p4.w()-q4o.w();q4o.x() = p4.x()-q4o.x();q4o.y() = p4.y()-q4o.y();q4o.z() = p4.z()-q4o.z();
	q5o = q5*(p5*(q5.conjugate())); q5o.w()=p5.w()-q5o.w();q5o.x() = p5.x()-q5o.x();q5o.y() = p5.y()-q5o.y();q5o.z() = p5.z()-q5o.z();
	q6o = q6*(p6*(q6.conjugate())); q6o.w()=p6.w()-q6o.w();q6o.x() = p6.x()-q6o.x();q6o.y() = p6.y()-q6o.y();q6o.z() = p6.z()-q6o.z();

	Eigen::Quaterniond q16,q15,q14,q13,q12,q11,q11o,q12o,q13o,q14o,q15o,q16o,qtemp,l0(cos(PI/4),sin(PI/4),0,0),l6,p;
	q16 = q1*(q2*(q3*(q4*(q5*q6))));
	q15 = q1*(q2*(q3*(q4*q5)));
	q14 = q1*(q2*(q3*q4));
	q13 = q1*(q2*q3);
	q12 = q1*q2;

	q11 = q1;

	q11o=q1o;

	q12o=q11*(p2*q11.conjugate());
	qtemp = q12*(p2*q12.conjugate());
	q12o.w()=q12o.w()-qtemp.w();q12o.x() = q12o.x()-qtemp.x(); q12o.y() = q12o.y()-qtemp.y();q12o.z() = q12o.z()-qtemp.z();
	q12o.w()=q12o.w()+q11o.w();q12o.x() = q12o.x()+q11o.x(); q12o.y() = q12o.y()+q11o.y();q12o.z() = q12o.z()+q11o.z();

	q13o=q12*(p3*q12.conjugate());
	qtemp = q13*(p3*q13.conjugate());
	q13o.w()=q13o.w()-qtemp.w();q13o.x() = q13o.x()-qtemp.x(); q13o.y() = q13o.y()-qtemp.y();q13o.z() = q13o.z()-qtemp.z();
	q13o.w()=q13o.w()+q12o.w();q13o.x() = q13o.x()+q12o.x(); q13o.y() = q13o.y()+q12o.y();q13o.z() = q13o.z()+q12o.z();

	q14o=q13*(p4*q13.conjugate());
	qtemp = q14*(p4*q14.conjugate());
	q14o.w()=q14o.w()-qtemp.w();q14o.x() = q14o.x()-qtemp.x(); q14o.y() = q14o.y()-qtemp.y();q14o.z() = q14o.z()-qtemp.z();
	q14o.w()=q14o.w()+q13o.w();q14o.x() = q14o.x()+q13o.x(); q14o.y() = q14o.y()+q13o.y();q14o.z() = q14o.z()+q13o.z();

	q15o=q14*(p5*q14.conjugate());
	qtemp = q15*(p5*q15.conjugate());
	q15o.w()=q15o.w()-qtemp.w();q15o.x() = q15o.x()-qtemp.x(); q15o.y() = q15o.y()-qtemp.y();q15o.z() = q15o.z()-qtemp.z();
	q15o.w()=q15o.w()+q14o.w();q15o.x() = q15o.x()+q14o.x(); q15o.y() = q15o.y()+q14o.y();q15o.z() = q15o.z()+q14o.z();

	q16o=q15*(p6*q15.conjugate());
	qtemp = q16*(p6*q16.conjugate());
	q16o.w()=q16o.w()-qtemp.w();q16o.x() = q16o.x()-qtemp.x(); q16o.y() = q16o.y()-qtemp.y();q16o.z() = q16o.z()-qtemp.z();
	q16o.w()=q16o.w()+q15o.w();q16o.x() = q16o.x()+q15o.x(); q16o.y() = q16o.y()+q15o.y();q16o.z() = q16o.z()+q15o.z();
	//std::cout<<"q16o:\n"<<q16o.w()<<" "<<q16o.x()<<" "<<q16o.y()<<" "<<q16o.z()<<std::endl;
	
	l6 = q16*(l0*q16.conjugate());
	//std::cout<<"Pose(l6):\n"<<l6.w()<<" "<<l6.x()<<" "<<l6.y()<<" "<<l6.z()<<std::endl;

	Eigen::Matrix3d R = l6.normalized().toRotationMatrix();
	std::cout<<"Rotation Matrix:\n"<<R<<std::endl;
	//quaternion used right handed coordinate frame,but rviz maybe used left handed,maybe there is an error, when we use rotation matrix sometimes.
	//std::cout<<"Rotation Matrix:\n"<<R<<std::endl;

	Eigen::Quaterniond pep;
	pep.w()=0; pep.x()=-784; pep.y()=-215.5; pep.z()=-4;		
	p = q16*(pep*q16.conjugate());
	//std::cout<<"p:\n"<<p.w()<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<std::endl;
	p.x()=p.x()+q16o.x();p.y()=p.y()+q16o.y();p.z()=p.z()+q16o.z();	
	//std::cout<<"Endeffect position:\n"<<p.w()<<" "<<p.x()<<" "<<p.y()<<" "<<p.z()<<std::endl;
	//there is a bug with Rotation Matrix to combine POSE directly! 
	T.row(0)<<R(0,0),R(0,1), R(0,2),p.x();
	T.row(1)<<R(1,0),R(1,1), R(1,2),p.y();
	T.row(2)<<R(2,0),R(2,1), R(2,2),p.z();
	T.row(3)<<0,0,0,1;

	return T;	
}

//Forward Kinematics: dual quaternion method
Eigen::Matrix<double,4,4> ForwardDualQuat(double theta1,double theta2,double theta3,double theta4,double theta5,double theta6){
	Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Identity();
	Eigen::Quaterniond q1,q2,q3,q4,q5,q6;
	q1.w()=cos(theta1/2); q1.x()=0; q1.y()=0; q1.z()=sin(theta1/2);
	q2.w()=cos(theta2/2); q2.x()=0; q2.y()=-sin(theta2/2); q2.z()=0;
	q3.w()=cos(theta3/2); q3.x()=0; q3.y()=-sin(theta3/2); q3.z()=0;
	q4.w()=cos(theta4/2); q4.x()=0; q4.y()=-sin(theta4/2); q4.z()=0;
	q5.w()=cos(theta5/2); q5.x()=0; q5.y()=0; q5.z()=-sin(theta5/2);
	q6.w()=cos(theta6/2); q6.x()=0; q6.y()=-sin(theta6/2); q6.z()=0;

	std::cout<<"Using dual quaternion method...\n"<<std::endl;
	double d1 = 0.0985, a2=-0.408, a3=-0.376,d4=0.1215,d5=0.1025,d6=0.094;
	Eigen::Vector3d m1,m2,m3,m4,m5,m6;
	m1 << 0,0,d1;
	m2 << a2*cos(theta2),0,a2*sin(theta2);
	m3 << a3*cos(theta3),-d4,a3*sin(theta3);
	m4 << d5*sin(theta4),0,-d5*cos(theta4);
	m5 << -d6*sin(theta5),-d6*cos(theta5),0;
	m6 << 0,0,0;

	Eigen::Quaterniond mm1(0,m1(0),m1(1),m1(2));
	Eigen::Quaterniond mm2(0,m2(0),m2(1),m2(2));
	Eigen::Quaterniond mm3(0,m3(0),m3(1),m3(2));
	Eigen::Quaterniond mm4(0,m4(0),m4(1),m4(2));
	Eigen::Quaterniond mm5(0,m5(0),m5(1),m5(2));
	Eigen::Quaterniond mm6(0,m6(0),m6(1),m6(2));

	DualQuaternion dq1(q1,mm1),dq2(q2,mm2),dq3(q3,mm3),dq4(q4,mm4),dq5(q5,mm5),dq6(q6,mm6);

	DualQuaternion dq16(Eigen::Quaterniond(0, 0, 0, 1),Eigen::Quaterniond(0, 0, 0, 0)),dq15(Eigen::Quaterniond(0, 0, 0, 1),Eigen::Quaterniond(0, 0, 0, 0));

	dq16 = dq1*dq2*dq3*dq4*dq5*dq6;
	//std::cout<<"dq16 real:\n"<<dq16.r().w()<<"  "<<dq16.r().x()<<"  "<<dq16.r().y()<<"  "<<dq16.r().z()<<"  "<<std::endl;
	//std::cout<<"Endeffect position(dq16 dual):\n"<<dq16.d().w()<<"  "<<dq16.d().x()<<"  "<<dq16.d().y()<<"  "<<dq16.d().z()<<"  "<<std::endl;
	Eigen::Quaterniond l0(cos(PI/4),sin(PI/4),0,0);
	Eigen::Quaterniond pep;
	pep.w()=0; pep.x()=-784; pep.y()=-215.5; pep.z()=-4;	
	DualQuaternion lp(l0,pep);
	//std::cout<<"lp:\n"<<lp.r().w()<<"  "<<lp.r().x()<<"  "<<lp.r().y()<<"  "<<lp.r().z()<<"  "<<std::endl;
	//std::cout<<"lp:\n"<<lp.d().w()<<"  "<<lp.d().x()<<"  "<<lp.d().y()<<"  "<<lp.d().z()<<"  "<<std::endl;
	DualQuaternion lp6 = dq16*(lp*dq16.conjugate());
	//std::cout<<"lp6 real(Endeffect Orientation):\n"<<lp6.r().w()<<"  "<<lp6.r().x()<<"  "<<lp6.r().y()<<"  "<<lp6.r().z()<<"  "<<std::endl;
	//std::cout<<"lp6 dual:\n"<<lp6.d().w()<<"  "<<lp6.d().x()<<"  "<<lp6.d().y()<<"  "<<lp6.d().z()<<"  "<<std::endl;
	Eigen::Matrix3d R = lp6.r().toRotationMatrix();
	//there is a bug with Rotation Matrix to combine POSE directly!
	//std::cout<<"Rotation Matrix:\n"<<R<<std::endl;
	T.row(0)<<R(0,0),R(0,1), R(0,2),dq16.d().x();
	T.row(1)<<R(1,0),R(1,1), R(1,2),dq16.d().y();
	T.row(2)<<R(2,0),R(2,1), R(2,2),dq16.d().z();
	T.row(3)<<0,0,0,1;

	return T;	
}

//Makes sure minimal angle is 0
double regularAngle(double theta){
	if (theta < ZERO_THRESH && theta > -ZERO_THRESH){
		theta = 0;
	} 
	if(std::isnan(theta))theta = 0;

	return theta;
}

//
double AngletoRad(double angle){
	return angle/180.0*M_PI;
}

//
double RadtoAngle(double rad){
	return rad/M_PI*180.0;
}

//Inverse Kinematics
//Mixed Solution of Analytic and Geometric Vector Method
int InverseKinematics(Eigen::Matrix<double,4,4> TcpPOS, double **q, double q6Des){
	int numSols = 0;
	double q1[2];
	double q5[2][2];
	double q6[2][2];
	double q3[2][2][2];
	double q2[2][2][2];
	double q4[2][2][2];

	//==========================================
	//       Solving for shoulder_joint q1
	//=========================================
	  
	//the location of the fifth coordinate frame wrt the base:
	Eigen::Vector4d P0_5 = TcpPOS * Eigen::Vector4d(0, 0, -d6, 1) - Eigen::Vector4d(0, 0, 0, 1);
	double psi = atan2(P0_5[1], P0_5[0]);
	//std::cout<<"psi:"<<psi<<std::endl;
	double phi_pos;

	if((fabs(d4 / (sqrt(pow(P0_5[0],2) + pow(P0_5[1],2))))>=1)&&(fabs(d4 / (sqrt(pow(P0_5[0],2) + pow(P0_5[1],2))))<(1+ZERO_THRESH))){phi_pos=0;}
	else
	phi_pos = acos(d4 / (sqrt(pow(P0_5[0],2) + pow(P0_5[1],2))));

	//std::cout<<"d4/sqrt value:"<<d4/(sqrt(pow(P0_5[0],2) + pow(P0_5[1],2)))<<std::endl;
	//std::cout<<"phi_pos:"<<phi_pos<<std::endl;

	q1[0] = regularAngle(psi + phi_pos + M_PI/2);
	q1[1] = regularAngle(psi - phi_pos + M_PI/2);
        //std::cout<<"q1[0]:"<<psi + phi_pos + M_PI/2<<" q1[1]:"<<psi - phi_pos + M_PI/2<<std::endl;

	//==========================================
	//       Solving for wrist2_joint q5
	//=========================================
	//the location of the sixth coordinate frame wrt the base:
	//Eigen::Vector4d P0_6 = TcpPOS * Eigen::Vector4d(0, 0, 0, 1) - Eigen::Vector4d(0, 0, 0, 1);
	//For each solution to q1
	for (int i = 0; i < 2; i++)
	{
		//find transformation from frame 6 to frame 1
		Eigen::Matrix<double,4,4> T1_0 = (TransMatrixSDH(a1, alpha1, d1, q1[i])).inverse();
		Eigen::Matrix<double,4,4> T1_6 = T1_0* TcpPOS;

		//the z location of the 6th frame wrt the 1st frame 
		//double P1_6z = P0_6[0]*sin(q1[i]) - P0_6[1]*cos(q1[i]);
	    	double P1_6z = T1_6(2,3);
		double temp_trig_ratio = (P1_6z - d4) / d6;
		//std::cout<<"temp_trig_ratio:"<<temp_trig_ratio<<std::endl;
		//invalid if the argument to acos is not in [-1, 1]:
		if((fabs(temp_trig_ratio)>=1)&&(fabs(temp_trig_ratio)<(1+ZERO_THRESH))){temp_trig_ratio=1.0;}

		if (fabs(temp_trig_ratio) > 1.0) continue;
			    
		q5[i][0] = regularAngle(acos(temp_trig_ratio));
		q5[i][1] = regularAngle(-acos(temp_trig_ratio));
        	//std::cout<<"q5["<<i<<"][0]:"<<q5[i][0]<<" q5["<<i<<"][1]:"<<q5[i][1]<<std::endl;   
		//==========================================
		//       Solving for q6
		//=========================================
    
		Eigen::Matrix<double,4,4> T0_1 = TransMatrixSDH(a1, alpha1, d1, q1[i]);
		Eigen::Matrix<double,4,4> T6_1 = TcpPOS.inverse()*T0_1;

		//For each solution to q5
		for(int j = 0; j < 2; j++)
		{
			double sin_q5 = sin(q5[i][j]);
			double z6_1_y = T6_1(1,2);
			double z6_1_x = T6_1(0,2);
		      
			//invalid if sin(q5)=0 or z_x and z_y both =0, in this case q6 is free
			if ( (sin_q5 < ZERO_THRESH && sin_q5 > -ZERO_THRESH) ||
				(  (z6_1_x < ZERO_THRESH && z6_1_x > -ZERO_THRESH) && 
				(z6_1_y < ZERO_THRESH && z6_1_y > -ZERO_THRESH)  )  ){
				q6[i][j] = 0; //choose arbitrary q6
			} else {
			q6[i][j] = regularAngle(atan2( -z6_1_y / sin_q5, z6_1_x / sin_q5 ));
			}
      			//std::cout<<"q6[i][j]"<<q6[i][j]<<std::endl;
			/*==========================================
			/       Solving for q2-q4
			/=========================================*/
			//find location of frame 3 wrt frame 1
			Eigen::Matrix<double,4,4> T5_4 = (TransMatrixSDH(a5, alpha5, d5, q5[i][j])).inverse();
			Eigen::Matrix<double,4,4> T6_5 = (TransMatrixSDH(a6, alpha6, d6, q6[i][j])).inverse();
			Eigen::Matrix<double,4,4> T1_4 = T1_0 * TcpPOS * T6_5 * T5_4;
			Eigen::Vector4d P1_3 = T1_4 * Eigen::Vector4d(0, -d4, 0, 1) - Eigen::Vector4d(0,0,0,1);
      
			//solve for q3 first
			temp_trig_ratio = (pow(P1_3[0],2) + pow(P1_3[1],2) - pow(a2, 2) - pow(a3, 2)) / (2 * a2 * a3);

			if((fabs(temp_trig_ratio)>=1)&&(fabs(temp_trig_ratio)<(1+ZERO_THRESH))){temp_trig_ratio=1.0;}

			//invalid if the argument to acos is not in [-1, 1]:
			if (fabs(temp_trig_ratio) > 1) continue;
			      
			double theta3 = acos(temp_trig_ratio);
			q3[i][j][0] = regularAngle(theta3);
			q3[i][j][1] = regularAngle(-theta3);
			//std::cout<<"q3[i][j]"<<q3[i][j][0]<<" "<<q3[i][j][1]<<std::endl;      
			//For each solution to q3
			for(int k = 0; k < 2; k++)
			{  
				//solve for q2 
				q2[i][j][k] = regularAngle(-atan2(P1_3(1), -P1_3(0)) + asin(a3 * sin(q3[i][j][k]) / sqrt(pow(P1_3[0],2) + pow(P1_3[1],2))));
				//find transformation from frame 3 to 4 
				Eigen::Matrix<double,4,4> T1_2 = TransMatrixSDH(a2, alpha2, d2, q2[i][j][k]);
				Eigen::Matrix<double,4,4> T2_3 = TransMatrixSDH(a3, alpha3, d3, q3[i][j][k]);
				Eigen::Matrix<double,4,4> T3_4 = (T1_2 * T2_3).inverse() * T1_4;
				//extract q4 from it
				q4[i][j][k] = regularAngle(atan2(T3_4(1,0), T3_4(0,0)));
				//std::cout<<"q4[i][j][k]"<<q4[i][j][k]<<std::endl;
				//write joint angles to solution buffer
				q[numSols][0] = q1[i];
				q[numSols][1] = q2[i][j][k];
				q[numSols][2] = q3[i][j][k];
				q[numSols][3] = q4[i][j][k];
				q[numSols][4] = q5[i][j];
				q[numSols][5] = q6[i][j];
				numSols++;
			}    
		}  			
	
	}

	return numSols;
}

//select a optimal inverse solution
Eigen::Matrix<double,1,6> getOptimalIK(Eigen::Matrix<double,4,4> Pos, double qf[6]){
	Eigen::Matrix<double,1,6> OptimalSol;
	double *q_sol[8], q6_des;
	//double qf[6] = {0, 0, 0,0,0,0};
	int sol_num;
	int i;
	double sumAngle, tmp;
	for(i = 0; i < 8; i++){
		q_sol[i] = new double[6];
	}
		
	sol_num = InverseKinematics(Pos,q_sol,q6_des);
	
	OptimalSol.row(0) << q_sol[0][0],q_sol[0][1],q_sol[0][2],q_sol[0][3],q_sol[0][4],q_sol[0][5];
	sumAngle = fabs(q_sol[0][0]-qf[0])+fabs(q_sol[0][1]-qf[1])+fabs(q_sol[0][2]-qf[2])+fabs(q_sol[0][3]-qf[3])+fabs(q_sol[0][4]-qf[4])+fabs(q_sol[0][5]-qf[5]);
	
	//if the sum of total 6 joint rotation angles is the smallest, it is the optimal solution,just for fun
	for(i=1;i<sol_num;i++){
		tmp = fabs(q_sol[i][0]-q_sol[i-1][0])+fabs(q_sol[i][1]-q_sol[i-1][1])+fabs(q_sol[i][2]-q_sol[i-1][2])
			+fabs(q_sol[i][3]-q_sol[i-1][3])+fabs(q_sol[i][4]-q_sol[i][4])+fabs(q_sol[i][5]-q_sol[i-1][5]);
		if(tmp<sumAngle){
			OptimalSol.row(0) << q_sol[i][0],q_sol[i][1],q_sol[i][2],q_sol[i][3],q_sol[i][4],q_sol[i][5];
			sumAngle = tmp;
		}
	}

	std::cout<<"Optimal Solution is: "<<OptimalSol<<std::endl;
	return OptimalSol;
}

//select the first inverse solution
Eigen::Matrix<double,1,6> getFirstIK(Eigen::Matrix<double,4,4> Pos, double qf[6]){
	Eigen::Matrix<double,1,6> FirstSol;
	double *q_sol[8], q6_des;
	
	int sol_num;
	int i;
	double sumAngle, tmp;
	for(i = 0; i < 8; i++){
		q_sol[i] = new double[6];
	}
		
	sol_num = InverseKinematics(Pos,q_sol,q6_des);
	FirstSol.row(0) << q_sol[0][0],q_sol[0][1],q_sol[0][2],q_sol[0][3],q_sol[0][4],q_sol[0][5];

	return FirstSol;
}

//Newton_Raphson Iteration Inverse Kinematics
Eigen::Matrix<double,1,6> Newton_Raphson(Eigen::Matrix<double,6,6> T, Eigen::Matrix<double,1,6> theta, Eigen::Matrix4d gz, Eigen::Matrix4d gd){
	int ilimit = 50000;
	int count = 0;
	double err = 10.0;
	double stol = 1e-16;	
	Eigen::MatrixXd J(6,6);
	Eigen::MatrixXd J35(6,6);
	Eigen::Matrix<double,1,6> delta_theta;
	double e;

	std::cout<<"initial theta:\n"<<theta<<std::endl;

	while(err>stol){
		//std::cout<<"Iteration num:\n";
		//std::cout<<count<<std::endl;
		J = jacbobian(T,theta);

		//std::cout<<"J:"<<J<<std::endl;

		//std::cout<<"Exp:\n"<<gd<<std::endl;
		

		//J35 = pseudoInverse(J);
		//pinv(J,J35);
		Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(J);
		J35 = cqr.pseudoInverse();//using Eigen3 newest head library
		//e = (tr2diff(gd,ForwardKinematics(theta[0],theta[1],theta[2],theta[3],theta[4],theta[5]))).dot(m);
		//J35=J35*e;
		//std::cout<<J35<<std::endl;

		delta_theta = J35*Psi(T,theta,gz,gd);
		//std::cout<<"delta_theta:"<<delta_theta<<std::endl;	

		err = fabs(norm(Psi(T,theta,gz,gd)));
		//std::cout<<"err:"<<err<<std::endl;

		theta = theta-delta_theta;

		//std::cout<<"theta:"<<theta<<std::endl;
		//std::cout<<"near:\n"<<ForwardKinematics(theta(0),theta(1),theta(2),theta(3),theta(4),theta(5))<<std::endl;
		
		count = count + 1;
		
		if(count>ilimit){
			std::cout<<"Error, solution wouldn't converge"<<std::endl;
			break;
		}		
	}

	std::cout<<"Iteration num:\n";
	std::cout<<count<<std::endl;	
	return theta;
}

//calculate jacobian matrix
Eigen::Matrix<double,6,6> jacbobian(Eigen::Matrix<double,6,6> T, Eigen::Matrix<double,1,6> theta){
	Eigen::MatrixXd R(6,6);
	Eigen::Matrix<double,1,6> T0,T1,T2,T3,T4,T5;
	double theta0,theta1,theta2,theta3,theta4,theta5;
	//Eigen::Matrix<double,1,6> temp;
	
	theta0 = theta(0);
	theta1 = theta(1);
	theta2 = theta(2);
	theta3 = theta(3);
	theta4 = theta(4);
	theta5 = theta(5);
	

	T0.block<1,6>(0,0)=T.block<1,6>(0,0);
	T1.block<1,6>(0,0)=T.block<1,6>(1,0);
	T2.block<1,6>(0,0)=T.block<1,6>(2,0);
	T3.block<1,6>(0,0)=T.block<1,6>(3,0);
	T4.block<1,6>(0,0)=T.block<1,6>(4,0);
	T5.block<1,6>(0,0)=T.block<1,6>(5,0);
	

	R.block<6,1>(0,0) = T0.transpose(); //column 0
	//std::cout<<adj(Exp_twist(T0,theta0))<<std::endl<<T1<<std::endl;
	R.block<6,1>(0,1) = (adj(Exp_twist(T0,theta0))*T1.transpose()).transpose(); //column 1
	
	R.block<6,1>(0,2) = (adj(Exp_twist(T0,theta0))*adj(Exp_twist(T1,theta1))*T2.transpose()).transpose(); //column 2
	R.block<6,1>(0,3) = (adj(Exp_twist(T0,theta0))*adj(Exp_twist(T1,theta1))*adj(Exp_twist(T2,theta2))*T3.transpose()).transpose(); //column 3
	R.block<6,1>(0,4) = (adj(Exp_twist(T0,theta0))*adj(Exp_twist(T1,theta1))*adj(Exp_twist(T2,theta2))*adj(Exp_twist(T3,theta3))*T4.transpose()).transpose(); //column 4
	R.block<6,1>(0,5) = (adj(Exp_twist(T0,theta0))*adj(Exp_twist(T1,theta1))*adj(Exp_twist(T2,theta2))*adj(Exp_twist(T3,theta3))*adj(Exp_twist(T4,theta4))*T5.transpose()).transpose(); //column 5

	return R;
}

//calculate Psi in formula 3-105
Eigen::Matrix<double,6,1> Psi(Eigen::Matrix<double,6,6> T, Eigen::Matrix<double,1,6> theta, Eigen::Matrix4d gz, Eigen::Matrix4d gd){
	Eigen::MatrixXd R(6,1);
	Eigen::Matrix<double,1,6> T0,T1,T2,T3,T4,T5;
	double theta0,theta1,theta2,theta3,theta4,theta5;
	Eigen::MatrixXd temp(4,4);
	Eigen::MatrixXd gd_pinv(4,4), logm(4,4), fk_gd(4,4), fk_gz(4,4);
	fk_gd = gd;
	fk_gz = gz;
	
	theta0 = theta(0);
	theta1 = theta(1);
	theta2 = theta(2);
	theta3 = theta(3);
	theta4 = theta(4);
	theta5 = theta(5);
	
	T0.block<1,6>(0,0)=T.block<1,6>(0,0);
	T1.block<1,6>(0,0)=T.block<1,6>(1,0);
	T2.block<1,6>(0,0)=T.block<1,6>(2,0);
	T3.block<1,6>(0,0)=T.block<1,6>(3,0);
	T4.block<1,6>(0,0)=T.block<1,6>(4,0);
	T5.block<1,6>(0,0)=T.block<1,6>(5,0);
	
	
	temp = Exp_twist(T0,theta0)*Exp_twist(T1,theta1)*Exp_twist(T2,theta2)*Exp_twist(T3,theta3)*Exp_twist(T4,theta4)*Exp_twist(T5,theta5);
	
	//gd_pinv = pseudoInverse(fk_gd);
	Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(fk_gd);
	gd_pinv = cqr.pseudoInverse();
	//std::cout<<"here\n"<<std::endl;
	temp=temp*fk_gz*gd_pinv;
	//std::cout<<"temp:\n"<<temp<<std::endl;
	logm = temp.log();  //#include <unsupported/Eigen/MatrixFunctions>
	//std::cout<<"logm:\n"<<logm<<std::endl;
	R = Vee(logm);
	
	return R;
}

//V operator in formula 3-105
Eigen::Matrix<double,6,1> Vee(Eigen::Matrix4d r){
	Eigen::Matrix<double,6,1> R;
	R.block<3,1>(0,0) = r.block<3,1>(0,3);
	//R.block<1,1>(3,0) = r.block<1,1>(2,1);
	//R.block<1,1>(4,0) = r.block<1,1>(0,2);
	//R.block<1,1>(5,0) = r.block<1,1>(1,0);
	R(3,0) = r(2,1);
	R(4,0) = r(0,2);
	R(5,0) = r(1,0);
	return R;
}

double norm(Eigen::Matrix<double,6,1> T){
	return T.norm();
}

//input x SE(3)
Eigen::Matrix<double,6,6> adj(Eigen::Matrix4d X){
	Eigen::Matrix<double,6,6> R;
	double p[3];
	p[0]= X(0,3);
	p[1]= X(1,3);
	p[2]= X(2,3);

	R.block<3,3>(0,0) = X.block<3,3>(0,0);
	R.block<3,3>(0,3) = w_hat(p)*X.block<3,3>(0,0);
	R.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
	//std::cout<<"adj zero:"<<R.block<3,3>(3,0)<<std::endl;
	R.block<3,3>(3,3) = X.block<3,3>(0,0);

	return R;
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
	//std::cout<<"Exp_twist\n"<<std::endl;

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

//function exp_rot for formula (2-11)
Eigen::Matrix3d exp_rot(double theta, Eigen::Matrix3d what){
	Eigen::Matrix3d R;
	Eigen::Matrix3d eye3;
	eye3.setIdentity(3,3);
	//std::cout<<"exp:"<<theta<<" "<<sin(theta)<<" "<<cos(theta)<<std::endl;
	R = eye3+what*sin(theta)+what*what*(1-cos(theta));

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

//jacobian Iteration Inverse Kinematics
Eigen::Matrix<double,1,6> jacob_Iteration(Eigen::Matrix<double,4,4> Tobj, Eigen::Matrix<double,1,6> q){
	int ilimit = 50000;
	double stol = 1e-16;
	//single xform case
	double nm = 1.0;
	int count = 0;
	Eigen::Matrix<double,1,6> e;
	//double e;
	Eigen::Matrix<double,6,1>  m;
	Eigen::Matrix<double,1,6> dq;
	dq = Eigen::MatrixXd::Zero(1,6);

	m(0,0)=1;m(1,0)=1;m(2,0)=1;m(3,0)=1;m(4,0)=1;m(5,0)=1;
	//m.Ones(6,1);
	//std::cout<<"m:"<<m<<std::endl;

	Eigen::MatrixXd T(6,6) , pInv(6,6);
	
	Eigen::Matrix<double,1,6> qt;
	//Eigen::MatrixXd::Index maxIndex;
	Eigen::Matrix<double,6,1> matE;

	Eigen::Matrix<double,1,6> jq;

	Eigen::Matrix<double,1,6> T0, T1, T2, T3, T4, T5;
	T0 << 0, 0, 0, 0, 0, 1;
	T1 << 98.5, 0, 0, 0, -1, 0;
	T2 << 98.5, 0, 408,0, -1, 0;
	T3 << 98.5, 0, 784,0, -1, 0;
	T4 << 121.5, -784, 0, 0, 0, -1;
	T5 << -4, 0,784, 0, -1, 0;

	Eigen::Matrix<double,6,6> TT;
	TT.block<1,6>(0,0) = T0;
	TT.block<1,6>(1,0) = T1;
	TT.block<1,6>(2,0) = T2;
	TT.block<1,6>(3,0) = T3;
	TT.block<1,6>(4,0) = T4;
	TT.block<1,6>(5,0) = T5;

	
	while(nm>stol){
		e = tr2diff(ForwardKinematics(q[0],q[1],q[2],q[3],q[4],q[5]),Tobj);
		//std::cout<<"e:"<<e<<std::endl;
		jq<<q[0],q[1],q[2],q[3],q[4],q[5];
		T = jacob0(q);
		//T = jacbobian(TT,q);

		pInv = T.completeOrthogonalDecomposition().pseudoInverse();		
		//std::cout<<"pInv:"<<pInv<<std::endl;
		//std::cout<<"pInv*e:"<<pInv*e.transpose()<<std::endl;
		dq = pInv*e.transpose();
		//dq = pInv*e;
		//std::cout<<"dq:"<<dq<<std::endl;
		q = q+dq;

		//q=q+dq.block<1,6>(0,0);
		//std::cout<<"dqblock:"<<dq.block<1,6>(0,0)<<std::endl;//?
		//std::cout<<"q:"<<q<<std::endl;	

		nm=dq.norm();
		//nm=dq.colwise().sum().maxCoeff(&maxIndex);
		//nm=pow(dq.norm(),2);
		//nm=dq.norm();
		//std::cout<<"nm:"<<nm<<std::endl;

		count = count + 1;
		if(count>ilimit){
			std::cout<<"Error, solution wouldn't converge"<<std::endl;
			break;
		}

	}
	std::cout<<"count:"<<count<<std::endl;
	qt = q;
	return qt;
}

//calculate jacob0
Eigen::Matrix<double,6,6> jacob0(Eigen::Matrix<double,1,6> q){
	Eigen::MatrixXd Jv(3, 6);
	Eigen::MatrixXd Jw(3, 6);
  
	Eigen::Vector4d o_0(0,0,0,1);
	Eigen::Vector4d o_6 = ForwardKinematics(q[0],q[1],q[2],q[3],q[4],q[5]) * o_0;
	//The origin of frame i:
	Eigen::Vector4d o_i;
  
	Eigen::Vector3d e3(0,0,1);
	//The z-axis of frame i:
	Eigen::Vector3d z_i;
  
	//The transformation from frame 0 to frame i:
	Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

  	double alpha[6],a[6],d[6];
	alpha[0]=alpha0;alpha[1]=alpha1;alpha[2]=alpha2;alpha[3]=alpha3;alpha[4]=alpha4;alpha[5]=alpha5;alpha[5]=alpha6;
	a[0]=a0;a[1]=a1;a[2]=a2;a[3]=a3;a[4]=a4;a[5]=a5;a[6]=a6;
	d[0]=d1;d[1]=d2;d[2]=d3;d[3]=d4;d[4]=d5;d[5]=d6;
  
	for(int i = 0; i < 6; i++){
		o_i = T * o_0;
		z_i = T.block<3,3>(0,0) * e3;
    
		//build Jv one column at a time:
		Jv.block<3,1>(0,i) = z_i.cross((o_6 - o_i).head<3>());
    
		//build Jw one column at a time:
		Jw.block<3,1>(0,i) = z_i;
    
		T *= TransMatrixSDH(a[i+1], alpha[i+1], d[i], q[i]);
		
	}
  
	//Put Jv and Jw together
	Eigen::MatrixXd J(6,6);
	J.block<3,6>(0,0) = Jv;
	J.block<3,6>(3,0) = Jw;
  
	return J;
}

//calculate the iteration error
Eigen::Matrix<double,1,6> tr2diff(Eigen::Matrix<double,4,4> T1, Eigen::Matrix<double,4,4> T2){
	Eigen::Matrix<double,1,6> d;
	d = Eigen::MatrixXd::Identity(1,6);
	Eigen::Vector3d  tmp,tmp1,tmp2;
	
	//std::cout<<"block1:"<<T1.block(0,0,1,3)<<std::endl<<"block2:"<<T2.block(0,0,1,3)<<std::endl;
	tmp1 << T1(0,0),T1(1,0),T1(2,0);
	tmp2 << T2(0,0),T2(1,0),T2(2,0);
	tmp=tmp1.cross(tmp2);

	tmp1 << T1(0,1),T1(1,1),T1(2,1);
	tmp2 << T2(0,1),T2(1,1),T2(2,1);
	tmp+=tmp1.cross(tmp2);
	
	tmp1 << T1(0,2),T1(1,2),T1(2,2);
	tmp2 << T2(0,2),T2(1,2),T2(2,2);
	tmp += tmp1.cross(tmp2);

	tmp = 0.5*tmp;

	//std::cout<<"tmp:\n"<<tmp<<std::endl;
	//tmp = 0.5*(T1.block(0,0,1,1)*T2.block(0,0,1,1)+T1.block(0,1,1,1)*T2.block(0,1,1,1)+T1.block(0,2,1,1)*T2.block(0,2,1,1));
	
	d << T2(0,3)-T1(0,3), T2(1,3)-T1(1,3),T2(2,3)-T1(2,3), tmp(0,0),tmp(1,0),tmp(2,0);
	//std::cout<<"d:"<<d<<std::endl;
	return d;
}

//trajectory planning
//ctraj function 
Eigen::Matrix<double,MOVLNUM,6> ctrajMat(Eigen::Matrix<double,4,4> startPos, Eigen::Matrix<double,4,4> endPos){
	Eigen::MatrixXd radVector= Eigen::Matrix<double,1,6>::Identity();
	
	int step;
	Eigen::MatrixXd temPos(4,4);
	
	step=MOVLNUM;
	Eigen::MatrixXd rtnMatPoints(step,6);	
	double qf[6] = {0, M_PI/2, 0, M_PI/2 , 0 ,0};//start point joint angle	
	

	double lx;
	double ly;
	double lz;
	lx=endPos(0,3)-startPos(0,3);
	ly=endPos(1,3)-startPos(1,3);
	lz=endPos(2,3)-startPos(2,3);

	float deltax,deltay,deltaz;

	deltax=(double)lx/step;
	deltay=(double)ly/step;
	deltaz=(double)lz/step;

	temPos=startPos;
	
	for(int i=0;i<step;++i)
	{
		temPos(0,3)=startPos(0,3)+(i+1)*(deltax);
		temPos(1,3)=startPos(1,3)+(i+1)*(deltay);
		temPos(2,3)=startPos(2,3)+(i+1)*(deltaz);
		
		radVector = getOptimalIK(temPos,qf);
		
		for(int j=0;j<6;++j)
		{
			rtnMatPoints(i,j)= radVector(0,j);
		}

	}
	std::cout<<"planning points matrix:\n"<<rtnMatPoints<<std::endl;
	
	return rtnMatPoints;
}

//jtraj function
Eigen::Matrix<double,MOVPNUM,6> jtrajCubic(Eigen::MatrixXd qStart, Eigen::MatrixXd qEnd){
	int n;
	double m;
	n=MOVPNUM;
	m=n-1;
	Eigen::MatrixXd t(n,1);

	Eigen::MatrixXd q0(1,6), q1(1,6);
	q0=qStart; 
	q1=qEnd; 
	
	int i=0;
	double tscal=n;
	//normalized time from 0->1
	for(;i<n;i++)
	{
		t(i,0)=i/m;
	}
	
	Eigen::MatrixXd qd0(1,6); //not using velocities boundary [qd0 qd1]
	qd0 = Eigen::Matrix<double,1,6>::Zero();
	Eigen::MatrixXd qd1(1,6);
	qd1 = Eigen::Matrix<double,1,6>::Zero();
	
	Eigen::MatrixXd A(1,6);
	Eigen::MatrixXd B(1,6);
	Eigen::MatrixXd C(1,6);

	
	//compute the polynomal coefficients
	A = -2*(q1-q0);
	B = 3*(q1 - q0);
	C = q0;


	Eigen::MatrixXd tt(n,4);
	for(i=0;i<n;i++){
		tt(i,0) = pow(t(i,0),3);
		tt(i,1) = pow(t(i,0),2);
		tt(i,2) = pow(t(i,0),1);
		tt(i,3) = 1;
	}

	Eigen::MatrixXd c(4,6);
	Eigen::MatrixXd temp(1,6);
	temp<<Eigen::MatrixXd::Zero(1,6);
	for(int i=0;i<6;i++)
	{
		c(0,i)=A(0,i);
		c(1,i)=B(0,i);
		c(2,i)=temp(0,i);
		c(3,i)=C(0,i);
	}

	Eigen::MatrixXd qt(n,6);
	
	qt=tt*c;

	for(int i=0;i<n;++i)
	{
		for(int j=0;j<6;++j)
		{
			if(fabs(qt(i,j))<ZERO_THRESH)
				qt(i,j)=0;
		}
	}

	//calculate velocity
	Eigen::MatrixXd qdt(n,6);
	for(int i=0;i<6;i++)
	{
		c(0,i)=temp(0,i);
		c(1,i)=3*A(0,i);
		c(2,i)=2*B(0,i);
		c(3,i)=temp(0,i);
	}
	qdt = tt*c/tscal;

	//calculate acceleration
	Eigen::MatrixXd qddt(n,6);
	for(int i=0;i<6;i++)
	{
		c(0,i)=temp(0,i);
		c(1,i)=6*A(0,i);
		c(2,i)=2*B(0,i);
		c(3,i)=temp(0,i);
	}
	qddt = tt*c/pow(tscal,2);


	return qt;
	
}

//jtrajQuintic
Eigen::Matrix<double,MOVPNUM,6> jtrajQuintic(Eigen::MatrixXd qStart, Eigen::MatrixXd qEnd){
	int n;
	double m;
	n=MOVPNUM;
	m=n-1;
	Eigen::MatrixXd t(n,1);

	Eigen::MatrixXd q0(1,6), q1(1,6);
	q0=qStart; 
	q1=qEnd; 
	
	int i=0;
	double tscal=n;
	//normalized time from 0->1
	for(;i<n;i++)
	{
		t(i,0)=i/m;
	}
	
	Eigen::MatrixXd qd0(1,6); //not using velocities boundary [qd0 qd1]
	qd0 = Eigen::Matrix<double,1,6>::Zero();
	Eigen::MatrixXd qd1(1,6);
	qd1 = Eigen::Matrix<double,1,6>::Zero();
	
	Eigen::MatrixXd A(1,6);
	Eigen::MatrixXd B(1,6);
	Eigen::MatrixXd C(1,6);
	Eigen::MatrixXd E(1,6);
	Eigen::MatrixXd F(1,6);
	
	//compute the polynomal coefficients
	A = 6*(q1-q0)-3*(qd1+qd0)*tscal;
	B = -15*(q1 - q0) + (8*qd0 + 7*qd1)*tscal;
	C = 10*(q1 - q0) - (6*qd0 + 4*qd1)*tscal;
	E = qd0*tscal;
	F = q0;


	Eigen::MatrixXd tt(n,6);
	for(i=0;i<n;i++){
		tt(i,0) = pow(t(i,0),5);
		tt(i,1) = pow(t(i,0),4);
		tt(i,2) = pow(t(i,0),3);
		tt(i,3) = pow(t(i,0),2);
		tt(i,4) = pow(t(i,0),1);
		tt(i,5) = 1;
	}

	Eigen::MatrixXd c(6,6);
	Eigen::MatrixXd temp(1,6);
	temp<<Eigen::MatrixXd::Zero(1,6);
	for(int i=0;i<6;i++)
	{
		c(0,i)=A(0,i);
		c(1,i)=B(0,i);
		c(2,i)=C(0,i);
		c(3,i)=temp(0,i);
		c(4,i)=E(0,i);
		c(5,i)=F(0,i);
	}

	Eigen::MatrixXd qt(n,6);
	qt=tt*c;

	for(int i=0;i<n;++i)
	{
		for(int j=0;j<6;++j)
		{
			if(fabs(qt(i,j))<ZERO_THRESH)
				qt(i,j)=0;
		}
	}

	//calculate velocity
	Eigen::MatrixXd qdt(n,6);
	for(int i=0;i<6;i++)
	{
		c(0,i)=temp(0,i);
		c(1,i)=5*A(0,i);
		c(2,i)=4*B(0,i);
		c(3,i)=3*C(0,i);
		c(4,i)=temp(0,i);
		c(5,i)=E(0,i);
	}
	qdt = tt*c/tscal;

	//calculate acceleration
	Eigen::MatrixXd qddt(n,6);
	for(int i=0;i<6;i++)
	{
		c(0,i)=temp(0,i);
		c(1,i)=temp(0,i);
		c(2,i)=20*A(0,i);
		c(3,i)=12*B(0,i);
		c(4,i)=6*C(0,i);
		c(5,i)=temp(0,i);
	}
	qddt = tt*c/pow(tscal,2);


	return qt;
	
}

//translation use Eigen AngleAxis
Eigen::MatrixXd EigenTransl(double theta,double px,double py,double pz){
	Eigen::Vector3d v(px,py,pz);
	Eigen::AngleAxisd rotation_vector(theta,v);
	
	Eigen::Matrix3d rotation_matrix;
	rotation_matrix=rotation_vector.toRotationMatrix();

	Eigen::Matrix4d Trans;
	Trans.setIdentity();
	Trans.block<3,3>(0,0)=rotation_matrix;
	Trans.rightCols(1)<<px,py,pz,1;
	//std::cout<<Trans<<std::endl;
	return Trans;
}


void pinv(Eigen::MatrixXd &input_matrix, Eigen::MatrixXd &output_matrix){
	output_matrix = Eigen::MatrixXd::Identity(input_matrix.rows(),input_matrix.cols());
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(input_matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);

	output_matrix = svd.solve(output_matrix);
}

double limit2pi(double rad){
	double r = rad;
	if(r<-2*M_PI)
	while(r<-M_PI){
		r = r + 2*M_PI;	
		if(r>-M_PI)break;
	}

	if(r>2*M_PI)
	while(r>M_PI){
		r = r -2*M_PI;
		if(r<M_PI)break;
	}

	//r= fmod(rad,2*M_PI);
	//r=asin(sin(rad));
	std::cout<<"rad:"<<rad<<" r:"<<r<<std::endl;
	return r;
}

Eigen::Matrix<double,3,3> Quat_Rot(Eigen::Quaterniond  qd){
	Eigen::Matrix<double,3,3> quat_w_theta;
	Eigen::Vector4d  q;
	q << qd.w(), qd.x(), qd.y(), qd.z();

	quat_w_theta << q(0)*q(0)+q(1)*q(1)-q(2)*q(2)-q(3)*q(3), 2*(q(1)*q(2)-q(0)*q(3)),2*(q(1)*q(3)+q(0)*q(2)),
			2*(q(1)*q(2)+q(0)*q(3)),q(0)*q(0)-q(1)*q(1)+q(2)*q(2)-q(3)*q(3), 2*(q(2)*q(3)-q(0)*q(1)),
			2*(q(1)*q(3)-q(0)*q(2)), 2*(q(2)*q(3)+q(0)*q(1)),q(0)*q(0)-q(1)*q(1)-q(2)*q(2)+q(3)*q(3);
	return quat_w_theta;
}


Eigen::Matrix3d RPY2R(double roll, double pitch, double yaw){
	Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
	Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
	Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
	Eigen::Quaterniond  q = rollAngle*pitchAngle*yawAngle;

	Eigen::Matrix3d rotationMatrix = q.matrix();
	return rotationMatrix;
}


