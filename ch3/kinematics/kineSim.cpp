/**
 * @file   kineSim.cpp
 * @author Huasong Min
 * @date   1 April 2017
 * @version 0.1
 * @brief   Some code of Kinematics of Robot 
 * @        Used in the textboook:The Basics of Robotics Theory and Technology written by Huasong Min.
 * @see https://github.com/mhuasong/ for a full description and follow-up descriptions.
 */

#include <iostream> 
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <robot_state_publisher/robot_state_publisher.h> 
#include "kinematics.h"

using namespace std; 

double joint1Lim_min=-M_PI*175.0/180.0,joint1Lim_max=M_PI*175.0/180.0;
double joint2Lim_min=-M_PI*175.0/180.0,joint2Lim_max=M_PI*175.0/180.0;
double joint3Lim_min=-M_PI*175.0/180.0,joint3Lim_max=M_PI*175.0/180.0;
double joint4Lim_min=-M_PI*175.0/180.0,joint4Lim_max=M_PI*175.0/180.0;
double joint5Lim_min=-M_PI*175.0/180.0,joint5Lim_max=M_PI*175.0/180.0;
double joint6Lim_min=-M_PI*175.0/180.0,joint6Lim_max=M_PI*175.0/180.0;

sensor_msgs::JointState joint_state; 
//jointSate publisher node
ros::Publisher joint_pub;
//marker publisher node
ros::Publisher marker_pub;
//define a visualization_msgs/Marker
visualization_msgs::Marker points;

//draw points on rviz, please add marker form rviz configuration panel
int DisplayPoint(double x,double y,double z){
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z+0.02;
	//printf("%lf %lf %lf\n",x,y,z);
	points.header.stamp = ros::Time::now();
	points.points.push_back(p);

	//发布point marker
	marker_pub.publish(points);
	
	return 0;
}

//Draw workspace
int Monte_carlo_Workspace(){

	int N=150000;
	double t1[N],t2[N],t3[N],t4[N],t5[N],t6[N];
	Eigen::Matrix<double,4,4> A1,A2,A3,A4,A5,A6,T;
	srand(time(0));
	for(int i=0;i<N;i++){	
		t1[i]=joint1Lim_min+(joint1Lim_max-joint1Lim_min)*random(N)/N;		
		t2[i]=joint2Lim_min+(joint2Lim_max-joint2Lim_min)*random(N)/N;	
		t3[i]=joint3Lim_min+(joint3Lim_max-joint3Lim_min)*random(N)/N;	
		t4[i]=joint4Lim_min+(joint4Lim_max-joint4Lim_min)*random(N)/N;	
		t5[i]=joint5Lim_min+(joint5Lim_max-joint5Lim_min)*random(N)/N;	
		t6[i]=joint6Lim_min+(joint6Lim_max-joint6Lim_min)*random(N)/N;
	}

	for(int i=0;i<N;i++){
		A1=TransMatrixMDH(a0,alpha0,d1,t1[i]);	
		A2=TransMatrixMDH(a1,alpha1,d2,t2[i]-M_PI/2.0);
		A3=TransMatrixMDH(a2,alpha2,d3,-t3[i]);
		A4=TransMatrixMDH(a3,alpha3,d4,t4[i]-M_PI/2.0);
		A5=TransMatrixMDH(a4,alpha4,d5,t5[i]);
		A6=TransMatrixMDH(a5,alpha5,d6,t6[i]);
		//T=A1*A2*A3*A4*A5*A6;
		T=A6;T=A5*T;T=A4*T;T=A3*T;T=A2*T;T=A1*T;

		double pointX=T(0,3);
		double pointY=T(1,3); 
		double pointZ=T(2,3);
		DisplayPoint(pointX,pointY,pointZ);
	}
	return 0;
}

//move joints in rviz
int moveJ(double j1,double j2,double j3,double j4,double j5,double j6){
	//update joint_state 
	joint_state.header.stamp = ros::Time::now(); 
	joint_state.header.frame_id= "";
	joint_state.name.resize(6); 
	joint_state.position.resize(6); 
			
	joint_state.name[0]="shoulder_joint"; 
	joint_state.position[0] = j1; 
	//ROS_INFO("shoulder: %f  ",joint_state.position[0]);

	joint_state.name[1] ="upperArm_joint"; 
	joint_state.position[1] = j2; 
	//ROS_INFO("upperArm: %f  ",joint_state.position[1]);

	joint_state.name[2] ="foreArm_joint"; 
	joint_state.position[2] = j3; 
	//ROS_INFO("foreArm: %f  ",joint_state.position[2]);

	joint_state.name[3] ="wrist1_joint"; 
	joint_state.position[3] = j4; 
	//ROS_INFO("wrist1: %f  ",joint_state.position[3]);

	joint_state.name[4] ="wrist2_joint"; 
	joint_state.position[4] = j5; 
	//ROS_INFO("wrist2: %f  ",joint_state.position[4]);

	joint_state.name[5] ="wrist3_joint"; 
	joint_state.position[5] = j6; 
	//ROS_INFO("wrist3: %f\n",joint_state.position[5]);

	joint_state.velocity.resize(6);
	joint_state.effort.resize(6);

	joint_state.velocity.resize(6);
	joint_state.effort.resize(6);
		
	//send the joint state and transform 
	joint_pub.publish(joint_state); 

	ros::Duration(0.1).sleep();//animate

	return 0;
}

//test ctraj function
void testCTraj(){
	Eigen::Matrix<double,4,4> startPos, endPos;
	int step = MOVLNUM;
	Eigen::MatrixXd PlanPoints(step,6);
	
	points.header.stamp = ros::Time::now();		
	points.action = visualization_msgs::Marker::ADD;
				
	points.scale.x = 0.01f;
	points.scale.y = 0.01f;
	points.scale.z = 0.01f;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	

	startPos = ForwardKinematics(0,-M_PI/2.0,-0,-M_PI/2.0,0,0);//from start position real pos= 0 0 0 0 0 0
	endPos = ForwardKinematics(1.570796,-0.785398,0,-1.570796,0.174533,0.174533);//to an end position

	DisplayPoint(startPos(0,3),startPos(1,3),startPos(2,3));
	DisplayPoint(endPos(0,3),endPos(1,3),endPos(2,3));

	points.color.r = 0.0f;
	points.color.g = 1.0f;

	double q[step][6];
	Eigen::Matrix<double,4,4> TcpCenter;
	double pointX,pointY,pointZ;

	moveJ(0,0,0,0,0,0);//move to start pos	
	//getchar();

	PlanPoints = ctrajMat(startPos,endPos);
	//getchar();

	for(int i=0;i<step;i++){

		for(int j=0;j<6;j++){
			q[i][j]=PlanPoints(i,j);
		}

		moveJ(q[i][0],q[i][1]+M_PI/2.0,-q[i][2],q[i][3]+M_PI/2.0,q[i][4],q[i][5]);

		
		TcpCenter=ForwardKinematics(q[i][0],q[i][1],q[i][2],q[i][3],q[i][4],q[i][5]);
	
		pointX=TcpCenter(0,3);
		pointY=TcpCenter(1,3);
		pointZ=TcpCenter(2,3);
	

		DisplayPoint(pointX,pointY,pointZ);
		//getchar();
	}
}

//test jtraj function
void testJTraj(){
	Eigen::Matrix<double,4,4> startPos, endPos;
	int step = MOVPNUM;
	Eigen::MatrixXd PlanPoints(step,6);
	
	points.header.stamp = ros::Time::now();		
	points.action = visualization_msgs::Marker::ADD;
				
	points.scale.x = 0.01f;
	points.scale.y = 0.01f;
	points.scale.z = 0.01f;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	
	
	Eigen::MatrixXd qStart(1,6),qEnd(1,6);
	qStart<<0,-M_PI/2.0,-0,-M_PI/2.0,0,0;
	qEnd<<1.570796,-0.785398,0,-0.785398,0.174533,0.174533;

	std::cout<<"starting draw start point and end point"<<std::endl;

	startPos = ForwardKinematics(0,-M_PI/2.0,-0,-M_PI/2.0,0,0);//from start position real pos= 0 0 0 0 0 0
	endPos = ForwardKinematics(1.570796,-0.785398,0,-0.785398,0.174533,0.174533);//to an end position

	DisplayPoint(startPos(0,3),startPos(1,3),startPos(2,3));
	DisplayPoint(endPos(0,3),endPos(1,3),endPos(2,3));
	
	std::cout<<startPos<<std::endl<<endPos<<std::endl;
	points.color.r = 0.0f;
	points.color.g = 1.0f;
	
	
	
	double q[step][6];
	Eigen::Matrix<double,4,4> TcpCenter;
	double pointX,pointY,pointZ;

	moveJ(0,0,0,0,0,0);//move to start pos

	//getchar();
	printf("\nPress 1 for Quintic polynomial. \
	\nPress any key for Cubic polynomial.\n");
	int command;	
	scanf("%d",&command);
	if(command==1){
		std::cout<<"Quintic polynomial."<<std::endl;
		PlanPoints = jtrajQuintic(qStart,qEnd);
	}
	else{
		std::cout<<"Cubic polynomial."<<std::endl;
		PlanPoints = jtrajCubic(qStart,qEnd);
	}
	//getchar();

	for(int i=0;i<step;i++){

		for(int j=0;j<6;j++){
			q[i][j]=PlanPoints(i,j);
		}

		moveJ(q[i][0],q[i][1]+M_PI/2.0,-q[i][2],q[i][3]+M_PI/2.0,q[i][4],q[i][5]);

		TcpCenter=ForwardKinematics(q[i][0],q[i][1],q[i][2],q[i][3],q[i][4],q[i][5]);
	
		pointX=TcpCenter(0,3);
		pointY=TcpCenter(1,3);
		pointZ=TcpCenter(2,3);
	

		DisplayPoint(pointX,pointY,pointZ);
		//getchar();
	}
}

//
int testMoveP(){
	Eigen::Matrix<double,1,6> qStart, qEnd; 
	int step = MOVPNUM;
	Eigen::MatrixXd PlanPoints(step,6);
	Eigen::Matrix<double,4,4> TcpCenter;
	double pointX,pointY,pointZ;
	double q[step][6];

	points.header.stamp = ros::Time::now();		
	points.action = visualization_msgs::Marker::ADD;
				
	points.scale.x = 0.01f;
	points.scale.y = 0.01f;
	points.scale.z = 0.01f;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	

	double q6_des;
	double qf[6]={0, M_PI/2, 0, M_PI/2 , 0 ,0};//assume this initial angle
	double point1[3],point2[3],rollAngle,pitchAngle,yawAngle;

	printf("Please input start position x y z:\n");
	scanf("%lf%lf%lf",&point1[0],&point1[1],&point1[2]);
	printf("Please input roll angle:\n");
	scanf("%lf",&rollAngle);
	rollAngle=rollAngle*M_PI/180.0;
	printf("Please input pitch angle:\n");
	scanf("%lf",&pitchAngle);
	pitchAngle=pitchAngle*M_PI/180.0;
	printf("Please input yaw angle:\n");
	scanf("%lf",&yawAngle);
	yawAngle=yawAngle*M_PI/180.0;

	printf("Please input end position x y z:\n");
	scanf("%lf%lf%lf",&point2[0],&point2[1],&point2[2]);

	DisplayPoint(point1[0],point1[1],point1[2]);
	DisplayPoint(point2[0],point2[1],point2[2]);
	//getchar();
	points.color.r = 0.0f;
	points.color.g = 1.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	

	Eigen::Matrix<double,4,4> T1,T2;
	Eigen::Matrix3d poseT;
	poseT = RPY2R(rollAngle,pitchAngle,yawAngle);

	// start position
	T1.block<3,3>(0,0) = poseT.block<3,3>(0,0);
	T1(0,3) = point1[0];
	T1(1,3) = point1[1];
	T1(2,3) = point1[2];
	T1(3,3) = 1;
	std::cout<<"T1:\n"<<T1<<std::endl;
	
	qStart = getFirstIK(T1,qf);

	// target position
	T2.block<3,3>(0,0) = poseT.block<3,3>(0,0);
	T2(0,3) = point2[0];
	T2(1,3) = point2[1];
	T2(2,3) = point2[2];
	T2(3,3) = 1;	
	std::cout<<"T2:\n"<<T2<<std::endl;

	qEnd = getFirstIK(T2,qf);

	PlanPoints = jtrajQuintic(qStart,qEnd);
	//getchar();

	for(int i=0;i<step;i++){

		for(int j=0;j<6;j++){
			q[i][j]=PlanPoints(i,j);
		}

		moveJ(q[i][0],q[i][1]+M_PI/2.0,-q[i][2],q[i][3]+M_PI/2.0,q[i][4],q[i][5]);

		TcpCenter=ForwardKinematics(q[i][0],q[i][1],q[i][2],q[i][3],q[i][4],q[i][5]);
	
		pointX=TcpCenter(0,3);
		pointY=TcpCenter(1,3);
		pointZ=TcpCenter(2,3);
	
		DisplayPoint(pointX,pointY,pointZ);
		//getchar();
	}
}

//test Inverse Kinematics of jacob Iteration method
void test_jacob_Iteration(){
	double pointX,pointY,pointZ, angle;
	Eigen::MatrixXd TcpTarget;
	Eigen::Matrix<double,1,6> qq;	

	Eigen::Matrix<double,1,6> T0, T1, T2, T3, T4, T5;
	T0 << 0, 0, 0, 0, 0, 1;
	T1 << 98.5, 0, 0, 0, -1, 0;
	T2 << 98.5, 0, 408,0, -1, 0;
	T3 << 98.5, 0, 784,0, -1, 0;
	T4 << 121.5, -784, 0, 0, 0, -1;
	T5 << -4, 0,784, 0, -1, 0;

	Eigen::Matrix4d gz;
	gz(0,0)=1; gz(0,1)=0; gz(0,2)=0; gz(0,3)= -784;
	gz(1,0)=0; gz(1,1)=0; gz(1,2)=-1; gz(1,3)= -215.5;
	gz(2,0)=0; gz(2,1)=1; gz(2,2)=0; gz(2,3)= -4;
	gz(3,0)=0; gz(3,1)=0; gz(3,2)=0; gz(3,3)=1;
	Eigen::Matrix<double,1,6> theta;
	Eigen::Matrix<double,6,6> T;
	double q0,q1,q2,q3,q4,q5;
/*
	printf("Testing jacob_Iteration Inverse Kinematics......\n");
	printf("Please input desired position x y z:\n");
	scanf("%lf%lf%lf",&pointX,&pointY,&pointZ);
	printf("Please input desired pos angle:\n");
	scanf("%lf",&angle);
	angle=angle*M_PI/180.0;
*/
	printf("Please input 6 joint angle:\n");
	scanf("%lf%lf%lf%lf%lf%lf",&q0,&q1,&q2,&q3,&q4,&q5);
	q0 = q0*M_PI/180.0;
	q1 = q1*M_PI/180.0;
	q2 = q2*M_PI/180.0;
	q3 = q3*M_PI/180.0;
	q4 = q4*M_PI/180.0;
	q5 = q5*M_PI/180.0;
			
	printf("%lf %lf %lf %lf %lf %lf\n", q0,q1,q2,q3,q4,q5);
			
	TcpTarget=ForwardKinematics(q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5);
			
	//draw tcp marker point
	points.action = visualization_msgs::Marker::ADD;
	points.lifetime=ros::Duration();			
	points.scale.x = 0.01f;
	points.scale.y = 0.01f;
	points.scale.z = 0.01f;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	
	pointX = TcpTarget(0,3);
	pointY = TcpTarget(1,3);
	pointZ = TcpTarget(2,3);

	DisplayPoint(pointX,pointY,pointZ);

	//TcpTarget = EigenTransl(angle,pointX,pointY,pointZ);
	//TcpTarget(0,3) = TcpTarget(0,3) *1000;
	//TcpTarget(1,3) = TcpTarget(1,3) *1000;
	//TcpTarget(2,3) = TcpTarget(2,3) *1000;	
	std::cout<<"desired Tcp:\n"<<TcpTarget<<std::endl;	
	
	T.block<1,6>(0,0) = T0;
	T.block<1,6>(1,0) = T1;
	T.block<1,6>(2,0) = T2;
	T.block<1,6>(3,0) = T3;
	T.block<1,6>(4,0) = T4;
	T.block<1,6>(5,0) = T5;

	theta(0,0) = 0.0; theta(0,1)=0.0; theta(0,2)=0.0;theta(0,3)=0.0; theta(0,4)=0.0; theta(0,5)=0.0;
			
	printf("Begin jacob_Iteration!\n");

	//qq = Newton_Raphson(T,theta,gz,TcpTarget);
	qq = jacob_Iteration(TcpTarget,theta);
	std::cout<<"get desired rad:"<<qq<<std::endl;
	std::cout<<"forward matrix:\n"<<Exp_twist(T0,qq(0,0))*Exp_twist(T1,qq(0,1))*Exp_twist(T2,qq(0,2))*Exp_twist(T3,qq(0,3))*Exp_twist(T4,qq(0,4))*Exp_twist(T5,qq(0,5))*gz<<std::endl;
	std::cout<<"last get matrix:\n"<<ForwardKinematics(limit2pi(qq(0,0)),limit2pi(qq(0,1)), limit2pi(qq(0,2)),limit2pi(qq(0,3)),limit2pi(qq(0,4)),limit2pi(qq(0,5)))<<std::endl;
	
	printf("And then Moving to target point by inverse computing!\n");
	moveJ(limit2pi(qq(0,0)),limit2pi(qq(0,1))+M_PI/2,  -limit2pi(qq(0,2)),limit2pi(qq(0,3))+M_PI/2 ,limit2pi(qq(0,4)),limit2pi(qq(0,5)));

}

//test Inverse Kinematics of Newton_Raphson Iteration method
void test_Newton_Raphson(){
	double pointX,pointY,pointZ, angle;
	Eigen::MatrixXd TcpTarget;
	Eigen::Matrix<double,1,6> qq;	

	Eigen::Matrix<double,1,6> T0, T1, T2, T3, T4, T5;
	T0 << 0, 0, 0, 0, 0, 1;
	T1 << 98.5, 0, 0, 0, -1, 0;
	T2 << 98.5, 0, 408,0, -1, 0;
	T3 << 98.5, 0, 784,0, -1, 0;
	T4 << 121.5, -784, 0, 0, 0, -1;
	T5 << -4, 0,784, 0, -1, 0;

	Eigen::Matrix4d gz;
	gz(0,0)=1; gz(0,1)=0; gz(0,2)=0; gz(0,3)= -784;
	gz(1,0)=0; gz(1,1)=0; gz(1,2)=-1; gz(1,3)= -215.5;
	gz(2,0)=0; gz(2,1)=1; gz(2,2)=0; gz(2,3)= -4;
	gz(3,0)=0; gz(3,1)=0; gz(3,2)=0; gz(3,3)=1;
	Eigen::Matrix<double,1,6> theta;
	Eigen::Matrix<double,6,6> T;

	//printf("Testing Newton_raphson Inverse Kinematics......\n");
	//printf("Please input desired position x y z:\n");
	//scanf("%lf%lf%lf",&pointX,&pointY,&pointZ);
	//printf("Please input desired pos angle:\n");
	//scanf("%lf",&angle);
	//angle=angle*M_PI/180.0;
	//TcpTarget = EigenTransl(angle,pointX,pointY,pointZ);
	double q0,q1,q2,q3,q4,q5;

	printf("Please input 6 joint angle:\n");
	scanf("%lf%lf%lf%lf%lf%lf",&q0,&q1,&q2,&q3,&q4,&q5);
	q0 = q0*M_PI/180.0;
	q1 = q1*M_PI/180.0;
	q2 = q2*M_PI/180.0;
	q3 = q3*M_PI/180.0;
	q4 = q4*M_PI/180.0;
	q5 = q5*M_PI/180.0;
			
	printf("%lf %lf %lf %lf %lf %lf\n", q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5);
			
	TcpTarget=ForwardKinematics(q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5);
			
	//draw tcp marker point
	points.action = visualization_msgs::Marker::ADD;
	points.lifetime=ros::Duration();			
	points.scale.x = 0.01f;
	points.scale.y = 0.01f;
	points.scale.z = 0.01f;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	
	pointX = TcpTarget(0,3);
	pointY = TcpTarget(1,3);
	pointZ = TcpTarget(2,3);
	DisplayPoint(pointX,pointY,pointZ);

	
	TcpTarget(0,3) = TcpTarget(0,3) *1000;
	TcpTarget(1,3) = TcpTarget(1,3) *1000;
	TcpTarget(2,3) = TcpTarget(2,3) *1000;	
	std::cout<<"desired Tcp:\n"<<TcpTarget<<std::endl;	
	
	T.block<1,6>(0,0) = T0;
	T.block<1,6>(1,0) = T1;
	T.block<1,6>(2,0) = T2;
	T.block<1,6>(3,0) = T3;
	T.block<1,6>(4,0) = T4;
	T.block<1,6>(5,0) = T5;

	theta(0,0) = 0.0; theta(0,1)=0.0; theta(0,2)=0.0;theta(0,3)=0.0; theta(0,4)=0.0; theta(0,5)=0.0;
			
	printf("Begin newton_raphson!\n");

	qq = Newton_Raphson(T,theta,gz,TcpTarget);
	std::cout<<"get desired rad:"<<qq<<std::endl;
	std::cout<<"forward matrix:\n"<<Exp_twist(T0,qq(0,0))*Exp_twist(T1,qq(0,1))*Exp_twist(T2,qq(0,2))*Exp_twist(T3,qq(0,3))*Exp_twist(T4,qq(0,4))*Exp_twist(T5,qq(0,5))*gz<<std::endl;
	std::cout<<"last get matrix:\n"<<ForwardKinematics(limit2pi(qq(0,0)),limit2pi(qq(0,1)), limit2pi(qq(0,2)),limit2pi(qq(0,3)),limit2pi(qq(0,4)),limit2pi(qq(0,5)))<<std::endl;
	
	printf("And then Moving to target point by inverse computing!\n");
	moveJ(limit2pi(qq(0,0)),limit2pi(qq(0,1))+M_PI/2,  -limit2pi(qq(0,2)),limit2pi(qq(0,3))+M_PI/2 ,limit2pi(qq(0,4)),limit2pi(qq(0,5)));

}

	#define NUM_POINTS 7
	#define NUM_SEGMENTS (NUM_POINTS+1)

double* GetPoint(int i, double Points[7][3]){
	//return 1st point
	if(i<0){
		return Points[0];
	}
	//return last point
	if(i<NUM_POINTS)
		return Points[i];

	return Points[NUM_POINTS-1];
}

//trajectory planning B-spline
void testBspline(){
	//draw  marker point
	points.action = visualization_msgs::Marker::ADD;
	points.lifetime=ros::Duration();			
	points.scale.x = 0.005f;
	points.scale.y = 0.005f;
	points.scale.z = 0.01f;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	

	// the control points for the curve
	double Points[7][3] = {
		{-0.5,0.4,0.3},
		{-0.32,0.6,0.3},
		{0.2,0.45,0.3},
		{0.3,0.63,0.25},
		{0.34,0.56,0.3},
		{0.36,0.45,0.3},
		{0.40,0.51,0.27}
	};

	for(int j=0;j<NUM_POINTS;j++){
		std::cout<<Points[j][0]<<" "<<Points[j][1]<<" "<<Points[j][2]<<std::endl; 
		DisplayPoint( Points[j][0],Points[j][1],Points[j][2] );

	}
	//getchar();
	double qf[6] = {0, M_PI/2, 0, M_PI/2 , 0 ,0};//start point joint angle
	Eigen::Matrix<double,4,4> temPos;
	Eigen::Matrix3d poseT;
	Eigen::MatrixXd q= Eigen::Matrix<double,1,6>::Identity();

	// the level of detail of the curve
	unsigned int LOD=40;
	poseT = RPY2R(M_PI/4.0,M_PI,0);

	// start position
	temPos.block<3,3>(0,0) = poseT.block<3,3>(0,0);
	temPos(0,3) = Points[0][0];
	temPos(1,3) = Points[0][1];
	temPos(2,3) = Points[0][2];
	temPos(3,3) = 1;

	// use the parametric time value 0 to 1
	for(int start_cv=-3,m=0;m!=NUM_SEGMENTS;++m,++start_cv){
		for(int i=0;i!=LOD;++i) {

			double t = (double)i/(LOD-1);

			// the t value inverted
			double it = 1.0-t;

			// calculate blending functions
			double b0 = it*it*it/6.0;
			double b1 = (3*t*t*t - 6*t*t +4)/6.0;
			double b2 = (-3*t*t*t +3*t*t + 3*t + 1)/6.0;
			double b3 =  t*t*t/6.0;

			// sum the control points mulitplied by their respective blending functions
			double x = b0*GetPoint(start_cv+0,Points)[0] +
				  b1*GetPoint(start_cv+1,Points)[0] + 
				  b2*GetPoint(start_cv+2,Points)[0] + 
				  b3*GetPoint(start_cv+3,Points)[0] ;

			double y = b0*GetPoint(start_cv+0,Points)[1] + 
				  b1*GetPoint(start_cv+1,Points)[1] + 
				  b2*GetPoint(start_cv+2,Points)[1] + 
				  b3*GetPoint(start_cv+3,Points)[1] ;

			double z = b0*GetPoint(start_cv+0,Points)[2] + 
				  b1*GetPoint(start_cv+1,Points)[2] + 
				  b2*GetPoint(start_cv+2,Points)[2] + 
				  b3*GetPoint(start_cv+3,Points)[2] ;

			temPos(0,3) = x;
			temPos(1,3) = y;
			temPos(2,3) = z;

			q = getFirstIK(temPos,qf);
			moveJ(q(0),q(1)+M_PI/2.0,-q(2),q(3)+M_PI/2.0,q(4),q(5));
			// Display the point
			DisplayPoint( x,y,z );
		}
	}

	//specify the last point on the curve
	temPos(0,3) = Points[NUM_POINTS-1][0];
	temPos(1,3) = Points[NUM_POINTS-1][1];
	temPos(2,3) = Points[NUM_POINTS-1][2];

	q = getFirstIK(temPos,qf);
	moveJ(q(0),q(1)+M_PI/2.0,-q(2),q(3)+M_PI/2.0,q(4),q(5));
	// Display the last point
	DisplayPoint( temPos(0,3) ,temPos(1,3),temPos(2,3) );
}

//trajectory planning: CatmullRom
void testCatmullRom(){
	//draw  marker point
	points.action = visualization_msgs::Marker::ADD;
	points.lifetime=ros::Duration();			
	points.scale.x = 0.005f;
	points.scale.y = 0.005f;
	points.scale.z = 0.01f;
	points.color.r = 1.0f;
	points.color.g = 0.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	

	// the control points for the curve
	double Points[4][3] = {
		{-0.5,0.4,0.3},
		{-0.32,0.6,0.3},
		{0.2,0.45,0.3},
		{0.3,0.63,0.25},
	};
	
	for(int j=0;j<4;j++){
		std::cout<<Points[j][0]<<" "<<Points[j][1]<<" "<<Points[j][2]<<std::endl; 
		DisplayPoint( Points[j][0],Points[j][1],Points[j][2] );

	}
	
	getchar();
	getchar();
	double qf[6] = {0, M_PI/2, 0, M_PI/2 , 0 ,0};//reference point joint angle
	Eigen::Matrix<double,4,4> temPos;
	Eigen::Matrix3d poseT;
	Eigen::MatrixXd q= Eigen::Matrix<double,1,6>::Identity();

	// the level of detail of the curve
	unsigned int LOD=40;
	poseT = RPY2R(M_PI/4.0,M_PI,0);

	// start position
	temPos.block<3,3>(0,0) = poseT.block<3,3>(0,0);
	temPos(0,3) = Points[0][0];
	temPos(1,3) = Points[0][1];
	temPos(2,3) = Points[0][2];
	temPos(3,3) = 1;
	Eigen::Vector3d P0,P1,P2,P3;

	//the first segment curve
	P0<<2*Points[0][0]-Points[1][0],2*Points[0][1]-Points[0][1],2*Points[0][2]-Points[1][2];
	P1<<Points[0][0],Points[0][1],Points[0][2];
	P2<<Points[1][0],Points[1][1],Points[1][2];
	P3<<Points[2][0],Points[2][1],Points[2][2];

	double factor = 0.5;
	Eigen::Vector3d c0 = P1;
	Eigen::Vector3d c1 = (P2-P0)*factor;
	Eigen::Vector3d c2 = (P2-P1)*3.0-(P3-P1)*factor-(P2-P0)*2.0*factor;
	Eigen::Vector3d c3 = (P2-P1)*-2.0+(P3-P1)*factor+(P2-P0)*factor;

	Eigen::Vector3d curvePoint;

	for(int i=0;i!=LOD;++i) {
		double t = (double)i/LOD;
		//double it = 1.0-t;
		curvePoint = c3*t*t*t + c2*t*t + c1*t + c0;

		temPos(0,3) = curvePoint(0);
		temPos(1,3) = curvePoint(1);
		temPos(2,3) = curvePoint(2);

		q = getFirstIK(temPos,qf);
		moveJ(q(0),q(1)+M_PI/2.0,-q(2),q(3)+M_PI/2.0,q(4),q(5));		
		DisplayPoint( curvePoint(0),curvePoint(1),curvePoint(2));

	}	

	//the second segment curve
	P0<<Points[0][0],Points[0][1],Points[0][2];
	P1<<Points[1][0],Points[1][1],Points[1][2];
	P2<<Points[2][0],Points[2][1],Points[2][2];
	P3<<Points[3][0],Points[3][1],Points[3][2];

	factor = 0.5;
	c0 = P1;
	c1 = (P2-P0)*factor;
	c2 = (P2-P1)*3.0-(P3-P1)*factor-(P2-P0)*2.0*factor;
	c3 = (P2-P1)*-2.0+(P3-P1)*factor+(P2-P0)*factor;

	//Eigen::Vector3d curvePoint;

	for(int i=0;i!=LOD;++i) {
		double t = (double)i/LOD;
		//double it = 1.0-t;
		curvePoint = c3*t*t*t + c2*t*t + c1*t + c0;

		temPos(0,3) = curvePoint(0);
		temPos(1,3) = curvePoint(1);
		temPos(2,3) = curvePoint(2);

		q = getFirstIK(temPos,qf);
		moveJ(q(0),q(1)+M_PI/2.0,-q(2),q(3)+M_PI/2.0,q(4),q(5));		
		DisplayPoint( curvePoint(0),curvePoint(1),curvePoint(2));

	}	

	//the third segment curve

	P0<<Points[1][0],Points[1][1],Points[1][2];
	P1<<Points[2][0],Points[2][1],Points[2][2];
	P2<<Points[3][0],Points[3][1],Points[3][2];
	P3<<2*Points[3][0]-Points[2][0],2*Points[3][1]-Points[2][1],2*Points[3][2]-Points[2][2];

	factor = 0.3;
	c0 = P1;
	c1 = (P2-P0)*factor;
	c2 = (P2-P1)*3.0-(P3-P1)*factor-(P2-P0)*2.0*factor;
	c3 = (P2-P1)*-2.0+(P3-P1)*factor+(P2-P0)*factor;

	

	for(int i=0;i!=LOD;++i) {
		double t = (double)i/LOD;
		//double it = 1.0-t;
		curvePoint = c3*t*t*t + c2*t*t + c1*t + c0;
		temPos(0,3) = curvePoint(0);
		temPos(1,3) = curvePoint(1);
		temPos(2,3) = curvePoint(2);

		q = getFirstIK(temPos,qf);
		moveJ(q(0),q(1)+M_PI/2.0,-q(2),q(3)+M_PI/2.0,q(4),q(5));		
		DisplayPoint( curvePoint(0),curvePoint(1),curvePoint(2));

	}	
}

int main(int argc, char** argv) { 
	ros::init(argc, argv, "aubo_joint_publisher"); //node name 'aubo_joint_publisher'
	ros::NodeHandle n; 
	joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1); //setup joint_states publisher 
	marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);//setup Marker publisher 
	//initialize point marker 
	points.header.frame_id = "/base_link";
	points.header.stamp = ros::Time::now();
	points.ns = "aubo_joint_publisher";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.x = 0.0;
	points.pose.orientation.y = 0.0;
	points.pose.orientation.z = 0.0;
	points.pose.orientation.w = 1.0;

	//id->point markers
	points.id = 0;
  
	// marker type: POINTS
	points.type = visualization_msgs::Marker::POINTS;
 
	//POINTS marker scale
	points.scale.x = 0.005f;
	points.scale.y = 0.005f;
	points.scale.z = 0.005f;
	// green, alpha not zero
	points.color.r = 0.0f;
	points.color.g = 1.0f;
	points.color.b = 0.0f;
	points.color.a = 1.0;	    

	ros::Rate loop_rate(10); 
	
	int command,i,j;
	int num_sol;
	double *q_sol[8], q6_des;
	double j0,j1,j2,j3,j4,j5;
	double q0,q1,q2,q3,q4,q5;

	Eigen::Matrix<double,4,4> TcpCenter;

	char Menu[600]="\n0. Exit.\
			\n1. Test Foward Kinematics(MDH,Screw thoery,quaternion,dual quaternion).\
			\n2. Draw Monte carlo workspace.\
			\n3. Test Inverse Kinematics(Mixed Method).\
			\n4. Test Jacobian Inverse solution.\
			\n5. Test Newton Raphson Inverse solution.\
			\n6. TestCtraj MoveL(Cartesian trajectory).\
			\n7. TestJtraj moveC(Joint space trajectory).\
			\n8. Test Move point to point by Jtraj.\
			\n9. Test trajectory planning by B-spline.\
			\n10. Test trajectory planning by CatmullRom.\
			\n   Please input your command:";
	
	double pointX,pointY,pointZ;

	while (ros::ok()) { 
		printf("%s",Menu);
		
		scanf("%d",&command);
		switch(command){
		case 0: 
			exit(1);
			break;
		case 1: 
			printf("Please input 6 joint angle:\n");
			scanf("%lf%lf%lf%lf%lf%lf",&j0,&j1,&j2,&j3,&j4,&j5);
			q0 = j0*M_PI/180.0;
			q1 = j1*M_PI/180.0;
			q2 = j2*M_PI/180.0;
			q3 = j3*M_PI/180.0;
			q4 = j4*M_PI/180.0;
			q5 = j5*M_PI/180.0;
			
			printf("%lf %lf %lf %lf %lf %lf\n", q0,q1,q2,q3,q4,q5);
			//move joint in ROS rviz
			moveJ(q0,q1,q2,q3,q4,q5); 
			//Forward Kinematics: calculate TCP point POSE by MDH method
			TcpCenter=ForwardKinematics(q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5);
			std::cout<<"TCP POSE matrix:"<<std::endl;
			std::cout<<TcpCenter<<std::endl;

			//Forward Kinematics: calculate TCP point POSE by Screw thoery method (PoE formula)
			std::cout<<"Screw forward matrix:"<<std::endl;
			std::cout<<ForwardPoE(q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5)<<std::endl;

			//Forward Kinematics: calculate TCP point POSE by quaternion method
			std::cout<<"Quaternion forward matrix:"<<std::endl;
			std::cout<<ForwardQuat(q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5)<<std::endl;

			//Forward Kinematics: calculate TCP point POSE by dual quaternion method
			std::cout<<"Dual Quaternion forward matrix:"<<std::endl;
			std::cout<<ForwardDualQuat(q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5)<<std::endl;

			//draw tcpcenter point in rviz
			pointX=TcpCenter(0,3);
			pointY=TcpCenter(1,3);
			pointZ=TcpCenter(2,3);
			printf("%lf %lf %lf\n",pointX,pointY,pointZ);
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear
			marker_pub.publish(points);
			
			points.action = visualization_msgs::Marker::ADD;
			points.lifetime=ros::Duration();			
			points.scale.x = 0.01f;
			points.scale.y = 0.01f;
			points.scale.z = 0.01f;
			points.color.r = 1.0f;
			points.color.g = 0.0f;
			points.color.b = 0.0f;
			points.color.a = 1.0;	
			DisplayPoint(pointX,pointY,pointZ);
			

			break;
		case 2:
			printf("Wait for drawing Monte carlo workspace......\n");
			points.scale.x = 0.005f;
			points.scale.y = 0.005f;
			points.scale.z = 0.005f;
			points.color.r = 0;
			points.color.g = 1.0f;
			points.color.b = 0.0f;
			points.color.a = 1.0;	
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work
			points.points.clear(); //work to clear
			marker_pub.publish(points);
			
			points.action = visualization_msgs::Marker::ADD;	
			Monte_carlo_Workspace();
			printf("Finish drawing Monte carlo workspace......\n");
			break;
		case 3:
			printf("Testing Inverse Kinematics......\n");
			printf("Please input 6 joint angle:\n");
			scanf("%lf%lf%lf%lf%lf%lf",&j0,&j1,&j2,&j3,&j4,&j5);
			q0 = j0*M_PI/180.0;
			q1 = j1*M_PI/180.0;
			q2 = j2*M_PI/180.0;
			q3 = j3*M_PI/180.0;
			q4 = j4*M_PI/180.0;
			q5 = j5*M_PI/180.0;
			printf("Input to real angle:%lf %lf %lf %lf %lf %lf\n", j0,j1-90,-j2,j3-90,j4,j5);
			printf("RAD:%lf %lf %lf %lf %lf %lf\n", q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5);
			
			//calculate forward kinematics
			TcpCenter=ForwardKinematics(q0,q1-M_PI/2.0,-q2,q3-M_PI/2.0,q4,q5);
			
			pointX=TcpCenter(0,3);
			pointY=TcpCenter(1,3);
			pointZ=TcpCenter(2,3);
			//printf("%lf %lf %lf\n",pointX,pointY,pointZ);
			std:cout<<"desired matrix:\n"<<TcpCenter<<std::endl;
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear
			
			marker_pub.publish(points);
			
			//draw tcp marker point
			points.action = visualization_msgs::Marker::ADD;
			points.lifetime=ros::Duration();			
			points.scale.x = 0.01f;
			points.scale.y = 0.01f;
			points.scale.z = 0.01f;
			points.color.r = 1.0f;
			points.color.g = 0.0f;
			points.color.b = 0.0f;
			points.color.a = 1.0;	
			DisplayPoint(pointX,pointY,pointZ);
			
			printf("Now using inverse to moveJ!\n");
			
			q0 = q0;
			q1 = q1-M_PI/2.0;
			q2 = -q2;
			q3 = q3-M_PI/2.0;
			q4 = q4;
			q5 = q5;
			
			std::cout << "joint angle(-pi~+pi):"<<q0<<","<<q1<<","<<q2<<","<<q3<<","<<q4<<","<<q5<<std::endl;
			//calculate Inverse solution
			//make solution buffer
			for(i = 0; i < 8; i++)
			{
				q_sol[i] = new double[6];
			}		
			num_sol = InverseKinematics(ForwardKinematics(q0,q1,q2,q3,q4,q5),q_sol,q6_des);				
			std::cout << "solution num:"<<num_sol<<std::endl;
			if(num_sol == 0) std::cout << "Sorry,compute no solution! "<<std::endl;
			for(i=0;i<num_sol;i++){
				//printf("Moving to zero position first!\n");
				moveJ(0,0,0,0,0,0); 
				printf("And then Moving to target point by inverse computing!\n");

				moveJ(q_sol[i][0],q_sol[i][1]+M_PI/2.0,-q_sol[i][2],q_sol[i][3]+M_PI/2.0,q_sol[i][4],q_sol[i][5]);
				std::cout<<ForwardKinematics(q_sol[i][0],q_sol[i][1],q_sol[i][2],q_sol[i][3],q_sol[i][4],q_sol[i][5])<<std::endl;



				//moveJ(q_sol[i][0],q_sol[i][1],q_sol[i][2],q_sol[i][3],q_sol[i][4],q_sol[i][5]);
				//std::cout << q_sol[i][0]<<std::endl;
				std::cout << "Solution "<<i<<" :[ "<< RadtoAngle(q_sol[i][0]) <<","<< RadtoAngle(q_sol[i][1]) <<","<< RadtoAngle(q_sol[i][2])
				 <<","<< RadtoAngle(q_sol[i][3] ) <<","<< RadtoAngle(q_sol[i][4]) <<","<< RadtoAngle(q_sol[i][5]) << "]" << std::endl;			
				std::cout << "press any key to continue..."<<std::endl;
				getchar();

			}
									
			break;
		case 4:
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear			
			marker_pub.publish(points);
			test_jacob_Iteration();
			
			break;
		case 5:
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear			
			marker_pub.publish(points);
			test_Newton_Raphson();
			
			break;			
		case 6:
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear			
			marker_pub.publish(points);
			testCTraj();
			
			break;
		case 7:
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear			
			marker_pub.publish(points);
			testJTraj();
			
			break;
		case 8:
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear			
			marker_pub.publish(points);
			testMoveP();
			
			break;
		case 9:
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear
			
			marker_pub.publish(points);			
			
			testBspline();
			break;
		case 10:
			points.header.stamp = ros::Time::now();
			points.action = visualization_msgs::Marker::DELETE;//delete not work 
			points.points.clear();//work to clear
			
			marker_pub.publish(points);			
			
			testCatmullRom();
			break;

		default:
			printf("No this command!\n");
			
			break;
		}

		ros::spinOnce();
		// This will adjust as needed per iteration 
		loop_rate.sleep(); 
		
	} 

	return 0; 
}


