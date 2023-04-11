#include<iostream> 
#include<string>
#include<ros/ros.h>
#include<sensor_msgs/JointState.h>
#include<robot_state_publisher/robot_state_publisher.h> 

using namespace std; 

sensor_msgs::JointState joint_state; 
ros::Publisher joint_pub;

int moveJ(double j1,double j2,double j3,double j4,double j5,double j6);

int main(int argc, char** argv) { 
	ros::init(argc, argv, "aubo_joint_publisher");  
	ros::NodeHandle n; 
	joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);   
	ros::Rate loop_rate(10);  
	int command;
	char Menu[256]="\n0. Exit.\n1. Control joints to move.\nPlease input your command:";
	double j1,j2,j3,j4,j5,j6;
	double arcAng1,arcAng2,arcAng3,arcAng4,arcAng5,arcAng6;
	while (ros::ok()) { 
		printf("%s",Menu);
		
		scanf("%d",&command);
		switch(command){
		case 0: 
			exit(1);
			break;
		case 1: 
			printf("Please input 6 joint angle:\n");
			scanf("%lf%lf%lf%lf%lf%lf",&j1,&j2,&j3,&j4,&j5,&j6);
			arcAng1=j1*M_PI/180.0;
			arcAng2=j2*M_PI/180.0;
			arcAng3=j3*M_PI/180.0;
			arcAng4=j4*M_PI/180.0;
			arcAng5=j5*M_PI/180.0;
			arcAng6=j6*M_PI/180.0;
			printf("%lf %lf %lf %lf %lf %lf\n", arcAng1,arcAng2,arcAng3,arcAng4,arcAng5,arcAng6);
			moveJ(arcAng1,arcAng2,arcAng3,arcAng4,arcAng5,arcAng6); 
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

	//send the joint state and transform 
	joint_pub.publish(joint_state); 

	return 0;
}
