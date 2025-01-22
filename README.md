# Basics-of-Robotics-Theory-and-Technology
Source code and slides for the textbook written by Huasong Min: The Basics of Robotics Theory and Technology
#
教材《机器人理论与技术基础》章节实验代码、课堂讲义及说明
目录说明：
ch2:
	ch2_1.cpp: 教材第二章位形表示法之间的转换，包括RPY法、旋量法、四元数法、Rodrigues参数法。
	ex2_7: Aubo协作机器人在ROS下的建模。
ch3:
	delta_robot: 并联机器人Delta运动学实验。
	differential_wmr: 带小脚轮的两轮差分驱动移动机器人运动学参考实验。
	kinematics: 第3章的6轴机械臂的运动学正逆解函数代码以及第5章运动规划中机械臂相关的轨迹规划算法函数库。运动学相关算法代码在kinematics.cpp中，正运动学算法包括:SDH、MDH、旋量法、四元数法、对偶四元数法，逆运动学算法包括解析法（MDH）、几何法与解析法混合法、雅可比迭代、牛顿-拉夫森迭代；轨迹规划算法只包含了简单：直线插补（操作空间（末端坐标系）、关节空间（关节坐标系））、关节空间多项式插补算法（Quintic polynomial,Cubic polynomial,b-spline,CatmullRom)。kineSim.cpp为测试主程序。
	mecanum_robot: 四轮全向移动全向移动机器人运动学实验。
	omni_robot: 三轮全向移动机器人运动学实验。
	test: Aubo协作机械臂运动学仿真测试实验。
ch4: 
	第四章机器人正逆动力学实验(RNEA、ABA算法）。
ch5:
	运动规划的算法网络开源代码推荐，Atsushi Sakai等人的开源python代码在我的github下也有fork。
mhsRobotParser:
	教材第7章机器人编程语言解析器设计的开源代码示范。
ppt:
	本人编写的教材讲义初稿。
