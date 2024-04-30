#include "getToken.h"
#include "SyntaxAnalysis.h"
#include "errorinfo.h"

using namespace std;
extern char tokenString[16];
extern long nCurPos;
extern int lineno;
int endlable=0;

Eigen::Matrix4d curpos; 
Eigen::VectorXd curangle(6);
Eigen::Matrix4d CompileMatrix;
Eigen::Vector3d pos0;
Eigen::VectorXd angle0(6);
B_CONST_VAR bv;
I_CONST_VAR iv;
Point pv[100];
Joint jv[100];

int errInforDisplay(ERROR_EM a,int linenum)
{
	ElemType et;
	et.ErrNo=a;
	et.linenum=linenum+1;
	switch(et.ErrNo)
	{
			case ERR_EMPTY:
				cout<<"There is a warning in line  "<<et.linenum<<" : "<<"空输入"<<endl;
			break;
			case ERR_ZERO:
				cout<<"There is a warning in line  "<<et.linenum<<" : "<<"空行"<<endl;
			break;
			case ERR_COMDATA:
				cout<<"There is an error in line  "<<et.linenum<<" : "<<"数据语法错误"<<endl;
			break;
			case ERR_COMLBL:
				cout<<"There is an error in line  "<<et.linenum<<" : "<<"未定义命令"<<endl;
			break;
			case ERR_NOCHAR:
				cout<<"There is an error in line  "<<et.linenum<<" : "<<"无字符"<<endl;
			break;
			case ERR_NOINIT:
				cout<<"There is an error in line  "<<et.linenum<<" : "<<"缺失init"<<endl;
			break;
			case ERR_VALUE:
				cout<<"There is an error in line  "<<et.linenum<<" : "<<"数值错误或无效数值"<<endl;
			break;
			case ERR_NOINITORBEGIN:
				cout<<"There is an error in line  "<<et.linenum<<" : "<<"缺失init或begin"<<endl;
			break;
	}
	return 1;
}

int SyAnalyze(Tokentype CurToken)
{
	unsigned char ttype;

	ttype=CurToken;

	if(ttype==ENDFILE)
	{
		return false;
	}
	switch(ttype)
	{	case INIT:
			cout<<"init"<<endl;
			curpos=init_analyze();
			break;
		case BEGIN:
			cout<<"begin"<<endl;
			curpos=Begin_SyAnalyze();
			//curangle=ikine(curpos);
			break;
		case END:
			cout<<"end"<<endl;
			endlable=End_SyAnalyze();
			break;
		case ENDLINE:
			cout<<"endline"<<endl;
			errInforDisplay(ERR_ZERO,lineno);
			getchar();
			break;
		case NUM:
			cout<<"num"<<endl;
			errInforDisplay(ERR_VALUE,lineno);
			getchar();
			break;
		case DEFN:
			cout<<"defn"<<endl;
			defn_analyze();
			break;
		case DEFX:
			cout<<"defx"<<endl;
			defx_analyze();
			break;
		case MOVEP:
			cout<<"movep"<<endl;
			curpos=movep_analyze();
			break;
		case MOVEL:
			cout<<"movel"<<endl;
			curpos=movel_analyze();
			break;
		case MOVEJ:
			cout<<"movej"<<endl;
			curpos=movej_analyze();
			break;
		default:			
			errInforDisplay(ERR_COMLBL,lineno);
			getchar();
			break;
	}
	return ttype;	
}

Eigen::Matrix4d init_analyze()
{
	cout<<"please move manipulator to initial pose."<<endl;
	Eigen::Matrix4d initpos;
	//具体如何初始化及控制机械臂运动到初始位姿,这里我们省略了
	if(ENDLINE!=lex2token())
	{
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
	}
	
	return initpos;
}

Eigen::Matrix4d Begin_SyAnalyze()
{
	Eigen::Matrix4d beginpos;
	//具体begin语句要做啥事没有写
	
	if(ENDLINE!=lex2token())
	{
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
	}
	
	return beginpos;
}

int End_SyAnalyze()
{
	Tokentype over;
	over=lex2token();

	if(over==ENDLINE)
	{
		cout<<"解析结束"<<endl;
		return 1;
	}
	else 
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
		return 0;
}

int defn_analyze()
{
	Tokentype defn_token;
	defn_token=lex2token();
	int Ict;
	int Bct;

	switch(defn_token)
	{
	case B:
		lex2token();
		Bct=atoi(tokenString);
		lex2token();
		bv.Bvar[Bct]=tokenString[0];
		//cout<<"B0  B1  B2="<<";"<<bv.Bvar[0]<<";"<<bv.Bvar[1]<<";"<<bv.Bvar[2]<<endl;
		//cout<<"I0  I1  I2="<<";"<<iv.Ivar[0]<<";"<<iv.Ivar[1]<<";"<<iv.Ivar[2]<<endl;
		break;
	case I:
		lex2token();
		Ict=atoi(tokenString);
		lex2token();
		iv.Ivar[Ict]=atoi(tokenString);
		//cout<<"B0  B1  B2="<<";"<<bv.Bvar[0]<<";"<<bv.Bvar[1]<<";"<<bv.Bvar[2]<<endl;
		//cout<<"I0  I1  I2="<<";"<<iv.Ivar[0]<<";"<<iv.Ivar[1]<<";"<<iv.Ivar[2]<<endl;
		break;
	default:
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
		break;
	}

	cout<<"B0  B1  B2="<<";"<<bv.Bvar[0]<<";"<<bv.Bvar[1]<<";"<<bv.Bvar[2]<<endl;
	cout<<"I0  I1  I2="<<";"<<iv.Ivar[0]<<";"<<iv.Ivar[1]<<";"<<iv.Ivar[2]<<endl;
	
	return 0;
}

int defx_analyze()
{
	Tokentype defx_token;
	defx_token=lex2token();
	int Pct;
	int Jct;

	switch(defx_token)
	{
	case P:
		lex2token();
		Pct=atoi(tokenString);
		lex2token();
		pv[Pct].x=atoi(tokenString);
		lex2token();
		pv[Pct].y=atoi(tokenString);
		lex2token();
		pv[Pct].z=atoi(tokenString);
		
		break;
	case J:
		lex2token();
		Jct=atoi(tokenString);
		lex2token();
		jv[Jct].j1=atoi(tokenString);
		lex2token();
		jv[Jct].j2=atoi(tokenString);
		lex2token();
		jv[Jct].j3=atoi(tokenString);
		lex2token();
		jv[Jct].j4=atoi(tokenString);
		lex2token();
		jv[Jct].j5=atoi(tokenString);
		lex2token();
		jv[Jct].j6=atoi(tokenString);

		break;
	default:
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
		break;
	}

	cout<<"P0 = "<<pv[0].x<<";"<<pv[0].y<<";"<<pv[0].z<<endl;
	cout<<"P1 = "<<pv[1].x<<";"<<pv[1].y<<";"<<pv[1].z<<endl;
	cout<<"J0 = "<<jv[0].j1<<";"<<jv[0].j2<<";"<<jv[0].j3<<";"<<jv[0].j4<<";"<<jv[0].j5<<";"<<jv[0].j6<<endl;
	return 0;
}

Eigen::Matrix4d movep_analyze()
{
	Tokentype movep_token=lex2token();
	Point movep_point;
	Eigen::Vector3d moveppos;
	Eigen::VectorXd movepangle(6);

	switch(movep_token)
	{
	case NUM:
		movep_point.x=atoi(tokenString);
		cout<<"x="<<movep_point.x<<endl;
		if(NUM!=lex2token())
		{
		errInforDisplay(ERR_VALUE,lineno);
		getchar();
		exit(1);
		}
		movep_point.y=atoi(tokenString);
		cout<<"y="<<movep_point.y<<endl;
		if(NUM!=lex2token())
		{
		errInforDisplay(ERR_VALUE,lineno);
		getchar();
		exit(1);
		}
		movep_point.z=atoi(tokenString);
		cout<<"z="<<movep_point.z<<endl;
		break;
	case P:
		lex2token();
		movep_point.x=pv[atoi(tokenString)].x;
		movep_point.y=pv[atoi(tokenString)].y;
		movep_point.z=pv[atoi(tokenString)].z;
		cout<<"x="<<movep_point.x<<endl;
		cout<<"y="<<movep_point.y<<endl;
		cout<<"z="<<movep_point.z<<endl;
		break;
	default:
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
		break;
	}
	
	moveppos(0)=movep_point.x;
	moveppos(1)=movep_point.y;
	moveppos(2)=movep_point.z;
	//compute movepangle through inverse kinematics ,for example: ikine(moveppos);
	//planning CompileMatrix through trajectory planning function,for example:jtraj(curangle,movepangle);
	return CompileMatrix;
}

Eigen::Matrix4d movel_analyze()
{
	Tokentype movel_token=lex2token();
	Point movel_point;
	Eigen::Vector3d movelpos;
	Eigen::VectorXd movelangle(6);

	switch(movel_token)
	{
	case NUM:
		movel_point.x=atoi(tokenString);
		cout<<"x="<<movel_point.x<<endl;
		if(NUM!=lex2token())
		{
		errInforDisplay(ERR_VALUE,lineno);
		getchar();
		exit(1);
		}
		movel_point.y=atoi(tokenString);
		cout<<"y="<<movel_point.y<<endl;
		if(NUM!=lex2token())
		{
		errInforDisplay(ERR_VALUE,lineno);
		getchar();
		exit(1);
		}
		movel_point.z=atoi(tokenString);
		cout<<"z="<<movel_point.z<<endl;
		break;
	case P:
		lex2token();
		movel_point.x=pv[atoi(tokenString)].x;
		movel_point.y=pv[atoi(tokenString)].y;
		movel_point.z=pv[atoi(tokenString)].z;
		cout<<"x="<<movel_point.x<<endl;
		cout<<"y="<<movel_point.y<<endl;
		cout<<"z="<<movel_point.z<<endl;
		break;
	default:
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
		break;
	}
	
	movelpos(0)=movel_point.x;
	movelpos(1)=movel_point.y;
	movelpos(2)=movel_point.z;
	//planning CompileMatrix through trajectory planning function,for example: ctraj(curpos,movelpos);
	return CompileMatrix;
}

Eigen::Matrix4d movej_analyze()
{
	Tokentype movej_token=lex2token();
	Eigen::Vector3d movejpos;
	Eigen::VectorXd movejangle(6);

	int axisnum;
	int Jcount;
	double roundangle0;
	double roundangle1;
	switch(movej_token)
	{
	case NUM:
		axisnum=atoi(tokenString);
		if(NUM!=lex2token())
		{
		errInforDisplay(ERR_VALUE,lineno);
		getchar();
		exit(1);
		}
		roundangle0=atoi(tokenString);
		roundangle1=roundangle0*PI/180;
		cout<<"axis** "<<axisnum<<endl<<"roundangle** "<<roundangle0<<endl;
		movejangle=curangle;
		movejangle(0,axisnum-1)=movejangle(0,axisnum-1)+roundangle1;
		//convert CompileMatrix from movejangle;
		curangle=movejangle;
		//compute end-effect position through forward kinematics for example: movejpos=fkine(movejangle);
		break;
	case J:
		lex2token();
		Jcount=atoi(tokenString);
		movejangle=curangle;
		movejangle(0)=(jv[Jcount].j1)*PI/180;
		movejangle(1)=(jv[Jcount].j2)*PI/180;
		movejangle(2)=(jv[Jcount].j3)*PI/180;
		movejangle(3)=(jv[Jcount].j4)*PI/180;
		movejangle(4)=(jv[Jcount].j5)*PI/180;
		movejangle(5)=(jv[Jcount].j6)*PI/180;
		//convert CompileMatrix from movejangle;
		curangle=movejangle;
		//compute end-effect position through forward kinematics for example: movejpos=fkine(movejangle);
		break;
	default:
		errInforDisplay(ERR_COMDATA,lineno);
		getchar();
		exit(1);
		break;
	}
	cout<<" six angles is: "<<movejangle(0)<<";"<<movejangle(1)<<";"<<movejangle(2)<<";"<<movejangle(3)<<";"<<movejangle(4)<<";"<<movejangle(5)<<";"<<endl;
	//planning CompileMatrix through trajectory planning function,for example: jtraj(curpos,movelpos);
	return CompileMatrix;
}


