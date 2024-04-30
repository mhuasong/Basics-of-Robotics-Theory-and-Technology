#ifndef _INSTRUCTIONSET_H
#define _INSTRUCTIONSET_H

#define WORDNUM 16	//指令集分词库的总数
#define FALSE 0
#define MOVPNUM 15	//
#define BEGINNUM 15	//
#define MOVLNUM 15	//
#define MOVJNUM 15	//

//指令集词库，对应表7.11
typedef enum
{
	ENDFILE,ENDLINE,INVALID,
	INIT,BEGIN,MOVEP,MOVEL,MOVEJ,END,B,I,P,J,X,Y,Z,DEFN,DEFX,
	ID,NUM
}Tokentype;

//词法分析FA转换图，5种状态，对应图7.14
typedef enum
{
	START,INID,INNUM,DONE,INPOINT
}StateType;

//定义一个点三维坐标的结构体
typedef struct
{
	double x;
	double y;
	double z;
}Point;

//指令类别
typedef struct
{
	unsigned char ucType;
	char szValue[16];
}STRU_CONST_VAR;

//字节数据
typedef struct 
{
	int bn;
	char Bvar[100];	
}B_CONST_VAR;

//整型数据
typedef struct
{
	int in;
	int Ivar[100];
}I_CONST_VAR;

//关节角
typedef struct
{
	double j1;
	double j2;
	double j3;
	double j4;
	double j5;
	double j6;
}Joint;

#endif
