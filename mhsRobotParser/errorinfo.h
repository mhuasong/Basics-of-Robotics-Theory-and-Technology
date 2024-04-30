#ifndef _ERROR_H_
#define _ERROR_H_
#include<string.h>
#include"InstructionSet.h"

typedef enum
{
	ERR_EMPTY,  //空参数
	ERR_ZERO,      //空行
	ERR_COMDATA,   //命令数据语法错?
	ERR_COMLBL,	//未定义命令/标签
	ERR_NOCHAR,//"没有字符"
	ERR_NOINIT,	//文件没有NOP或者END
	ERR_VALUE,		//数值错误
	ERR_NOINITORBEGIN
}ERROR_EM;	

typedef struct 
{
	ERROR_EM ErrNo;
	int linenum;
}ElemType;

#endif
