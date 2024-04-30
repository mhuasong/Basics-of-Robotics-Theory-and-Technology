#ifndef _ERROR_H_
#define _ERROR_H_
#include<string.h>
#include"InstructionSet.h"

typedef enum
{
	ERR_EMPTY,  //�ղ���
	ERR_ZERO,      //����
	ERR_COMDATA,   //���������﷨��?
	ERR_COMLBL,	//δ��������/��ǩ
	ERR_NOCHAR,//"û���ַ�"
	ERR_NOINIT,	//�ļ�û��NOP����END
	ERR_VALUE,		//��ֵ����
	ERR_NOINITORBEGIN
}ERROR_EM;	

typedef struct 
{
	ERROR_EM ErrNo;
	int linenum;
}ElemType;

#endif
