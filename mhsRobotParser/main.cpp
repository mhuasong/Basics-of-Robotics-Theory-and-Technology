#include<stdlib.h>
#include<stdio.h>
#include<fstream>
#include<iostream>
#include "getToken.h"
#include "SyntaxAnalysis.h"

using namespace std;
FILE* file;
unsigned char ttype;
Tokentype CurToken;

int main(int argc, char** argv) 
{

	file = fopen("test.srl","r");
	if(file == NULL)
	{
		cout<<"Open file error!"<<endl;
		exit(-1);
	}
	cout<<"Please press return key to read the robot program:"<<endl;
	getchar();
	while(readOneLine()!= -1)
	{
		//cout<<"Read one line!"<<endl;

		CurToken = lex2token();
		ttype=CurToken;
		cout<<"ttype = "<<static_cast<int>(ttype)<<endl;
		if(ttype==ENDFILE)
			{
				return false;
			}
		ttype=SyAnalyze(CurToken);
		cout<<"---------"<<endl;
	}
	cout<<"Got it!"<<endl;
	fclose(file);
	return 0;
}


