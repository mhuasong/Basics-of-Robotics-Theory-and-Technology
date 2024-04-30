#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <iostream>

#include "getToken.h"

using namespace std;
char lineBuf[257];
char tokenString[16];
static int linepos = 0;
extern FILE *file;
extern long nCurPos;
int lineno=-1;
static struct
{
	const char *str;
	Tokentype tok;
} Words[WORDNUM]
={
	{"BEGIN",BEGIN},{"NUM",NUM},{"INIT",INIT},
	{"MOVEP",MOVEP},{"MOVEL",MOVEL},{"MOVEJ",MOVEJ},
	{"B",B},{"X",X},{"Y",Y},{"Z",Z},{"I",I},{"P",P},{"J",J},{"DEFN",DEFN},{"DEFX",DEFX},
	{"END",END}
};

int readOneLine()
{
	int bufsize=0;
	
	if(fgets(lineBuf,80,file))
	{
		bufsize=strlen(lineBuf);
		linepos=0;
		lineno++;
		cout<<"Line No: "<<lineno<<endl;
		//cout<<"Bufsize = "<<bufsize<<endl;
		printf("%s",lineBuf);
	}
	else
	{
		bufsize=-1;
	}
	return bufsize;
}

static int getNextChar(void)
{
	int bufsize=0;
	bufsize=strlen(lineBuf);
	if(linepos<bufsize+1)
	{

		return lineBuf[linepos++];
		
	}
	else
		return lineBuf[linepos];
		
}

static Tokentype Wordslookup(char *s)
{
	int i;
	for(i=0;i<WORDNUM;i++)
	{
		if(!strcmp(s,Words[i].str))
		{
			return Words[i].tok;
		}
	}
	return INVALID;
}

void ungetNextChar(void)
{
		linepos-- ;
}

Tokentype lex2token(void)
{
	Tokentype currentToken = INVALID;
	int tokenStringIndex=0; 
	StateType state = START;
	int save = 0;
	char c = 0;
	while(state!=DONE)
	{
		c=getNextChar();
		save=1;
		switch(state)
		{
		case START:
			if(isalpha(c))
			{
				state=INID;
			}
			else if(isdigit(c)||(c=='-'))
			{
				state=INNUM;
			}
			else if(c==' ')
			{
				save=0;
			}
			else if(c=='/')
			{
				save=0;
			}
			else
			{
				state=DONE;
				switch(c)
				{
				case EOF:
					save=FALSE;
					currentToken=ENDFILE;
					break;
				case '\n':
				case '\r':
					save=0;
					currentToken=ENDLINE;
					break;
				case'*':
				case':':
					break;
				}
			}
			break;
		case INID:
			if (!isalpha(c) )
			{		
				
				ungetNextChar();
				save = FALSE;
				state = DONE;
				currentToken = ID;
			}
			break;
		case INNUM:
			if(isdigit(c))
			{state=INNUM;}
			else if(c=='.')
			{
				state=INPOINT;
			}
			else {
				ungetNextChar();
				save = 0;
				state = DONE;
				currentToken = NUM;
			}
			break;
		case INPOINT:
			if(!isdigit(c))
			{
				ungetNextChar();
				save=0;
				state=DONE;
				currentToken=NUM;
			}
			break;
		case DONE:
			break;
		}
		if ((save) && (tokenStringIndex <= 20)) {
			tokenString[tokenStringIndex] = c;
			tokenStringIndex++;
		}
		if (state == DONE) {
			tokenString[tokenStringIndex] = '\0';
			if (currentToken == ID) {
				currentToken = Wordslookup(tokenString);
			}
			else{
					return currentToken;
				}
		}
	}
	return currentToken;
}

