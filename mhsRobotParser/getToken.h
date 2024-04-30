#ifndef _GETTOKEN_H_
#define _GETTOKEN_
#include<stdio.h>
#include "InstructionSet.h"

int readOneLine();
static int getNextChar(void);
static Tokentype Wordslookup(char*s);
Tokentype lex2token(void) ;
void ungetNextChar(void);


#endif

