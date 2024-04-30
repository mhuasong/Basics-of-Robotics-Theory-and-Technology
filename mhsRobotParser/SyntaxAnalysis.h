#ifndef _SYNTAXANLYSIS_H
#define _SYNTAXANLYSIS_H
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <cmath>
#include <Eigen/Eigen>
#include "InstructionSet.h"
#include "errorinfo.h"

#define PI  M_PI

int errInforDisplay(ERROR_EM a,int linenum);
int SyAnalyze(Tokentype CurToken);
Eigen::Matrix4d init_analyze(void);
Eigen::Matrix4d Begin_SyAnalyze();
int End_SyAnalyze(void);
int defn_analyze();
int defx_analyze();
Eigen::Matrix4d movej_analyze();
Eigen::Matrix4d movel_analyze(void);
Eigen::Matrix4d movep_analyze(void);

#endif
