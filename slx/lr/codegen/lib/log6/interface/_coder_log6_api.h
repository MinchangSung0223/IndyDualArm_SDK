//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_log6_api.h
//
// Code generation for function 'log6'
//

#ifndef _CODER_LOG6_API_H
#define _CODER_LOG6_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void log6(real_T T[16], creal_T lambda[6]);

void log6_api(const mxArray *prhs, const mxArray **plhs);

void log6_atexit();

void log6_initialize();

void log6_terminate();

void log6_xil_shutdown();

void log6_xil_terminate();

#endif
// End of code generation (_coder_log6_api.h)
