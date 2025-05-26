//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// log6.cpp
//
// Code generation for function 'log6'
//

// Include files
#include "log6.h"
#include "logm.h"
#include "rt_nonfinite.h"

// Function Definitions
void log6(const double T[16], creal_T lambda[6])
{
  creal_T se3mat[16];
  coder::logm(T, se3mat);
  lambda[0] = se3mat[12];
  lambda[1] = se3mat[13];
  lambda[2] = se3mat[14];
  lambda[3] = se3mat[6];
  lambda[4] = se3mat[8];
  lambda[5] = se3mat[1];
}

// End of code generation (log6.cpp)
