//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sylvesterRecursive.h
//
// Code generation for function 'sylvesterRecursive'
//

#ifndef SYLVESTERRECURSIVE_H
#define SYLVESTERRECURSIVE_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
void sylvesterRecursive(int ia0, int ja0, int ib0, int jb0, creal_T C[16],
                        int ic0, int jc0, int m, int n);

void sylvesterRecursive(int ia0, int ja0, int ib0, int jb0, double C[16],
                        int ic0, int jc0, int m, int n);

} // namespace coder

#endif
// End of code generation (sylvesterRecursive.h)
