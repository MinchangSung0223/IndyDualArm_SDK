//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// insertionsort.cpp
//
// Code generation for function 'insertionsort'
//

// Include files
#include "insertionsort.h"
#include "rt_nonfinite.h"

// Function Definitions
namespace coder {
namespace internal {
void insertionsort(double x[2])
{
  double xc;
  int idx;
  xc = x[1];
  idx = 1;
  if (xc < x[0]) {
    x[1] = x[0];
    idx = 0;
  }
  x[idx] = xc;
}

} // namespace internal
} // namespace coder

// End of code generation (insertionsort.cpp)
