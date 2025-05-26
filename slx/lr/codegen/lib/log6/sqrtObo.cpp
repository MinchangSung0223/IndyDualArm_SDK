//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sqrtObo.cpp
//
// Code generation for function 'sqrtObo'
//

// Include files
#include "sqrtObo.h"
#include "log6_rtwutil.h"
#include "rt_nonfinite.h"
#include <cmath>

// Function Definitions
namespace coder {
double sqrtObo(double a, int s)
{
  double val;
  if (s == 0) {
    val = a - 1.0;
  } else {
    double r;
    int n0;
    n0 = s;
    if (rt_atan2d_snf(0.0, a) >= 1.5707963267948966) {
      a = std::sqrt(a);
      n0 = s - 1;
    }
    val = a - 1.0;
    a = std::sqrt(a);
    r = a + 1.0;
    n0 -= 2;
    for (int i{0}; i <= n0; i++) {
      a = std::sqrt(a);
      r *= a + 1.0;
    }
    val /= r;
  }
  return val;
}

} // namespace coder

// End of code generation (sqrtObo.cpp)
