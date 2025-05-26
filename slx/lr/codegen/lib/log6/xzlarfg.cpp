//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzlarfg.cpp
//
// Code generation for function 'xzlarfg'
//

// Include files
#include "xzlarfg.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
double xzlarfg(int n, double &alpha1, double x[3])
{
  double tau;
  tau = 0.0;
  if (n > 0) {
    double xnorm;
    xnorm = blas::xnrm2(n - 1, x);
    if (xnorm != 0.0) {
      double a_tmp;
      a_tmp = std::abs(alpha1);
      xnorm = std::abs(xnorm);
      if (a_tmp < xnorm) {
        a_tmp /= xnorm;
        xnorm *= std::sqrt(a_tmp * a_tmp + 1.0);
      } else if (a_tmp > xnorm) {
        xnorm /= a_tmp;
        xnorm = a_tmp * std::sqrt(xnorm * xnorm + 1.0);
      } else if (std::isnan(xnorm)) {
        xnorm = rtNaN;
      } else {
        xnorm = a_tmp * 1.4142135623730951;
      }
      if (alpha1 >= 0.0) {
        xnorm = -xnorm;
      }
      if (std::abs(xnorm) < 1.0020841800044864E-292) {
        __m128d r;
        int knt;
        int vectorUB;
        int vectorUB_tmp;
        knt = 0;
        do {
          knt++;
          vectorUB = (((n - 1) / 2) << 1) + 2;
          vectorUB_tmp = vectorUB - 2;
          for (int k{2}; k <= vectorUB_tmp; k += 2) {
            r = _mm_loadu_pd(&x[k - 1]);
            _mm_storeu_pd(&x[k - 1],
                          _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
          }
          for (int k{vectorUB}; k <= n; k++) {
            x[k - 1] *= 9.9792015476736E+291;
          }
          xnorm *= 9.9792015476736E+291;
          alpha1 *= 9.9792015476736E+291;
        } while ((std::abs(xnorm) < 1.0020841800044864E-292) && (knt < 20));
        a_tmp = std::abs(alpha1);
        xnorm = std::abs(blas::xnrm2(n - 1, x));
        if (a_tmp < xnorm) {
          a_tmp /= xnorm;
          xnorm *= std::sqrt(a_tmp * a_tmp + 1.0);
        } else if (a_tmp > xnorm) {
          xnorm /= a_tmp;
          xnorm = a_tmp * std::sqrt(xnorm * xnorm + 1.0);
        } else if (std::isnan(xnorm)) {
          xnorm = rtNaN;
        } else {
          xnorm = a_tmp * 1.4142135623730951;
        }
        if (alpha1 >= 0.0) {
          xnorm = -xnorm;
        }
        tau = (xnorm - alpha1) / xnorm;
        a_tmp = 1.0 / (alpha1 - xnorm);
        for (int k{2}; k <= vectorUB_tmp; k += 2) {
          r = _mm_loadu_pd(&x[k - 1]);
          _mm_storeu_pd(&x[k - 1], _mm_mul_pd(_mm_set1_pd(a_tmp), r));
        }
        for (int k{vectorUB}; k <= n; k++) {
          x[k - 1] *= a_tmp;
        }
        for (int k{0}; k < knt; k++) {
          xnorm *= 1.0020841800044864E-292;
        }
        alpha1 = xnorm;
      } else {
        int knt;
        int vectorUB;
        tau = (xnorm - alpha1) / xnorm;
        a_tmp = 1.0 / (alpha1 - xnorm);
        knt = (((n - 1) / 2) << 1) + 2;
        vectorUB = knt - 2;
        for (int k{2}; k <= vectorUB; k += 2) {
          __m128d r;
          r = _mm_loadu_pd(&x[k - 1]);
          _mm_storeu_pd(&x[k - 1], _mm_mul_pd(_mm_set1_pd(a_tmp), r));
        }
        for (int k{knt}; k <= n; k++) {
          x[k - 1] *= a_tmp;
        }
        alpha1 = xnorm;
      }
    }
  }
  return tau;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzlarfg.cpp)
