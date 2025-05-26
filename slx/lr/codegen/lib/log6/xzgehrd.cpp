//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xzgehrd.cpp
//
// Code generation for function 'xzgehrd'
//

// Include files
#include "xzgehrd.h"
#include "rt_nonfinite.h"
#include "xnrm2.h"
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
void xzgehrd(double a[4], int ilo, int ihi)
{
  double work[2];
  if ((ihi - ilo) + 1 > 1) {
    work[0] = 0.0;
    work[1] = 0.0;
    for (int i{ilo}; i < ihi; i++) {
      double alpha1;
      double beta1;
      double d;
      double tau;
      int exitg1;
      int ia;
      int knt;
      int lastc;
      int lastv;
      alpha1 = a[1];
      tau = 0.0;
      if (ihi - 1 > 0) {
        beta1 = blas::b_xnrm2(0, a, 2);
        if (beta1 != 0.0) {
          double b_tmp;
          d = a[1];
          tau = std::abs(d);
          b_tmp = std::abs(beta1);
          if (tau < b_tmp) {
            tau /= b_tmp;
            beta1 = b_tmp * std::sqrt(tau * tau + 1.0);
          } else if (tau > b_tmp) {
            beta1 = b_tmp / tau;
            beta1 = tau * std::sqrt(beta1 * beta1 + 1.0);
          } else if (std::isnan(b_tmp)) {
            beta1 = rtNaN;
          } else {
            beta1 = tau * 1.4142135623730951;
          }
          if (d >= 0.0) {
            beta1 = -beta1;
          }
          if (std::abs(beta1) < 1.0020841800044864E-292) {
            knt = 0;
            do {
              knt++;
              beta1 *= 9.9792015476736E+291;
              alpha1 *= 9.9792015476736E+291;
            } while ((std::abs(beta1) < 1.0020841800044864E-292) && (knt < 20));
            tau = std::abs(alpha1);
            if (tau < b_tmp) {
              tau /= b_tmp;
              beta1 = b_tmp * std::sqrt(tau * tau + 1.0);
            } else if (tau > b_tmp) {
              beta1 = b_tmp / tau;
              beta1 = tau * std::sqrt(beta1 * beta1 + 1.0);
            } else if (std::isnan(b_tmp)) {
              beta1 = rtNaN;
            } else {
              beta1 = tau * 1.4142135623730951;
            }
            if (alpha1 >= 0.0) {
              beta1 = -beta1;
            }
            tau = (beta1 - alpha1) / beta1;
            for (lastv = 0; lastv < knt; lastv++) {
              beta1 *= 1.0020841800044864E-292;
            }
            alpha1 = beta1;
          } else {
            tau = (beta1 - d) / beta1;
            alpha1 = beta1;
          }
        }
      }
      a[1] = 1.0;
      if (tau != 0.0) {
        bool exitg2;
        lastv = ihi - 1;
        knt = ihi;
        while ((lastv > 0) && (a[knt - 1] == 0.0)) {
          lastv = 0;
          knt--;
        }
        lastc = ihi;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          ia = lastc + 1;
          do {
            exitg1 = 0;
            if (ia + 1 <= (lastc + ((lastv - 1) << 1)) + 2) {
              if (a[ia] != 0.0) {
                exitg1 = 1;
              } else {
                ia += 2;
              }
            } else {
              lastc--;
              exitg1 = 2;
            }
          } while (exitg1 == 0);
          if (exitg1 == 1) {
            exitg2 = true;
          }
        }
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        __m128d r;
        __m128d r1;
        int vectorUB;
        if (lastc != 0) {
          knt = static_cast<unsigned char>(lastc);
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(knt) * sizeof(double));
          knt = lastc + 2;
          lastv = (knt - 2) / 2 * 2 + 3;
          vectorUB = lastv - 2;
          for (ia = 3; ia <= vectorUB; ia += 2) {
            r = _mm_loadu_pd(&a[ia - 1]);
            r = _mm_mul_pd(r, _mm_set1_pd(a[1]));
            r1 = _mm_loadu_pd(&work[ia - 3]);
            r = _mm_add_pd(r1, r);
            _mm_storeu_pd(&work[ia - 3], r);
          }
          for (ia = lastv; ia <= knt; ia++) {
            work[ia - 3] += a[ia - 1] * a[1];
          }
        }
        if (!(-tau == 0.0)) {
          d = a[1];
          if (d != 0.0) {
            beta1 = d * -tau;
            knt = lastc + 3;
            lastv = (knt - 3) / 2 * 2 + 3;
            vectorUB = lastv - 2;
            for (ia = 3; ia <= vectorUB; ia += 2) {
              r = _mm_loadu_pd(&work[ia - 3]);
              r = _mm_mul_pd(r, _mm_set1_pd(beta1));
              r1 = _mm_loadu_pd(&a[ia - 1]);
              r = _mm_add_pd(r1, r);
              _mm_storeu_pd(&a[ia - 1], r);
            }
            for (ia = lastv; ia < knt; ia++) {
              a[ia - 1] += work[ia - 3] * beta1;
            }
          }
        }
      }
      if (tau != 0.0) {
        lastv = ihi - 1;
        knt = ihi;
        while ((lastv > 0) && (a[knt - 1] == 0.0)) {
          lastv = 0;
          knt--;
        }
        lastc = 1;
        ia = 4;
        do {
          exitg1 = 0;
          if (ia <= lastv + 3) {
            if (a[3] != 0.0) {
              exitg1 = 1;
            } else {
              ia++;
            }
          } else {
            lastc = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
      } else {
        lastv = 0;
        lastc = 0;
      }
      if (lastv > 0) {
        if (lastc != 0) {
          work[0] = a[1] * a[3];
        }
        if (!(-tau == 0.0)) {
          knt = 4;
          for (lastv = 0; lastv < lastc; lastv++) {
            if (work[0] != 0.0) {
              beta1 = work[0] * -tau;
              for (ia = knt; ia <= knt; ia++) {
                a[ia - 1] += a[(ia - knt) + 1] * beta1;
              }
            }
            knt += 2;
          }
        }
      }
      a[1] = alpha1;
    }
  }
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xzgehrd.cpp)
