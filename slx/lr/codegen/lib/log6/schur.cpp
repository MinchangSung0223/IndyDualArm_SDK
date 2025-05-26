//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// schur.cpp
//
// Code generation for function 'schur'
//

// Include files
#include "schur.h"
#include "rt_nonfinite.h"
#include "xhseqr.h"
#include "xnrm2.h"
#include "xzlarf.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Function Definitions
namespace coder {
void schur(const double A[16], double V[16], double T[16])
{
  double work[4];
  double xnorm;
  int i;
  bool p;
  p = true;
  for (i = 0; i < 16; i++) {
    if (p) {
      xnorm = A[i];
      if (std::isinf(xnorm) || std::isnan(xnorm)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    int knt;
    for (int i1{0}; i1 < 16; i1++) {
      V[i1] = rtNaN;
    }
    knt = 2;
    for (i = 0; i < 3; i++) {
      if (knt <= 4) {
        std::memset(&V[(i * 4 + knt) + -1], 0,
                    static_cast<unsigned int>(-knt + 5) * sizeof(double));
      }
      knt++;
    }
    for (int i1{0}; i1 < 16; i1++) {
      T[i1] = rtNaN;
    }
  } else {
    __m128d r;
    double tau[3];
    int i1;
    int iaii;
    int ix0;
    int knt;
    int scalarLB;
    int vectorUB;
    std::copy(&A[0], &A[16], &T[0]);
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (int b_i{0}; b_i < 3; b_i++) {
      double alpha1_tmp;
      int alpha1_tmp_tmp_tmp;
      int in;
      int lastc;
      int lastv;
      int vectorUB_tmp;
      knt = b_i << 2;
      in = (b_i + 1) << 2;
      alpha1_tmp_tmp_tmp = b_i + knt;
      alpha1_tmp = T[alpha1_tmp_tmp_tmp + 1];
      if (b_i + 3 <= 4) {
        i = b_i + 1;
      } else {
        i = 2;
      }
      ix0 = (i + knt) + 2;
      tau[b_i] = 0.0;
      xnorm = internal::blas::xnrm2(2 - b_i, T, ix0);
      if (xnorm != 0.0) {
        double a;
        a = std::abs(alpha1_tmp);
        xnorm = std::abs(xnorm);
        if (a < xnorm) {
          a /= xnorm;
          xnorm *= std::sqrt(a * a + 1.0);
        } else if (a > xnorm) {
          xnorm /= a;
          xnorm = a * std::sqrt(xnorm * xnorm + 1.0);
        } else if (std::isnan(xnorm)) {
          xnorm = rtNaN;
        } else {
          xnorm = a * 1.4142135623730951;
        }
        if (alpha1_tmp >= 0.0) {
          xnorm = -xnorm;
        }
        if (std::abs(xnorm) < 1.0020841800044864E-292) {
          knt = 0;
          i1 = (ix0 - b_i) + 1;
          do {
            knt++;
            iaii = ((((i1 - ix0) + 1) / 2) << 1) + ix0;
            vectorUB_tmp = iaii - 2;
            for (i = ix0; i <= vectorUB_tmp; i += 2) {
              r = _mm_loadu_pd(&T[i - 1]);
              _mm_storeu_pd(&T[i - 1],
                            _mm_mul_pd(_mm_set1_pd(9.9792015476736E+291), r));
            }
            for (i = iaii; i <= i1; i++) {
              T[i - 1] *= 9.9792015476736E+291;
            }
            xnorm *= 9.9792015476736E+291;
            alpha1_tmp *= 9.9792015476736E+291;
          } while ((std::abs(xnorm) < 1.0020841800044864E-292) && (knt < 20));
          a = std::abs(alpha1_tmp);
          xnorm = std::abs(internal::blas::xnrm2(2 - b_i, T, ix0));
          if (a < xnorm) {
            a /= xnorm;
            xnorm *= std::sqrt(a * a + 1.0);
          } else if (a > xnorm) {
            xnorm /= a;
            xnorm = a * std::sqrt(xnorm * xnorm + 1.0);
          } else if (std::isnan(xnorm)) {
            xnorm = rtNaN;
          } else {
            xnorm = a * 1.4142135623730951;
          }
          if (alpha1_tmp >= 0.0) {
            xnorm = -xnorm;
          }
          tau[b_i] = (xnorm - alpha1_tmp) / xnorm;
          a = 1.0 / (alpha1_tmp - xnorm);
          for (i = ix0; i <= vectorUB_tmp; i += 2) {
            r = _mm_loadu_pd(&T[i - 1]);
            _mm_storeu_pd(&T[i - 1], _mm_mul_pd(_mm_set1_pd(a), r));
          }
          for (i = iaii; i <= i1; i++) {
            T[i - 1] *= a;
          }
          for (i = 0; i < knt; i++) {
            xnorm *= 1.0020841800044864E-292;
          }
          alpha1_tmp = xnorm;
        } else {
          tau[b_i] = (xnorm - alpha1_tmp) / xnorm;
          a = 1.0 / (alpha1_tmp - xnorm);
          i1 = (ix0 - b_i) + 1;
          scalarLB = ((((i1 - ix0) + 1) / 2) << 1) + ix0;
          vectorUB = scalarLB - 2;
          for (i = ix0; i <= vectorUB; i += 2) {
            r = _mm_loadu_pd(&T[i - 1]);
            _mm_storeu_pd(&T[i - 1], _mm_mul_pd(_mm_set1_pd(a), r));
          }
          for (i = scalarLB; i <= i1; i++) {
            T[i - 1] *= a;
          }
          alpha1_tmp = xnorm;
        }
      }
      T[alpha1_tmp_tmp_tmp + 1] = 1.0;
      i = in + 1;
      if (tau[b_i] != 0.0) {
        bool exitg2;
        lastv = 2 - b_i;
        knt = (alpha1_tmp_tmp_tmp - b_i) + 3;
        while ((lastv + 1 > 0) && (T[knt] == 0.0)) {
          lastv--;
          knt--;
        }
        lastc = 4;
        exitg2 = false;
        while ((!exitg2) && (lastc > 0)) {
          int exitg1;
          knt = in + lastc;
          ix0 = knt;
          do {
            exitg1 = 0;
            if (ix0 <= knt + (lastv << 2)) {
              if (T[ix0 - 1] != 0.0) {
                exitg1 = 1;
              } else {
                ix0 += 4;
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
        lastv = -1;
        lastc = 0;
      }
      if (lastv + 1 > 0) {
        int i2;
        if (lastc != 0) {
          std::memset(&work[0], 0,
                      static_cast<unsigned int>(lastc) * sizeof(double));
          knt = alpha1_tmp_tmp_tmp + 2;
          i1 = (in + (lastv << 2)) + 1;
          for (vectorUB_tmp = i; vectorUB_tmp <= i1; vectorUB_tmp += 4) {
            i2 = vectorUB_tmp + lastc;
            for (ix0 = vectorUB_tmp; ix0 < i2; ix0++) {
              iaii = ix0 - vectorUB_tmp;
              work[iaii] += T[ix0 - 1] * T[knt - 1];
            }
            knt++;
          }
        }
        if (!(-tau[b_i] == 0.0)) {
          knt = in;
          for (i = 0; i <= lastv; i++) {
            xnorm = T[(alpha1_tmp_tmp_tmp + i) + 1];
            if (xnorm != 0.0) {
              xnorm *= -tau[b_i];
              i1 = knt + 1;
              i2 = lastc + knt;
              scalarLB = ((((i2 - knt) / 2) << 1) + knt) + 1;
              vectorUB = scalarLB - 2;
              for (vectorUB_tmp = i1; vectorUB_tmp <= vectorUB;
                   vectorUB_tmp += 2) {
                __m128d r1;
                r = _mm_loadu_pd(&work[(vectorUB_tmp - knt) - 1]);
                r1 = _mm_loadu_pd(&T[vectorUB_tmp - 1]);
                _mm_storeu_pd(
                    &T[vectorUB_tmp - 1],
                    _mm_add_pd(r1, _mm_mul_pd(r, _mm_set1_pd(xnorm))));
              }
              for (vectorUB_tmp = scalarLB; vectorUB_tmp <= i2;
                   vectorUB_tmp++) {
                T[vectorUB_tmp - 1] += work[(vectorUB_tmp - knt) - 1] * xnorm;
              }
            }
            knt += 4;
          }
        }
      }
      internal::reflapack::xzlarf(3 - b_i, 3 - b_i, alpha1_tmp_tmp_tmp + 2,
                                  tau[b_i], T, (b_i + in) + 2, work);
      T[alpha1_tmp_tmp_tmp + 1] = alpha1_tmp;
    }
    std::copy(&T[0], &T[16], &V[0]);
    for (i = 2; i >= 0; i--) {
      ix0 = (i + 1) << 2;
      for (int b_i{0}; b_i <= i; b_i++) {
        V[ix0 + b_i] = 0.0;
      }
      i1 = i + 3;
      for (int b_i{i1}; b_i < 5; b_i++) {
        knt = ix0 + b_i;
        V[knt - 1] = V[knt - 5];
      }
    }
    V[1] = 0.0;
    V[2] = 0.0;
    V[3] = 0.0;
    V[0] = 1.0;
    knt = 2;
    work[0] = 0.0;
    work[1] = 0.0;
    work[2] = 0.0;
    work[3] = 0.0;
    for (int b_i{2}; b_i >= 0; b_i--) {
      iaii = (b_i + (b_i << 2)) + 5;
      if (b_i + 1 < 3) {
        V[iaii] = 1.0;
        internal::reflapack::xzlarf(3 - b_i, 2 - b_i, iaii + 1, tau[knt], V,
                                    iaii + 5, work);
        ix0 = iaii + 2;
        i1 = (iaii - b_i) + 3;
        scalarLB = (((((i1 - iaii) - 1) / 2) << 1) + iaii) + 2;
        vectorUB = scalarLB - 2;
        for (i = ix0; i <= vectorUB; i += 2) {
          r = _mm_loadu_pd(&V[i - 1]);
          _mm_storeu_pd(&V[i - 1], _mm_mul_pd(_mm_set1_pd(-tau[knt]), r));
        }
        for (i = scalarLB; i <= i1; i++) {
          V[i - 1] *= -tau[knt];
        }
      }
      V[iaii] = 1.0 - tau[knt];
      for (i = 0; i < b_i; i++) {
        V[(iaii - i) - 1] = 0.0;
      }
      knt = b_i - 1;
    }
    internal::lapack::xhseqr(T, V);
  }
}

} // namespace coder

// End of code generation (schur.cpp)
