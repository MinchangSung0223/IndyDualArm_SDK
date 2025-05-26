//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// ordeig.cpp
//
// Code generation for function 'ordeig'
//

// Include files
#include "ordeig.h"
#include "eigStandard.h"
#include "rt_nonfinite.h"
#include "xdlahqr.h"
#include "xdsterf.h"
#include "xzgehrd.h"
#include "xzlascl.h"
#include <cmath>
#include <emmintrin.h>

// Function Definitions
namespace coder {
void ordeig(const double A[16], creal_T E[4])
{
  double a__3[2];
  double anrm;
  int k;
  bool iscale;
  iscale = true;
  for (k = 0; k < 16; k++) {
    if (iscale) {
      anrm = A[k];
      if (std::isinf(anrm) || std::isnan(anrm)) {
        iscale = false;
      }
    } else {
      iscale = false;
    }
  }
  if (!iscale) {
    E[0].re = rtNaN;
    E[0].im = 0.0;
    E[1].re = rtNaN;
    E[1].im = 0.0;
    E[2].re = rtNaN;
    E[2].im = 0.0;
    E[3].re = rtNaN;
    E[3].im = 0.0;
  } else {
    k = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (k + 1 <= 4) {
        bool guard1;
        guard1 = false;
        if (k + 1 != 4) {
          int At_tmp;
          At_tmp = k + (k << 2);
          anrm = A[At_tmp + 1];
          if (anrm != 0.0) {
            creal_T etemp[2];
            double At[4];
            At[0] = A[At_tmp];
            At_tmp = k + ((k + 1) << 2);
            At[2] = A[At_tmp];
            At[1] = anrm;
            At[3] = A[At_tmp + 1];
            iscale = true;
            for (At_tmp = 0; At_tmp < 4; At_tmp++) {
              if (iscale) {
                anrm = At[At_tmp];
                if (std::isinf(anrm) || std::isnan(anrm)) {
                  iscale = false;
                }
              } else {
                iscale = false;
              }
            }
            if (!iscale) {
              etemp[0].re = rtNaN;
              etemp[0].im = 0.0;
              etemp[1].re = rtNaN;
              etemp[1].im = 0.0;
            } else {
              int exitg2;
              int i;
              bool exitg3;
              iscale = true;
              At_tmp = 0;
              exitg3 = false;
              while ((!exitg3) && (At_tmp < 2)) {
                i = 0;
                do {
                  exitg2 = 0;
                  if (i <= At_tmp) {
                    if (!(At[i + (At_tmp << 1)] == At[At_tmp + (i << 1)])) {
                      iscale = false;
                      exitg2 = 1;
                    } else {
                      i++;
                    }
                  } else {
                    At_tmp++;
                    exitg2 = 2;
                  }
                } while (exitg2 == 0);
                if (exitg2 == 1) {
                  exitg3 = true;
                }
              }
              if (iscale) {
                anrm = 0.0;
                At_tmp = 0;
                exitg3 = false;
                while ((!exitg3) && (At_tmp < 2)) {
                  i = 0;
                  do {
                    exitg2 = 0;
                    if (i <= At_tmp) {
                      double absx;
                      absx = std::abs(At[i + (At_tmp << 1)]);
                      if (std::isnan(absx)) {
                        anrm = rtNaN;
                        exitg2 = 1;
                      } else {
                        if (absx > anrm) {
                          anrm = absx;
                        }
                        i++;
                      }
                    } else {
                      At_tmp++;
                      exitg2 = 2;
                    }
                  } while (exitg2 == 0);
                  if (exitg2 == 1) {
                    exitg3 = true;
                  }
                }
                if (std::isinf(anrm) || std::isnan(anrm)) {
                  a__3[0] = rtNaN;
                  a__3[1] = rtNaN;
                } else {
                  int info;
                  iscale = false;
                  if ((anrm > 0.0) && (anrm < 1.0010415475915505E-146)) {
                    iscale = true;
                    anrm = 1.0010415475915505E-146 / anrm;
                    internal::reflapack::xzlascl(1.0, anrm, At);
                  } else if (anrm > 9.9895953610111751E+145) {
                    iscale = true;
                    anrm = 9.9895953610111751E+145 / anrm;
                    internal::reflapack::xzlascl(1.0, anrm, At);
                  }
                  a__3[0] = At[0];
                  a__3[1] = At[3];
                  info = internal::reflapack::xdsterf(a__3, At[1]);
                  if (info != 0) {
                    a__3[0] = rtNaN;
                    a__3[1] = rtNaN;
                  } else if (iscale) {
                    __m128d r;
                    r = _mm_loadu_pd(&a__3[0]);
                    _mm_storeu_pd(&a__3[0],
                                  _mm_mul_pd(_mm_set1_pd(1.0 / anrm), r));
                  }
                }
                etemp[0].re = a__3[0];
                etemp[0].im = 0.0;
                etemp[1].re = a__3[1];
                etemp[1].im = 0.0;
              } else {
                iscale = true;
                At_tmp = 0;
                exitg3 = false;
                while ((!exitg3) && (At_tmp < 2)) {
                  i = 0;
                  do {
                    exitg2 = 0;
                    if (i <= At_tmp) {
                      if (!(At[i + (At_tmp << 1)] == -At[At_tmp + (i << 1)])) {
                        iscale = false;
                        exitg2 = 1;
                      } else {
                        i++;
                      }
                    } else {
                      At_tmp++;
                      exitg2 = 2;
                    }
                  } while (exitg2 == 0);
                  if (exitg2 == 1) {
                    exitg3 = true;
                  }
                }
                if (iscale) {
                  double wi[2];
                  int info;
                  internal::reflapack::xzgehrd(At, 1, 2);
                  info = internal::reflapack::xdlahqr(1, 2, At, anrm, a__3, wi);
                  At_tmp = static_cast<unsigned char>(info);
                  for (i = 0; i < At_tmp; i++) {
                    etemp[i].re = rtNaN;
                    etemp[i].im = 0.0;
                  }
                  At_tmp = info + 1;
                  for (i = At_tmp; i < 3; i++) {
                    etemp[i - 1].re = 0.0;
                    etemp[i - 1].im = wi[i - 1];
                  }
                } else {
                  eigStandard(At, etemp);
                }
              }
            }
            E[k] = etemp[0];
            E[k + 1] = etemp[1];
            k += 2;
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          E[k].re = A[k + (k << 2)];
          E[k].im = 0.0;
          k++;
        }
      } else {
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
}

} // namespace coder

// End of code generation (ordeig.cpp)
