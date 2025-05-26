//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// logm.cpp
//
// Code generation for function 'logm'
//

// Include files
#include "logm.h"
#include "atanh.h"
#include "checkCondition.h"
#include "log.h"
#include "log6_data.h"
#include "log6_rtwutil.h"
#include "norm.h"
#include "ordeig.h"
#include "power.h"
#include "rt_nonfinite.h"
#include "schur.h"
#include "sqrt.h"
#include "sqrtObo.h"
#include "sqrtm2by2.h"
#include "sqrtmTri.h"
#include "xdlanv2.h"
#include "xgetrf.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <emmintrin.h>

// Variable Definitions
static const double dv[49]{0.5,
                           0.0,
                           0.0,
                           0.0,
                           0.0,
                           0.0,
                           0.0,
                           0.21132486540518711,
                           0.78867513459481287,
                           0.0,
                           0.0,
                           0.0,
                           0.0,
                           0.0,
                           0.11270166537925831,
                           0.5,
                           0.8872983346207417,
                           0.0,
                           0.0,
                           0.0,
                           0.0,
                           0.069431844202973714,
                           0.33000947820757187,
                           0.66999052179242813,
                           0.93056815579702634,
                           0.0,
                           0.0,
                           0.0,
                           0.046910077030668004,
                           0.23076534494715845,
                           0.5,
                           0.7692346550528415,
                           0.953089922969332,
                           0.0,
                           0.0,
                           0.033765242898423989,
                           0.16939530676686773,
                           0.38069040695840156,
                           0.61930959304159849,
                           0.83060469323313224,
                           0.966234757101576,
                           0.0,
                           0.025446043828620736,
                           0.12923440720030277,
                           0.29707742431130141,
                           0.5,
                           0.70292257568869854,
                           0.87076559279969723,
                           0.9745539561713793};

static const double dv1[49]{1.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.5,
                            0.5,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.27777777777777779,
                            0.44444444444444442,
                            0.27777777777777779,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            0.17392742256872692,
                            0.32607257743127305,
                            0.32607257743127305,
                            0.17392742256872692,
                            0.0,
                            0.0,
                            0.0,
                            0.11846344252809454,
                            0.23931433524968324,
                            0.28444444444444444,
                            0.23931433524968324,
                            0.11846344252809454,
                            0.0,
                            0.0,
                            0.085662246189585178,
                            0.1803807865240693,
                            0.23395696728634552,
                            0.23395696728634552,
                            0.1803807865240693,
                            0.085662246189585178,
                            0.0,
                            0.064742483084434851,
                            0.13985269574463832,
                            0.19091502525255946,
                            0.2089795918367347,
                            0.19091502525255946,
                            0.13985269574463832,
                            0.064742483084434851};

static const double dv2[7]{1.5869707387720632E-5, 0.0023138078842429789,
                           0.019381793135332531,  0.062091715889947621,
                           0.12764048108067749,   0.20609626234528361,
                           0.28790937142411938};

// Function Declarations
namespace coder {
static int computeLogOfSchurForm(const creal_T T[16], const creal_T d[4],
                                 creal_T L[16]);

static int computeLogOfSchurForm(const double T[16], const creal_T d[4],
                                 double L[16]);

static int logmParams(creal_T T[16], creal_T d[4], int &m, int &exitflag);

static int logmParams(double T[16], creal_T d[4], int &m, int &exitflag);

} // namespace coder

// Function Definitions
namespace coder {
static int computeLogOfSchurForm(const creal_T T[16], const creal_T d[4],
                                 creal_T L[16])
{
  creal_T Troot[16];
  creal_T P[4];
  creal_T b_P[4];
  creal_T b_d[4];
  creal_T r11;
  double P_re_tmp;
  double ai;
  double ai_tmp;
  double ar;
  double ar_tmp;
  double bi;
  double bim;
  double br;
  double brm;
  double im;
  double r;
  double r11_re_tmp;
  double r22_tmp;
  double re;
  double wt;
  int exitflag;
  int i;
  int i1;
  int jA;
  int jp1j;
  int m;
  int r1;
  int r2;
  int s;
  std::copy(&T[0], &T[16], &Troot[0]);
  std::copy(&d[0], &d[4], &b_d[0]);
  s = logmParams(Troot, b_d, m, exitflag);
  for (int j{0}; j < 3; j++) {
    if (s == 0) {
      i = j + (j << 2);
      Troot[i].re = T[i].re - 1.0;
      Troot[i].im = T[i].im;
      Troot[i + 1].re = T[i + 1].re;
      Troot[i + 1].im = T[i + 1].im;
      i = j + ((j + 1) << 2);
      Troot[i].re = T[i].re;
      Troot[i].im = T[i].im;
      Troot[i + 1].re = T[i + 1].re - 1.0;
      Troot[i + 1].im = T[i + 1].im;
    } else {
      creal_T Z0[4];
      double b_ar_tmp;
      jA = j + (j << 2);
      b_d[1] = T[jA + 1];
      jp1j = j + ((j + 1) << 2);
      b_d[2] = T[jp1j];
      r11 = T[jA];
      internal::scalar::b_sqrt(r11);
      b_d[0] = T[jp1j + 1];
      internal::scalar::b_sqrt(b_d[0]);
      br = r11.re + b_d[0].re;
      bi = r11.im + b_d[0].im;
      if (bi == 0.0) {
        if (b_d[2].im == 0.0) {
          re = b_d[2].re / br;
          im = 0.0;
        } else if (b_d[2].re == 0.0) {
          re = 0.0;
          im = b_d[2].im / br;
        } else {
          re = b_d[2].re / br;
          im = b_d[2].im / br;
        }
      } else if (br == 0.0) {
        if (b_d[2].re == 0.0) {
          re = b_d[2].im / bi;
          im = 0.0;
        } else if (b_d[2].im == 0.0) {
          re = 0.0;
          im = -(b_d[2].re / bi);
        } else {
          re = b_d[2].im / bi;
          im = -(b_d[2].re / bi);
        }
      } else {
        brm = std::abs(br);
        bim = std::abs(bi);
        if (brm > bim) {
          bim = bi / br;
          bi = br + bim * bi;
          re = (b_d[2].re + bim * b_d[2].im) / bi;
          im = (b_d[2].im - bim * b_d[2].re) / bi;
        } else if (bim == brm) {
          if (br > 0.0) {
            br = 0.5;
          } else {
            br = -0.5;
          }
          if (bi > 0.0) {
            bi = 0.5;
          } else {
            bi = -0.5;
          }
          re = (b_d[2].re * br + b_d[2].im * bi) / brm;
          im = (b_d[2].im * br - b_d[2].re * bi) / brm;
        } else {
          bim = br / bi;
          bi += bim * br;
          re = (bim * b_d[2].re + b_d[2].im) / bi;
          im = (bim * b_d[2].im - b_d[2].re) / bi;
        }
      }
      ar_tmp = r11.re - 1.0;
      Z0[0].re = r11.re - 1.0;
      ai = r11.im;
      Z0[0].im = r11.im;
      b_ar_tmp = b_d[1].re;
      Z0[1].re = b_d[1].re;
      ai_tmp = b_d[1].im;
      Z0[1].im = b_d[1].im;
      Z0[2].re = re;
      Z0[2].im = im;
      Z0[3].re = b_d[0].re - 1.0;
      Z0[3].im = b_d[0].im;
      if (s == 1) {
        Troot[jA] = Z0[0];
        Troot[jA + 1] = Z0[1];
        Troot[jp1j] = Z0[2];
        Troot[jp1j + 1] = Z0[3];
      } else {
        double b_P_re_tmp;
        double b_re;
        double c_P_re_tmp;
        internal::scalar::b_sqrt(r11);
        internal::scalar::b_sqrt(b_d[0]);
        br = r11.re + b_d[0].re;
        bi = r11.im + b_d[0].im;
        if (bi == 0.0) {
          if (im == 0.0) {
            b_re = re / br;
            r = 0.0;
          } else if (re == 0.0) {
            b_re = 0.0;
            r = im / br;
          } else {
            b_re = re / br;
            r = im / br;
          }
        } else if (br == 0.0) {
          if (re == 0.0) {
            b_re = im / bi;
            r = 0.0;
          } else if (im == 0.0) {
            b_re = 0.0;
            r = -(re / bi);
          } else {
            b_re = im / bi;
            r = -(re / bi);
          }
        } else {
          brm = std::abs(br);
          bim = std::abs(bi);
          if (brm > bim) {
            bim = bi / br;
            bi = br + bim * bi;
            b_re = (re + bim * im) / bi;
            r = (im - bim * re) / bi;
          } else if (bim == brm) {
            if (br > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (bi > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            b_re = (re * br + im * bi) / brm;
            r = (im * br - re * bi) / brm;
          } else {
            bim = br / bi;
            bi += bim * br;
            b_re = (bim * re + im) / bi;
            r = (bim * im - re) / bi;
          }
        }
        b_d[2].re = b_re;
        b_d[2].im = r;
        P[0].re = r11.re + 1.0;
        P[0].im = r11.im;
        P[1].re = b_d[1].re;
        P[1].im = b_d[1].im;
        P[2].re = b_re;
        P[2].im = r;
        P[3].re = b_d[0].re + 1.0;
        P[3].im = b_d[0].im;
        i = static_cast<unsigned char>(s - 2);
        for (int b_i{0}; b_i < i; b_i++) {
          internal::scalar::b_sqrt(r11);
          internal::scalar::b_sqrt(b_d[0]);
          br = r11.re + b_d[0].re;
          bi = r11.im + b_d[0].im;
          if (bi == 0.0) {
            if (b_d[2].im == 0.0) {
              b_re = b_d[2].re / br;
              r = 0.0;
            } else if (b_d[2].re == 0.0) {
              b_re = 0.0;
              r = b_d[2].im / br;
            } else {
              b_re = b_d[2].re / br;
              r = b_d[2].im / br;
            }
          } else if (br == 0.0) {
            if (b_d[2].re == 0.0) {
              b_re = b_d[2].im / bi;
              r = 0.0;
            } else if (b_d[2].im == 0.0) {
              b_re = 0.0;
              r = -(b_d[2].re / bi);
            } else {
              b_re = b_d[2].im / bi;
              r = -(b_d[2].re / bi);
            }
          } else {
            brm = std::abs(br);
            bim = std::abs(bi);
            if (brm > bim) {
              bim = bi / br;
              bi = br + bim * bi;
              b_re = (b_d[2].re + bim * b_d[2].im) / bi;
              r = (b_d[2].im - bim * b_d[2].re) / bi;
            } else if (bim == brm) {
              if (br > 0.0) {
                br = 0.5;
              } else {
                br = -0.5;
              }
              if (bi > 0.0) {
                bi = 0.5;
              } else {
                bi = -0.5;
              }
              b_re = (b_d[2].re * br + b_d[2].im * bi) / brm;
              r = (b_d[2].im * br - b_d[2].re * bi) / brm;
            } else {
              bim = br / bi;
              bi += bim * br;
              b_re = (bim * b_d[2].re + b_d[2].im) / bi;
              r = (bim * b_d[2].im - b_d[2].re) / bi;
            }
          }
          b_d[2].re = b_re;
          b_d[2].im = r;
          bi = r11.re + 1.0;
          r11_re_tmp = r11.im;
          r22_tmp = b_d[0].re + 1.0;
          br = b_d[0].im;
          for (i1 = 0; i1 < 2; i1++) {
            P_re_tmp = P[i1].re;
            wt = P[i1].im;
            b_P_re_tmp = P[i1 + 2].re;
            c_P_re_tmp = P[i1 + 2].im;
            b_P[i1].re = (P_re_tmp * bi - wt * r11_re_tmp) +
                         (b_P_re_tmp * b_ar_tmp - c_P_re_tmp * ai_tmp);
            b_P[i1].im = (P_re_tmp * r11_re_tmp + wt * bi) +
                         (b_P_re_tmp * ai_tmp + c_P_re_tmp * b_ar_tmp);
            b_P[i1 + 2].re = (P_re_tmp * b_re - wt * r) +
                             (b_P_re_tmp * r22_tmp - c_P_re_tmp * br);
            b_P[i1 + 2].im = (P_re_tmp * r + wt * b_re) +
                             (b_P_re_tmp * br + c_P_re_tmp * r22_tmp);
          }
          std::copy(&b_P[0], &b_P[4], &P[0]);
        }
        if (std::abs(P[1].re) + std::abs(P[1].im) >
            std::abs(P[0].re) + std::abs(P[0].im)) {
          r1 = 1;
          r2 = 0;
        } else {
          r1 = 0;
          r2 = 1;
        }
        ar = P[r2].re;
        r = P[r2].im;
        b_P_re_tmp = P[r1].re;
        c_P_re_tmp = P[r1].im;
        if (c_P_re_tmp == 0.0) {
          if (r == 0.0) {
            r11.re = ar / b_P_re_tmp;
            r11.im = 0.0;
          } else if (ar == 0.0) {
            r11.re = 0.0;
            r11.im = r / b_P_re_tmp;
          } else {
            r11.re = ar / b_P_re_tmp;
            r11.im = r / b_P_re_tmp;
          }
        } else if (b_P_re_tmp == 0.0) {
          if (ar == 0.0) {
            r11.re = r / c_P_re_tmp;
            r11.im = 0.0;
          } else if (r == 0.0) {
            r11.re = 0.0;
            r11.im = -(ar / c_P_re_tmp);
          } else {
            r11.re = r / c_P_re_tmp;
            r11.im = -(ar / c_P_re_tmp);
          }
        } else {
          brm = std::abs(b_P_re_tmp);
          bim = std::abs(c_P_re_tmp);
          if (brm > bim) {
            bim = c_P_re_tmp / b_P_re_tmp;
            bi = b_P_re_tmp + bim * c_P_re_tmp;
            r11.re = (ar + bim * r) / bi;
            r11.im = (r - bim * ar) / bi;
          } else if (bim == brm) {
            if (b_P_re_tmp > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (c_P_re_tmp > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            r11.re = (ar * br + r * bi) / brm;
            r11.im = (r * br - ar * bi) / brm;
          } else {
            bim = b_P_re_tmp / c_P_re_tmp;
            bi = c_P_re_tmp + bim * b_P_re_tmp;
            r11.re = (bim * ar + r) / bi;
            r11.im = (bim * r - ar) / bi;
          }
        }
        b_re = P[r1 + 2].im;
        r11_re_tmp = P[r1 + 2].re;
        r22_tmp = P[r2 + 2].re - (r11.re * r11_re_tmp - r11.im * b_re);
        wt = P[r2 + 2].im - (r11.re * b_re + r11.im * r11_re_tmp);
        if (c_P_re_tmp == 0.0) {
          if (ai == 0.0) {
            i = r1 << 1;
            b_d[i].re = ar_tmp / b_P_re_tmp;
            b_d[i].im = 0.0;
          } else if (ar_tmp == 0.0) {
            i = r1 << 1;
            b_d[i].re = 0.0;
            b_d[i].im = ai / b_P_re_tmp;
          } else {
            i = r1 << 1;
            b_d[i].re = ar_tmp / b_P_re_tmp;
            b_d[i].im = ai / b_P_re_tmp;
          }
        } else if (b_P_re_tmp == 0.0) {
          if (ar_tmp == 0.0) {
            i = r1 << 1;
            b_d[i].re = ai / c_P_re_tmp;
            b_d[i].im = 0.0;
          } else if (ai == 0.0) {
            i = r1 << 1;
            b_d[i].re = 0.0;
            b_d[i].im = -(ar_tmp / c_P_re_tmp);
          } else {
            i = r1 << 1;
            b_d[i].re = ai / c_P_re_tmp;
            b_d[i].im = -(ar_tmp / c_P_re_tmp);
          }
        } else {
          brm = std::abs(b_P_re_tmp);
          bim = std::abs(c_P_re_tmp);
          if (brm > bim) {
            bim = c_P_re_tmp / b_P_re_tmp;
            bi = b_P_re_tmp + bim * c_P_re_tmp;
            i = r1 << 1;
            b_d[i].re = (ar_tmp + bim * ai) / bi;
            b_d[i].im = (ai - bim * ar_tmp) / bi;
          } else if (bim == brm) {
            if (b_P_re_tmp > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (c_P_re_tmp > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            i = r1 << 1;
            b_d[i].re = (ar_tmp * br + ai * bi) / brm;
            b_d[i].im = (ai * br - ar_tmp * bi) / brm;
          } else {
            bim = b_P_re_tmp / c_P_re_tmp;
            bi = c_P_re_tmp + bim * b_P_re_tmp;
            i = r1 << 1;
            b_d[i].re = (bim * ar_tmp + ai) / bi;
            b_d[i].im = (bim * ai - ar_tmp) / bi;
          }
        }
        r1 <<= 1;
        r = b_d[r1].re;
        P_re_tmp = b_d[r1].im;
        ar = re - (r * r11_re_tmp - P_re_tmp * b_re);
        ai = im - (r * b_re + P_re_tmp * r11_re_tmp);
        if (wt == 0.0) {
          if (ai == 0.0) {
            i = r2 << 1;
            b_d[i].re = ar / r22_tmp;
            b_d[i].im = 0.0;
          } else if (ar == 0.0) {
            i = r2 << 1;
            b_d[i].re = 0.0;
            b_d[i].im = ai / r22_tmp;
          } else {
            i = r2 << 1;
            b_d[i].re = ar / r22_tmp;
            b_d[i].im = ai / r22_tmp;
          }
        } else if (r22_tmp == 0.0) {
          if (ar == 0.0) {
            i = r2 << 1;
            b_d[i].re = ai / wt;
            b_d[i].im = 0.0;
          } else if (ai == 0.0) {
            i = r2 << 1;
            b_d[i].re = 0.0;
            b_d[i].im = -(ar / wt);
          } else {
            i = r2 << 1;
            b_d[i].re = ai / wt;
            b_d[i].im = -(ar / wt);
          }
        } else {
          brm = std::abs(r22_tmp);
          bim = std::abs(wt);
          if (brm > bim) {
            bim = wt / r22_tmp;
            bi = r22_tmp + bim * wt;
            i = r2 << 1;
            b_d[i].re = (ar + bim * ai) / bi;
            b_d[i].im = (ai - bim * ar) / bi;
          } else if (bim == brm) {
            if (r22_tmp > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (wt > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            i = r2 << 1;
            b_d[i].re = (ar * br + ai * bi) / brm;
            b_d[i].im = (ai * br - ar * bi) / brm;
          } else {
            bim = r22_tmp / wt;
            bi = wt + bim * r22_tmp;
            i = r2 << 1;
            b_d[i].re = (bim * ar + ai) / bi;
            b_d[i].im = (bim * ai - ar) / bi;
          }
        }
        r2 <<= 1;
        r = b_d[r2].re;
        P_re_tmp = b_d[r2].im;
        b_d[r1].re -= r * r11.re - P_re_tmp * r11.im;
        b_d[r1].im -= r * r11.im + P_re_tmp * r11.re;
        if (c_P_re_tmp == 0.0) {
          if (ai_tmp == 0.0) {
            b_d[r1 + 1].re = b_ar_tmp / b_P_re_tmp;
            b_d[r1 + 1].im = 0.0;
          } else if (b_ar_tmp == 0.0) {
            b_d[r1 + 1].re = 0.0;
            b_d[r1 + 1].im = ai_tmp / b_P_re_tmp;
          } else {
            b_d[r1 + 1].re = b_ar_tmp / b_P_re_tmp;
            b_d[r1 + 1].im = ai_tmp / b_P_re_tmp;
          }
        } else if (b_P_re_tmp == 0.0) {
          if (b_ar_tmp == 0.0) {
            b_d[r1 + 1].re = ai_tmp / c_P_re_tmp;
            b_d[r1 + 1].im = 0.0;
          } else if (ai_tmp == 0.0) {
            b_d[r1 + 1].re = 0.0;
            b_d[r1 + 1].im = -(b_ar_tmp / c_P_re_tmp);
          } else {
            b_d[r1 + 1].re = ai_tmp / c_P_re_tmp;
            b_d[r1 + 1].im = -(b_ar_tmp / c_P_re_tmp);
          }
        } else {
          brm = std::abs(b_P_re_tmp);
          bim = std::abs(c_P_re_tmp);
          if (brm > bim) {
            bim = c_P_re_tmp / b_P_re_tmp;
            bi = b_P_re_tmp + bim * c_P_re_tmp;
            b_d[r1 + 1].re = (b_ar_tmp + bim * ai_tmp) / bi;
            b_d[r1 + 1].im = (ai_tmp - bim * b_ar_tmp) / bi;
          } else if (bim == brm) {
            if (b_P_re_tmp > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (c_P_re_tmp > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            b_d[r1 + 1].re = (b_ar_tmp * br + ai_tmp * bi) / brm;
            b_d[r1 + 1].im = (ai_tmp * br - b_ar_tmp * bi) / brm;
          } else {
            bim = b_P_re_tmp / c_P_re_tmp;
            bi = c_P_re_tmp + bim * b_P_re_tmp;
            b_d[r1 + 1].re = (bim * b_ar_tmp + ai_tmp) / bi;
            b_d[r1 + 1].im = (bim * ai_tmp - b_ar_tmp) / bi;
          }
        }
        r = b_d[r1 + 1].re;
        P_re_tmp = b_d[r1 + 1].im;
        ar = Z0[3].re - (r * r11_re_tmp - P_re_tmp * b_re);
        ai = Z0[3].im - (r * b_re + P_re_tmp * r11_re_tmp);
        if (wt == 0.0) {
          if (ai == 0.0) {
            b_d[r2 + 1].re = ar / r22_tmp;
            b_d[r2 + 1].im = 0.0;
          } else if (ar == 0.0) {
            b_d[r2 + 1].re = 0.0;
            b_d[r2 + 1].im = ai / r22_tmp;
          } else {
            b_d[r2 + 1].re = ar / r22_tmp;
            b_d[r2 + 1].im = ai / r22_tmp;
          }
        } else if (r22_tmp == 0.0) {
          if (ar == 0.0) {
            b_d[r2 + 1].re = ai / wt;
            b_d[r2 + 1].im = 0.0;
          } else if (ai == 0.0) {
            b_d[r2 + 1].re = 0.0;
            b_d[r2 + 1].im = -(ar / wt);
          } else {
            b_d[r2 + 1].re = ai / wt;
            b_d[r2 + 1].im = -(ar / wt);
          }
        } else {
          brm = std::abs(r22_tmp);
          bim = std::abs(wt);
          if (brm > bim) {
            bim = wt / r22_tmp;
            bi = r22_tmp + bim * wt;
            b_d[r2 + 1].re = (ar + bim * ai) / bi;
            b_d[r2 + 1].im = (ai - bim * ar) / bi;
          } else if (bim == brm) {
            if (r22_tmp > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (wt > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            b_d[r2 + 1].re = (ar * br + ai * bi) / brm;
            b_d[r2 + 1].im = (ai * br - ar * bi) / brm;
          } else {
            bim = r22_tmp / wt;
            bi = wt + bim * r22_tmp;
            b_d[r2 + 1].re = (bim * ar + ai) / bi;
            b_d[r2 + 1].im = (bim * ai - ar) / bi;
          }
        }
        r = b_d[r2 + 1].re;
        P_re_tmp = b_d[r2 + 1].im;
        b_d[r1 + 1].re -= r * r11.re - P_re_tmp * r11.im;
        b_d[r1 + 1].im -= r * r11.im + P_re_tmp * r11.re;
        Troot[jA] = b_d[0];
        Troot[jA + 1] = b_d[1];
        Troot[jp1j] = b_d[2];
        Troot[jp1j + 1] = b_d[3];
        r2 = j + (j << 2);
        if ((T[r2 + 1].re == 0.0) && (T[r2 + 1].im == 0.0) &&
            (T[r2].re >= 0.0)) {
          i = j + ((j + 1) << 2);
          if (T[i + 1].re >= 0.0) {
            b_d[0] = T[j + (j << 2)];
            b_d[2] = T[j + ((j + 1) << 2)];
            b_d[3] = T[(j + ((j + 1) << 2)) + 1];
            wt = 1.0 / rt_powd_snf(2.0, static_cast<double>(s));
            if ((b_d[0].re == b_d[3].re) && (b_d[0].im == b_d[3].im)) {
              r11 = power(b_d[0], wt - 1.0);
              P_re_tmp = wt * b_d[2].re;
              r = wt * b_d[2].im;
              Troot[i].re = P_re_tmp * r11.re - r * r11.im;
              Troot[i].im = P_re_tmp * r11.im + r * r11.re;
            } else {
              ar_tmp = b_d[3].re - b_d[0].re;
              ai_tmp = b_d[3].im - b_d[0].im;
              br = b_d[0].re + b_d[3].re;
              bi = b_d[0].im + b_d[3].im;
              if (bi == 0.0) {
                if (ai_tmp == 0.0) {
                  r11.re = ar_tmp / br;
                  r11.im = 0.0;
                } else if (ar_tmp == 0.0) {
                  r11.re = 0.0;
                  r11.im = ai_tmp / br;
                } else {
                  r11.re = ar_tmp / br;
                  r11.im = ai_tmp / br;
                }
              } else if (br == 0.0) {
                if (ar_tmp == 0.0) {
                  r11.re = ai_tmp / bi;
                  r11.im = 0.0;
                } else if (ai_tmp == 0.0) {
                  r11.re = 0.0;
                  r11.im = -(ar_tmp / bi);
                } else {
                  r11.re = ai_tmp / bi;
                  r11.im = -(ar_tmp / bi);
                }
              } else {
                brm = std::abs(br);
                bim = std::abs(bi);
                if (brm > bim) {
                  bim = bi / br;
                  bi = br + bim * bi;
                  r11.re = (ar_tmp + bim * ai_tmp) / bi;
                  r11.im = (ai_tmp - bim * ar_tmp) / bi;
                } else if (bim == brm) {
                  if (br > 0.0) {
                    br = 0.5;
                  } else {
                    br = -0.5;
                  }
                  if (bi > 0.0) {
                    bi = 0.5;
                  } else {
                    bi = -0.5;
                  }
                  r11.re = (ar_tmp * br + ai_tmp * bi) / brm;
                  r11.im = (ai_tmp * br - ar_tmp * bi) / brm;
                } else {
                  bim = br / bi;
                  bi += bim * br;
                  r11.re = (bim * ar_tmp + ai_tmp) / bi;
                  r11.im = (bim * ai_tmp - ar_tmp) / bi;
                }
              }
              if (checkCondition(r11)) {
                r11 = power(b_d[3], wt);
                b_d[0] = power(b_d[0], wt);
                re = r11.re - b_d[0].re;
                im = r11.im - b_d[0].im;
                P_re_tmp = b_d[2].re * re - b_d[2].im * im;
                r = b_d[2].re * im + b_d[2].im * re;
                if (ai_tmp == 0.0) {
                  if (r == 0.0) {
                    Troot[i].re = P_re_tmp / ar_tmp;
                    Troot[i].im = 0.0;
                  } else if (P_re_tmp == 0.0) {
                    Troot[i].re = 0.0;
                    Troot[i].im = r / ar_tmp;
                  } else {
                    Troot[i].re = P_re_tmp / ar_tmp;
                    Troot[i].im = r / ar_tmp;
                  }
                } else if (ar_tmp == 0.0) {
                  if (P_re_tmp == 0.0) {
                    Troot[i].re = r / ai_tmp;
                    Troot[i].im = 0.0;
                  } else if (r == 0.0) {
                    Troot[i].re = 0.0;
                    Troot[i].im = -(P_re_tmp / ai_tmp);
                  } else {
                    Troot[i].re = r / ai_tmp;
                    Troot[i].im = -(P_re_tmp / ai_tmp);
                  }
                } else {
                  brm = std::abs(ar_tmp);
                  bim = std::abs(ai_tmp);
                  if (brm > bim) {
                    bim = ai_tmp / ar_tmp;
                    bi = ar_tmp + bim * ai_tmp;
                    Troot[i].re = (P_re_tmp + bim * r) / bi;
                    Troot[i].im = (r - bim * P_re_tmp) / bi;
                  } else if (bim == brm) {
                    if (ar_tmp > 0.0) {
                      br = 0.5;
                    } else {
                      br = -0.5;
                    }
                    if (ai_tmp > 0.0) {
                      bi = 0.5;
                    } else {
                      bi = -0.5;
                    }
                    Troot[i].re = (P_re_tmp * br + r * bi) / brm;
                    Troot[i].im = (r * br - P_re_tmp * bi) / brm;
                  } else {
                    bim = ar_tmp / ai_tmp;
                    bi = ai_tmp + bim * ar_tmp;
                    Troot[i].re = (bim * P_re_tmp + r) / bi;
                    Troot[i].im = (bim * r - P_re_tmp) / bi;
                  }
                }
              } else {
                b_log(b_d[0]);
                b_log(b_d[3]);
                ar = wt * (b_d[0].re + b_d[3].re);
                ai = wt * (b_d[0].im + b_d[3].im);
                if (ai == 0.0) {
                  r22_tmp = ar / 2.0;
                  br = 0.0;
                } else if (ar == 0.0) {
                  r22_tmp = 0.0;
                  br = ai / 2.0;
                } else {
                  r22_tmp = ar / 2.0;
                  br = ai / 2.0;
                }
                if (r22_tmp == 0.0) {
                  r22_tmp = std::cos(br);
                  br = std::sin(br);
                } else if (br == 0.0) {
                  r22_tmp = std::exp(r22_tmp);
                  br = 0.0;
                } else if (std::isinf(br) && std::isinf(r22_tmp) &&
                           (r22_tmp < 0.0)) {
                  r22_tmp = 0.0;
                  br = 0.0;
                } else {
                  r = std::exp(r22_tmp / 2.0);
                  r22_tmp = r * (r * std::cos(br));
                  br = r * (r * std::sin(br));
                }
                b_atanh(r11);
                b_re =
                    std::ceil(((b_d[3].im - b_d[0].im) - 3.1415926535897931) /
                              6.2831853071795862);
                bi = r11.re + 0.0 * b_re;
                r11_re_tmp = r11.im + 3.1415926535897931 * b_re;
                r = wt * bi;
                P_re_tmp = wt * r11_re_tmp;
                if (P_re_tmp == 0.0) {
                  r11.re = std::sinh(r);
                  r11.im = 0.0;
                } else {
                  r11.re = std::sinh(r) * std::cos(P_re_tmp);
                  r11.im = std::cosh(r) * std::sin(P_re_tmp);
                }
                re = 2.0 * r22_tmp;
                im = 2.0 * br;
                b_re = re * r11.re - im * r11.im;
                im = re * r11.im + im * r11.re;
                if (ai_tmp == 0.0) {
                  if (im == 0.0) {
                    re = b_re / ar_tmp;
                    im = 0.0;
                  } else if (b_re == 0.0) {
                    re = 0.0;
                    im /= ar_tmp;
                  } else {
                    re = b_re / ar_tmp;
                    im /= ar_tmp;
                  }
                } else if (ar_tmp == 0.0) {
                  if (b_re == 0.0) {
                    re = im / ai_tmp;
                    im = 0.0;
                  } else if (im == 0.0) {
                    re = 0.0;
                    im = -(b_re / ai_tmp);
                  } else {
                    re = im / ai_tmp;
                    im = -(b_re / ai_tmp);
                  }
                } else {
                  brm = std::abs(ar_tmp);
                  bim = std::abs(ai_tmp);
                  if (brm > bim) {
                    bim = ai_tmp / ar_tmp;
                    bi = ar_tmp + bim * ai_tmp;
                    re = (b_re + bim * im) / bi;
                    im = (im - bim * b_re) / bi;
                  } else if (bim == brm) {
                    if (ar_tmp > 0.0) {
                      br = 0.5;
                    } else {
                      br = -0.5;
                    }
                    if (ai_tmp > 0.0) {
                      bi = 0.5;
                    } else {
                      bi = -0.5;
                    }
                    re = (b_re * br + im * bi) / brm;
                    im = (im * br - b_re * bi) / brm;
                  } else {
                    bim = ar_tmp / ai_tmp;
                    bi = ai_tmp + bim * ar_tmp;
                    re = (bim * b_re + im) / bi;
                    im = (bim * im - b_re) / bi;
                  }
                }
                Troot[i].re = b_d[2].re * re - b_d[2].im * im;
                Troot[i].im = b_d[2].re * im + b_d[2].im * re;
              }
            }
          }
        }
      }
    }
  }
  std::memset(&L[0], 0, 16U * sizeof(creal_T));
  i = static_cast<unsigned char>(m);
  for (int j{0}; j < i; j++) {
    creal_T K[16];
    creal_T X[16];
    int i3;
    signed char ipiv[4];
    r1 = j + 7 * (m - 1);
    r = dv[r1];
    wt = dv1[r1];
    for (i1 = 0; i1 < 16; i1++) {
      K[i1].re = r * Troot[i1].re;
      K[i1].im = r * Troot[i1].im;
      X[i1] = Troot[i1];
    }
    K[0].re++;
    K[5].re++;
    K[10].re++;
    K[15].re++;
    ipiv[0] = 1;
    ipiv[1] = 2;
    ipiv[2] = 3;
    ipiv[3] = 4;
    for (int b_j{0}; b_j < 3; b_j++) {
      int b_tmp;
      int mmj_tmp;
      mmj_tmp = 2 - b_j;
      b_tmp = b_j * 5;
      jp1j = b_tmp + 2;
      r1 = 5 - b_j;
      jA = 0;
      r = std::abs(K[b_tmp].re) + std::abs(K[b_tmp].im);
      for (int k{2}; k < r1; k++) {
        r2 = (b_tmp + k) - 1;
        bim = std::abs(K[r2].re) + std::abs(K[r2].im);
        if (bim > r) {
          jA = k - 1;
          r = bim;
        }
      }
      r1 = b_tmp + jA;
      if ((K[r1].re != 0.0) || (K[r1].im != 0.0)) {
        if (jA != 0) {
          r1 = b_j + jA;
          ipiv[b_j] = static_cast<signed char>(r1 + 1);
          r11 = K[b_j];
          K[b_j] = K[r1];
          K[r1] = r11;
          r11 = K[b_j + 4];
          K[b_j + 4] = K[r1 + 4];
          K[r1 + 4] = r11;
          r11 = K[b_j + 8];
          K[b_j + 8] = K[r1 + 8];
          K[r1 + 8] = r11;
          r11 = K[b_j + 12];
          K[b_j + 12] = K[r1 + 12];
          K[r1 + 12] = r11;
        }
        i1 = (b_tmp - b_j) + 4;
        for (int b_i{jp1j}; b_i <= i1; b_i++) {
          ar = K[b_i - 1].re;
          ai = K[b_i - 1].im;
          br = K[b_tmp].re;
          bi = K[b_tmp].im;
          if (bi == 0.0) {
            if (ai == 0.0) {
              re = ar / br;
              im = 0.0;
            } else if (ar == 0.0) {
              re = 0.0;
              im = ai / br;
            } else {
              re = ar / br;
              im = ai / br;
            }
          } else if (br == 0.0) {
            if (ar == 0.0) {
              re = ai / bi;
              im = 0.0;
            } else if (ai == 0.0) {
              re = 0.0;
              im = -(ar / bi);
            } else {
              re = ai / bi;
              im = -(ar / bi);
            }
          } else {
            brm = std::abs(br);
            bim = std::abs(bi);
            if (brm > bim) {
              bim = bi / br;
              bi = br + bim * bi;
              re = (ar + bim * ai) / bi;
              im = (ai - bim * ar) / bi;
            } else if (bim == brm) {
              if (br > 0.0) {
                br = 0.5;
              } else {
                br = -0.5;
              }
              if (bi > 0.0) {
                bi = 0.5;
              } else {
                bi = -0.5;
              }
              re = (ar * br + ai * bi) / brm;
              im = (ai * br - ar * bi) / brm;
            } else {
              bim = br / bi;
              bi += bim * br;
              re = (bim * ar + ai) / bi;
              im = (bim * ai - ar) / bi;
            }
          }
          K[b_i - 1].re = re;
          K[b_i - 1].im = im;
        }
      }
      jA = b_tmp;
      for (jp1j = 0; jp1j <= mmj_tmp; jp1j++) {
        r1 = (b_tmp + (jp1j << 2)) + 4;
        r = K[r1].re;
        P_re_tmp = K[r1].im;
        if ((r != 0.0) || (P_re_tmp != 0.0)) {
          r11.re = -r - P_re_tmp * 0.0;
          r11.im = r * 0.0 - P_re_tmp;
          i1 = jA + 6;
          i3 = (jA - b_j) + 8;
          for (r1 = i1; r1 <= i3; r1++) {
            r2 = ((b_tmp + r1) - jA) - 5;
            r = K[r2].re;
            P_re_tmp = K[r2].im;
            K[r1 - 1].re += r * r11.re - P_re_tmp * r11.im;
            K[r1 - 1].im += r * r11.im + P_re_tmp * r11.re;
          }
        }
        jA += 4;
      }
    }
    for (int b_i{0}; b_i < 3; b_i++) {
      signed char i2;
      i2 = ipiv[b_i];
      if (i2 != b_i + 1) {
        r11 = X[b_i];
        X[b_i] = X[i2 - 1];
        X[i2 - 1] = r11;
        r11 = X[b_i + 4];
        X[b_i + 4] = X[i2 + 3];
        X[i2 + 3] = r11;
        r11 = X[b_i + 8];
        X[b_i + 8] = X[i2 + 7];
        X[i2 + 7] = r11;
        r11 = X[b_i + 12];
        X[b_i + 12] = X[i2 + 11];
        X[i2 + 11] = r11;
      }
    }
    for (int b_j{0}; b_j < 4; b_j++) {
      r2 = b_j << 2;
      for (int k{0}; k < 4; k++) {
        jp1j = k << 2;
        jA = k + r2;
        if ((X[jA].re != 0.0) || (X[jA].im != 0.0)) {
          i1 = k + 2;
          for (int b_i{i1}; b_i < 5; b_i++) {
            r1 = (b_i + jp1j) - 1;
            r = X[jA].re;
            P_re_tmp = K[r1].im;
            bi = X[jA].im;
            r11_re_tmp = K[r1].re;
            i3 = (b_i + r2) - 1;
            X[i3].re -= r * r11_re_tmp - bi * P_re_tmp;
            X[i3].im -= r * P_re_tmp + bi * r11_re_tmp;
          }
        }
      }
    }
    for (int b_j{0}; b_j < 4; b_j++) {
      r2 = b_j << 2;
      for (int k{3}; k >= 0; k--) {
        jp1j = k << 2;
        jA = k + r2;
        r = X[jA].re;
        P_re_tmp = X[jA].im;
        if ((r != 0.0) || (P_re_tmp != 0.0)) {
          r1 = k + jp1j;
          br = K[r1].re;
          bi = K[r1].im;
          if (bi == 0.0) {
            if (P_re_tmp == 0.0) {
              re = r / br;
              im = 0.0;
            } else if (r == 0.0) {
              re = 0.0;
              im = P_re_tmp / br;
            } else {
              re = r / br;
              im = P_re_tmp / br;
            }
          } else if (br == 0.0) {
            if (r == 0.0) {
              re = P_re_tmp / bi;
              im = 0.0;
            } else if (P_re_tmp == 0.0) {
              re = 0.0;
              im = -(r / bi);
            } else {
              re = P_re_tmp / bi;
              im = -(r / bi);
            }
          } else {
            brm = std::abs(br);
            bim = std::abs(bi);
            if (brm > bim) {
              bim = bi / br;
              bi = br + bim * bi;
              re = (r + bim * P_re_tmp) / bi;
              im = (P_re_tmp - bim * r) / bi;
            } else if (bim == brm) {
              if (br > 0.0) {
                br = 0.5;
              } else {
                br = -0.5;
              }
              if (bi > 0.0) {
                bi = 0.5;
              } else {
                bi = -0.5;
              }
              re = (r * br + P_re_tmp * bi) / brm;
              im = (P_re_tmp * br - r * bi) / brm;
            } else {
              bim = br / bi;
              bi += bim * br;
              re = (bim * r + P_re_tmp) / bi;
              im = (bim * P_re_tmp - r) / bi;
            }
          }
          X[jA].re = re;
          X[jA].im = im;
          for (int b_i{0}; b_i < k; b_i++) {
            r1 = b_i + jp1j;
            r = X[jA].re;
            P_re_tmp = K[r1].im;
            bi = X[jA].im;
            r11_re_tmp = K[r1].re;
            i1 = b_i + r2;
            X[i1].re -= r * r11_re_tmp - bi * P_re_tmp;
            X[i1].im -= r * P_re_tmp + bi * r11_re_tmp;
          }
        }
      }
    }
    for (i1 = 0; i1 < 16; i1++) {
      L[i1].re += wt * X[i1].re;
      L[i1].im += wt * X[i1].im;
    }
  }
  r = rt_powd_snf(2.0, static_cast<double>(s));
  for (i = 0; i < 16; i++) {
    L[i].re *= r;
    L[i].im *= r;
  }
  for (int j{0}; j < 3; j++) {
    r2 = j + (j << 2);
    b_d[0] = T[r2];
    b_log(b_d[0]);
    r1 = j + ((j + 1) << 2);
    b_d[3] = T[r1 + 1];
    b_log(b_d[3]);
    L[r2] = b_d[0];
    L[r1 + 1] = b_d[3];
    if (((!(T[r2].re < 0.0)) || (!(T[r2].im == 0.0))) &&
        ((!(T[r1 + 1].re < 0.0)) || (!(T[r2].im == 0.0)))) {
      bi = T[r1 + 1].re;
      r = T[r2].re;
      r11_re_tmp = T[r1 + 1].im;
      r22_tmp = T[r2].im;
      if ((r == bi) && (r22_tmp == r11_re_tmp)) {
        ar = T[r1].re;
        ai = T[r1].im;
        if (r22_tmp == 0.0) {
          if (ai == 0.0) {
            L[r1].re = ar / r;
            L[r1].im = 0.0;
          } else if (ar == 0.0) {
            L[r1].re = 0.0;
            L[r1].im = ai / r;
          } else {
            L[r1].re = ar / r;
            L[r1].im = ai / r;
          }
        } else if (r == 0.0) {
          if (ar == 0.0) {
            L[r1].re = ai / r22_tmp;
            L[r1].im = 0.0;
          } else if (ai == 0.0) {
            L[r1].re = 0.0;
            L[r1].im = -(ar / r22_tmp);
          } else {
            L[r1].re = ai / r22_tmp;
            L[r1].im = -(ar / r22_tmp);
          }
        } else {
          brm = std::abs(r);
          bim = std::abs(r22_tmp);
          if (brm > bim) {
            bim = r22_tmp / r;
            bi = r + bim * r22_tmp;
            L[r1].re = (ar + bim * ai) / bi;
            L[r1].im = (ai - bim * ar) / bi;
          } else if (bim == brm) {
            if (r > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (r22_tmp > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            L[r1].re = (ar * br + ai * bi) / brm;
            L[r1].im = (ai * br - ar * bi) / brm;
          } else {
            bim = r / r22_tmp;
            bi = r22_tmp + bim * r;
            L[r1].re = (bim * ar + ai) / bi;
            L[r1].im = (bim * ai - ar) / bi;
          }
        }
      } else {
        ar_tmp = bi - r;
        ai_tmp = r11_re_tmp - r22_tmp;
        br = bi + r;
        bi = r11_re_tmp + r22_tmp;
        if (bi == 0.0) {
          if (ai_tmp == 0.0) {
            r11.re = ar_tmp / br;
            r11.im = 0.0;
          } else if (ar_tmp == 0.0) {
            r11.re = 0.0;
            r11.im = ai_tmp / br;
          } else {
            r11.re = ar_tmp / br;
            r11.im = ai_tmp / br;
          }
        } else if (br == 0.0) {
          if (ar_tmp == 0.0) {
            r11.re = ai_tmp / bi;
            r11.im = 0.0;
          } else if (ai_tmp == 0.0) {
            r11.re = 0.0;
            r11.im = -(ar_tmp / bi);
          } else {
            r11.re = ai_tmp / bi;
            r11.im = -(ar_tmp / bi);
          }
        } else {
          brm = std::abs(br);
          bim = std::abs(bi);
          if (brm > bim) {
            bim = bi / br;
            bi = br + bim * bi;
            r11.re = (ar_tmp + bim * ai_tmp) / bi;
            r11.im = (ai_tmp - bim * ar_tmp) / bi;
          } else if (bim == brm) {
            if (br > 0.0) {
              br = 0.5;
            } else {
              br = -0.5;
            }
            if (bi > 0.0) {
              bi = 0.5;
            } else {
              bi = -0.5;
            }
            r11.re = (ar_tmp * br + ai_tmp * bi) / brm;
            r11.im = (ai_tmp * br - ar_tmp * bi) / brm;
          } else {
            bim = br / bi;
            bi += bim * br;
            r11.re = (bim * ar_tmp + ai_tmp) / bi;
            r11.im = (bim * ai_tmp - ar_tmp) / bi;
          }
        }
        if (checkCondition(r11)) {
          bi = b_d[3].re - b_d[0].re;
          r11_re_tmp = b_d[3].im - b_d[0].im;
          r = T[r1].re;
          P_re_tmp = T[r1].im;
          r22_tmp = r * bi - P_re_tmp * r11_re_tmp;
          r = r * r11_re_tmp + P_re_tmp * bi;
          if (ai_tmp == 0.0) {
            if (r == 0.0) {
              L[r1].re = r22_tmp / ar_tmp;
              L[r1].im = 0.0;
            } else if (r22_tmp == 0.0) {
              L[r1].re = 0.0;
              L[r1].im = r / ar_tmp;
            } else {
              L[r1].re = r22_tmp / ar_tmp;
              L[r1].im = r / ar_tmp;
            }
          } else if (ar_tmp == 0.0) {
            if (r22_tmp == 0.0) {
              L[r1].re = r / ai_tmp;
              L[r1].im = 0.0;
            } else if (r == 0.0) {
              L[r1].re = 0.0;
              L[r1].im = -(r22_tmp / ai_tmp);
            } else {
              L[r1].re = r / ai_tmp;
              L[r1].im = -(r22_tmp / ai_tmp);
            }
          } else {
            brm = std::abs(ar_tmp);
            bim = std::abs(ai_tmp);
            if (brm > bim) {
              bim = ai_tmp / ar_tmp;
              bi = ar_tmp + bim * ai_tmp;
              L[r1].re = (r22_tmp + bim * r) / bi;
              L[r1].im = (r - bim * r22_tmp) / bi;
            } else if (bim == brm) {
              if (ar_tmp > 0.0) {
                br = 0.5;
              } else {
                br = -0.5;
              }
              if (ai_tmp > 0.0) {
                bi = 0.5;
              } else {
                bi = -0.5;
              }
              L[r1].re = (r22_tmp * br + r * bi) / brm;
              L[r1].im = (r * br - r22_tmp * bi) / brm;
            } else {
              bim = ar_tmp / ai_tmp;
              bi = ai_tmp + bim * ar_tmp;
              L[r1].re = (bim * r22_tmp + r) / bi;
              L[r1].im = (bim * r - r22_tmp) / bi;
            }
          }
        } else {
          b_atanh(r11);
          r = std::ceil(((b_d[3].im - b_d[0].im) - 3.1415926535897931) /
                        6.2831853071795862);
          re = 2.0 * r11.re + 0.0 * r;
          im = 2.0 * r11.im + 6.2831853071795862 * r;
          r = T[r1].re;
          P_re_tmp = T[r1].im;
          r22_tmp = r * re - P_re_tmp * im;
          r = r * im + P_re_tmp * re;
          if (ai_tmp == 0.0) {
            if (r == 0.0) {
              L[r1].re = r22_tmp / ar_tmp;
              L[r1].im = 0.0;
            } else if (r22_tmp == 0.0) {
              L[r1].re = 0.0;
              L[r1].im = r / ar_tmp;
            } else {
              L[r1].re = r22_tmp / ar_tmp;
              L[r1].im = r / ar_tmp;
            }
          } else if (ar_tmp == 0.0) {
            if (r22_tmp == 0.0) {
              L[r1].re = r / ai_tmp;
              L[r1].im = 0.0;
            } else if (r == 0.0) {
              L[r1].re = 0.0;
              L[r1].im = -(r22_tmp / ai_tmp);
            } else {
              L[r1].re = r / ai_tmp;
              L[r1].im = -(r22_tmp / ai_tmp);
            }
          } else {
            brm = std::abs(ar_tmp);
            bim = std::abs(ai_tmp);
            if (brm > bim) {
              bim = ai_tmp / ar_tmp;
              bi = ar_tmp + bim * ai_tmp;
              L[r1].re = (r22_tmp + bim * r) / bi;
              L[r1].im = (r - bim * r22_tmp) / bi;
            } else if (bim == brm) {
              if (ar_tmp > 0.0) {
                br = 0.5;
              } else {
                br = -0.5;
              }
              if (ai_tmp > 0.0) {
                bi = 0.5;
              } else {
                bi = -0.5;
              }
              L[r1].re = (r22_tmp * br + r * bi) / brm;
              L[r1].im = (r * br - r22_tmp * bi) / brm;
            } else {
              bim = ar_tmp / ai_tmp;
              bi = ai_tmp + bim * ar_tmp;
              L[r1].re = (bim * r22_tmp + r) / bi;
              L[r1].im = (bim * r - r22_tmp) / bi;
            }
          }
        }
      }
    }
  }
  return exitflag;
}

static int computeLogOfSchurForm(const double T[16], const creal_T d[4],
                                 double L[16])
{
  __m128d b_r1;
  __m128d r;
  creal_T b_d[4];
  double K[16];
  double Troot[16];
  double X[16];
  double P[4];
  double Z0[4];
  double b_Troot[4];
  double a21;
  double f;
  double loga1;
  double loga2;
  double wt;
  int Troot_tmp;
  int Troot_tmp_tmp;
  int exitflag;
  int i1;
  int i2;
  int j;
  int kAcol;
  int lastBlock;
  int m;
  int r1;
  int r2;
  int s;
  signed char blockStruct[3];
  std::copy(&T[0], &T[16], &Troot[0]);
  std::copy(&d[0], &d[4], &b_d[0]);
  s = logmParams(Troot, b_d, m, exitflag);
  blockStruct[0] = 0;
  blockStruct[1] = 0;
  blockStruct[2] = 0;
  j = 0;
  while (j + 1 < 3) {
    if (T[(j + (j << 2)) + 1] != 0.0) {
      blockStruct[j] = 2;
      blockStruct[j + 1] = 0;
      j += 2;
    } else if (T[(j + ((j + 1) << 2)) + 2] == 0.0) {
      blockStruct[j] = 1;
      j++;
    } else {
      blockStruct[j] = 0;
      j++;
    }
  }
  if (T[11] != 0.0) {
    blockStruct[2] = 2;
  } else if ((blockStruct[1] == 0) || (blockStruct[1] == 1)) {
    blockStruct[2] = 1;
  }
  lastBlock = 0;
  for (j = 0; j < 3; j++) {
    signed char i;
    i = blockStruct[j];
    if (i == 0) {
      if (lastBlock != 0) {
        lastBlock = 0;
      } else {
        kAcol = j + (j << 2);
        Troot[kAcol] = sqrtObo(T[kAcol], s);
      }
    } else {
      double A[4];
      double A_tmp;
      double b_A_tmp;
      double c_A_tmp;
      lastBlock = i;
      Troot_tmp_tmp = j << 2;
      kAcol = j + Troot_tmp_tmp;
      b_Troot[0] = Troot[kAcol];
      A_tmp = T[kAcol];
      A[0] = A_tmp;
      Troot_tmp = (j + Troot_tmp_tmp) + 1;
      b_Troot[1] = Troot[Troot_tmp];
      loga2 = T[Troot_tmp];
      A[1] = loga2;
      Troot_tmp_tmp = j + ((j + 1) << 2);
      b_Troot[2] = Troot[Troot_tmp_tmp];
      b_A_tmp = T[Troot_tmp_tmp];
      A[2] = b_A_tmp;
      b_Troot[3] = Troot[Troot_tmp_tmp + 1];
      c_A_tmp = T[Troot_tmp_tmp + 1];
      A[3] = c_A_tmp;
      if (s == 0) {
        b_Troot[0] = A_tmp - 1.0;
        b_Troot[1] = loga2;
        b_Troot[2] = b_A_tmp;
        b_Troot[3] = c_A_tmp - 1.0;
      } else {
        sqrtm2by2(A);
        Z0[0] = A[0] - 1.0;
        Z0[1] = A[1];
        Z0[2] = A[2];
        Z0[3] = A[3] - 1.0;
        if (s == 1) {
          b_Troot[0] = A[0] - 1.0;
          b_Troot[1] = A[1];
          b_Troot[2] = A[2];
          b_Troot[3] = A[3] - 1.0;
        } else {
          sqrtm2by2(A);
          P[0] = A[0] + 1.0;
          P[1] = A[1];
          P[2] = A[2];
          P[3] = A[3] + 1.0;
          i1 = static_cast<unsigned char>(s - 2);
          for (int b_i{0}; b_i < i1; b_i++) {
            __m128d b_r2;
            __m128d r3;
            sqrtm2by2(A);
            wt = A[0] + 1.0;
            a21 = A[1];
            f = A[2];
            loga1 = A[3] + 1.0;
            r = _mm_loadu_pd(&P[0]);
            b_r1 = _mm_mul_pd(r, _mm_set1_pd(wt));
            b_r2 = _mm_loadu_pd(&P[2]);
            r3 = _mm_mul_pd(b_r2, _mm_set1_pd(a21));
            b_r1 = _mm_add_pd(b_r1, r3);
            _mm_storeu_pd(&P[0], b_r1);
            r = _mm_mul_pd(r, _mm_set1_pd(f));
            b_r1 = _mm_mul_pd(b_r2, _mm_set1_pd(loga1));
            r = _mm_add_pd(r, b_r1);
            _mm_storeu_pd(&P[2], r);
          }
          if (std::abs(P[1]) > std::abs(P[0])) {
            r1 = 1;
            r2 = 0;
          } else {
            r1 = 0;
            r2 = 1;
          }
          a21 = P[r2] / P[r1];
          r = _mm_loadu_pd(&Z0[0]);
          i1 = r1 << 1;
          _mm_storeu_pd(&b_Troot[i1], _mm_div_pd(r, _mm_set1_pd(P[r1])));
          r = _mm_loadu_pd(&b_Troot[i1]);
          b_r1 = _mm_loadu_pd(&Z0[2]);
          f = P[r1 + 2];
          i2 = r2 << 1;
          _mm_storeu_pd(
              &b_Troot[i2],
              _mm_div_pd(_mm_sub_pd(b_r1, _mm_mul_pd(r, _mm_set1_pd(f))),
                         _mm_set1_pd(P[r2 + 2] - a21 * f)));
          r = _mm_loadu_pd(&b_Troot[i2]);
          b_r1 = _mm_loadu_pd(&b_Troot[i1]);
          _mm_storeu_pd(&b_Troot[i1],
                        _mm_sub_pd(b_r1, _mm_mul_pd(r, _mm_set1_pd(a21))));
          if ((loga2 == 0.0) && (A_tmp >= 0.0) && (c_A_tmp >= 0.0)) {
            wt = 1.0 / rt_powd_snf(2.0, static_cast<double>(s));
            if (A_tmp == c_A_tmp) {
              b_Troot[2] = wt * b_A_tmp * rt_powd_snf(A_tmp, wt - 1.0);
            } else {
              f = c_A_tmp - A_tmp;
              a21 = f / (A_tmp + c_A_tmp);
              if ((std::abs(a21) > 90.509667991878089) ||
                  (std::abs(a21 - 1.0) < 0.011048543456039804) ||
                  (std::abs(a21 + 1.0) < 0.011048543456039804)) {
                b_Troot[2] =
                    b_A_tmp *
                    (rt_powd_snf(c_A_tmp, wt) - rt_powd_snf(A_tmp, wt)) / f;
              } else {
                b_atanh(a21);
                b_Troot[2] =
                    b_A_tmp *
                    (2.0 *
                     std::exp(wt * (std::log(A_tmp) + std::log(c_A_tmp)) /
                              2.0) *
                     std::sinh(wt * a21) / f);
              }
            }
          }
        }
      }
      Troot[kAcol] = b_Troot[0];
      Troot[Troot_tmp] = b_Troot[1];
      Troot[Troot_tmp_tmp] = b_Troot[2];
      Troot[Troot_tmp_tmp + 1] = b_Troot[3];
    }
  }
  if (blockStruct[2] == 0) {
    Troot[15] = sqrtObo(T[15], s);
  }
  std::memset(&L[0], 0, 16U * sizeof(double));
  i1 = static_cast<unsigned char>(m);
  for (j = 0; j < i1; j++) {
    int ipiv[4];
    r1 = j + 7 * (m - 1);
    a21 = dv[r1];
    wt = dv1[r1];
    for (i2 = 0; i2 <= 14; i2 += 2) {
      r = _mm_loadu_pd(&Troot[i2]);
      _mm_storeu_pd(&K[i2], _mm_mul_pd(_mm_set1_pd(a21), r));
    }
    K[0]++;
    K[5]++;
    K[10]++;
    K[15]++;
    std::copy(&Troot[0], &Troot[16], &X[0]);
    internal::lapack::xgetrf(K, ipiv);
    for (int b_i{0}; b_i < 3; b_i++) {
      i2 = ipiv[b_i];
      if (i2 != b_i + 1) {
        a21 = X[b_i];
        X[b_i] = X[i2 - 1];
        X[i2 - 1] = a21;
        a21 = X[b_i + 4];
        X[b_i + 4] = X[i2 + 3];
        X[i2 + 3] = a21;
        a21 = X[b_i + 8];
        X[b_i + 8] = X[i2 + 7];
        X[i2 + 7] = a21;
        a21 = X[b_i + 12];
        X[b_i + 12] = X[i2 + 11];
        X[i2 + 11] = a21;
      }
    }
    for (r1 = 0; r1 < 4; r1++) {
      r2 = r1 << 2;
      for (Troot_tmp_tmp = 0; Troot_tmp_tmp < 4; Troot_tmp_tmp++) {
        kAcol = Troot_tmp_tmp << 2;
        i2 = Troot_tmp_tmp + r2;
        if (X[i2] != 0.0) {
          Troot_tmp = Troot_tmp_tmp + 2;
          for (int b_i{Troot_tmp}; b_i < 5; b_i++) {
            lastBlock = (b_i + r2) - 1;
            X[lastBlock] -= X[i2] * K[(b_i + kAcol) - 1];
          }
        }
      }
    }
    for (r1 = 0; r1 < 4; r1++) {
      r2 = r1 << 2;
      for (Troot_tmp_tmp = 3; Troot_tmp_tmp >= 0; Troot_tmp_tmp--) {
        kAcol = Troot_tmp_tmp << 2;
        i2 = Troot_tmp_tmp + r2;
        f = X[i2];
        if (f != 0.0) {
          X[i2] = f / K[Troot_tmp_tmp + kAcol];
          for (int b_i{0}; b_i < Troot_tmp_tmp; b_i++) {
            lastBlock = b_i + r2;
            X[lastBlock] -= X[i2] * K[b_i + kAcol];
          }
        }
      }
    }
    for (i2 = 0; i2 <= 14; i2 += 2) {
      r = _mm_loadu_pd(&X[i2]);
      b_r1 = _mm_loadu_pd(&L[i2]);
      _mm_storeu_pd(&L[i2], _mm_add_pd(b_r1, _mm_mul_pd(_mm_set1_pd(wt), r)));
    }
  }
  a21 = rt_powd_snf(2.0, static_cast<double>(s));
  for (i1 = 0; i1 <= 14; i1 += 2) {
    r = _mm_loadu_pd(&L[i1]);
    _mm_storeu_pd(&L[i1], _mm_mul_pd(_mm_set1_pd(a21), r));
  }
  lastBlock = 0;
  for (j = 0; j < 3; j++) {
    switch (blockStruct[j]) {
    case 0:
      if (lastBlock != 0) {
        lastBlock = 0;
      } else {
        r1 = j + (j << 2);
        L[r1] = std::log(T[r1]);
      }
      break;
    case 1:
      lastBlock = 1;
      kAcol = j << 2;
      Troot_tmp = j + kAcol;
      kAcol = (j + kAcol) + 1;
      b_Troot[1] = L[kAcol];
      r1 = (j + 1) << 2;
      r2 = j + r1;
      b_Troot[2] = L[r2];
      r1 = (j + r1) + 1;
      a21 = T[Troot_tmp];
      loga1 = std::log(a21);
      wt = T[r1];
      loga2 = std::log(wt);
      if ((!(a21 < 0.0)) && (!(wt < 0.0))) {
        if (a21 == wt) {
          b_Troot[2] = T[r2] / a21;
        } else {
          f = wt - a21;
          a21 = f / (a21 + wt);
          if ((std::abs(a21) > 90.509667991878089) ||
              (std::abs(a21 - 1.0) < 0.011048543456039804) ||
              (std::abs(a21 + 1.0) < 0.011048543456039804)) {
            b_Troot[2] = T[r2] * (loga2 - loga1) / f;
          } else {
            b_atanh(a21);
            b_Troot[2] = T[r2] * (2.0 * a21) / f;
          }
        }
      }
      L[Troot_tmp] = loga1;
      L[kAcol] = b_Troot[1];
      L[r2] = b_Troot[2];
      L[r1] = loga2;
      break;
    default:
      lastBlock = 2;
      r1 = j + (j << 2);
      a21 = T[r1];
      r2 = j + ((j + 1) << 2);
      loga1 = T[r2];
      loga2 = T[r1 + 1];
      f = 0.5 * std::log(a21 * a21 - loga1 * loga2);
      wt = std::sqrt(-loga1 * loga2);
      a21 = rt_atan2d_snf(wt, a21) / wt;
      L[r1] = f;
      L[r1 + 1] = a21 * loga2;
      L[r2] = a21 * loga1;
      L[r2 + 1] = f;
      break;
    }
  }
  if (blockStruct[2] == 0) {
    L[15] = std::log(T[15]);
  }
  return exitflag;
}

static int logmParams(creal_T T[16], creal_T d[4], int &m, int &exitflag)
{
  creal_T TrootmI[16];
  creal_T b_TrootmI[16];
  creal_T c_tmp[16];
  cuint8_T TrootmI_tmp[16];
  double TrootmI_re_tmp;
  double a2;
  double b_TrootmI_re_tmp;
  double c_TrootmI_re_tmp;
  double d3;
  double d_TrootmI_re_tmp;
  double e_TrootmI_re_tmp;
  double f_TrootmI_re_tmp;
  double g_TrootmI_re_tmp;
  double h_TrootmI_re_tmp;
  double i_TrootmI_re_tmp;
  double j_TrootmI_re_tmp;
  double k_TrootmI_re_tmp;
  double l_TrootmI_re_tmp;
  double m_TrootmI_re_tmp;
  double n_TrootmI_re_tmp;
  double o_TrootmI_re_tmp;
  int ind;
  int j;
  int k;
  int s;
  int s0;
  signed char b_I[16];
  bool exitg2;
  bool foundm;
  m = 1;
  exitflag = 0;
  for (j = 0; j < 16; j++) {
    b_I[j] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  s = 0;
  creal_T b_d[4];
  int exitg1;
  do {
    exitg1 = 0;
    b_d[0].re = d[0].re - 1.0;
    b_d[0].im = d[0].im;
    b_d[1].re = d[1].re - 1.0;
    b_d[1].im = d[1].im;
    b_d[2].re = d[2].re - 1.0;
    b_d[2].im = d[2].im;
    b_d[3].re = d[3].re - 1.0;
    b_d[3].im = d[3].im;
    if ((b_norm(b_d) > 0.28790937142411938) && (s < 100)) {
      internal::scalar::b_sqrt(d[0]);
      internal::scalar::b_sqrt(d[1]);
      internal::scalar::b_sqrt(d[2]);
      internal::scalar::b_sqrt(d[3]);
      s++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  s0 = s;
  if (s == 100) {
    exitflag = 1;
  }
  for (k = 0; k < s; k++) {
    sqrtmTriRecursive(T, 1, 1, 4);
  }
  for (j = 0; j < 16; j++) {
    unsigned char re;
    re = static_cast<unsigned char>(b_I[j]);
    TrootmI_tmp[j].re = re;
    TrootmI_tmp[j].im = 0U;
    TrootmI[j].re = T[j].re - static_cast<double>(re);
    TrootmI[j].im = T[j].im;
  }
  for (j = 0; j < 4; j++) {
    a2 = TrootmI[j].re;
    TrootmI_re_tmp = TrootmI[j].im;
    b_TrootmI_re_tmp = TrootmI[j + 4].re;
    c_TrootmI_re_tmp = TrootmI[j + 4].im;
    d_TrootmI_re_tmp = TrootmI[j + 8].re;
    e_TrootmI_re_tmp = TrootmI[j + 8].im;
    f_TrootmI_re_tmp = TrootmI[j + 12].re;
    g_TrootmI_re_tmp = TrootmI[j + 12].im;
    for (int i{0}; i < 4; i++) {
      ind = i << 2;
      h_TrootmI_re_tmp = TrootmI[ind].im;
      i_TrootmI_re_tmp = TrootmI[ind].re;
      j_TrootmI_re_tmp = TrootmI[ind + 1].im;
      k_TrootmI_re_tmp = TrootmI[ind + 1].re;
      l_TrootmI_re_tmp = TrootmI[ind + 2].im;
      m_TrootmI_re_tmp = TrootmI[ind + 2].re;
      n_TrootmI_re_tmp = TrootmI[ind + 3].im;
      o_TrootmI_re_tmp = TrootmI[ind + 3].re;
      ind += j;
      c_tmp[ind].re =
          (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
            (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
             c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
           (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
            e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
          (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
           g_TrootmI_re_tmp * n_TrootmI_re_tmp);
      c_tmp[ind].im =
          (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
            (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
             c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
           (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
            e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
          (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
           g_TrootmI_re_tmp * o_TrootmI_re_tmp);
    }
  }
  for (j = 0; j < 4; j++) {
    a2 = TrootmI[j].re;
    TrootmI_re_tmp = TrootmI[j].im;
    b_TrootmI_re_tmp = TrootmI[j + 4].re;
    c_TrootmI_re_tmp = TrootmI[j + 4].im;
    d_TrootmI_re_tmp = TrootmI[j + 8].re;
    e_TrootmI_re_tmp = TrootmI[j + 8].im;
    f_TrootmI_re_tmp = TrootmI[j + 12].re;
    g_TrootmI_re_tmp = TrootmI[j + 12].im;
    for (int i{0}; i < 4; i++) {
      ind = i << 2;
      h_TrootmI_re_tmp = c_tmp[ind].im;
      i_TrootmI_re_tmp = c_tmp[ind].re;
      j_TrootmI_re_tmp = c_tmp[ind + 1].im;
      k_TrootmI_re_tmp = c_tmp[ind + 1].re;
      l_TrootmI_re_tmp = c_tmp[ind + 2].im;
      m_TrootmI_re_tmp = c_tmp[ind + 2].re;
      n_TrootmI_re_tmp = c_tmp[ind + 3].im;
      o_TrootmI_re_tmp = c_tmp[ind + 3].re;
      ind += j;
      b_TrootmI[ind].re =
          (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
            (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
             c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
           (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
            e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
          (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
           g_TrootmI_re_tmp * n_TrootmI_re_tmp);
      b_TrootmI[ind].im =
          (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
            (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
             c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
           (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
            e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
          (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
           g_TrootmI_re_tmp * o_TrootmI_re_tmp);
    }
  }
  d3 = rt_powd_snf(c_norm(b_TrootmI), 0.33333333333333331);
  a2 = std::fmax(std::sqrt(c_norm(c_tmp)), d3);
  if (a2 <= 1.5869707387720632E-5) {
    foundm = true;
  } else if (a2 <= 0.0023138078842429789) {
    m = 2;
    foundm = true;
  } else {
    foundm = false;
  }
  k = 0;
  exitg2 = false;
  while ((!exitg2) && (!foundm)) {
    creal_T b_c_tmp[16];
    double a3;
    double d4;
    bool guard1;
    bool guard2;
    bool more;
    more = false;
    if (s > s0) {
      for (j = 0; j < 4; j++) {
        a2 = TrootmI[j].re;
        TrootmI_re_tmp = TrootmI[j].im;
        b_TrootmI_re_tmp = TrootmI[j + 4].re;
        c_TrootmI_re_tmp = TrootmI[j + 4].im;
        d_TrootmI_re_tmp = TrootmI[j + 8].re;
        e_TrootmI_re_tmp = TrootmI[j + 8].im;
        f_TrootmI_re_tmp = TrootmI[j + 12].re;
        g_TrootmI_re_tmp = TrootmI[j + 12].im;
        for (int i{0}; i < 4; i++) {
          ind = i << 2;
          h_TrootmI_re_tmp = TrootmI[ind].im;
          i_TrootmI_re_tmp = TrootmI[ind].re;
          j_TrootmI_re_tmp = TrootmI[ind + 1].im;
          k_TrootmI_re_tmp = TrootmI[ind + 1].re;
          l_TrootmI_re_tmp = TrootmI[ind + 2].im;
          m_TrootmI_re_tmp = TrootmI[ind + 2].re;
          n_TrootmI_re_tmp = TrootmI[ind + 3].im;
          o_TrootmI_re_tmp = TrootmI[ind + 3].re;
          ind += j;
          b_TrootmI[ind].re =
              (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
                (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
                 c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
               (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
                e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
              (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
               g_TrootmI_re_tmp * n_TrootmI_re_tmp);
          b_TrootmI[ind].im =
              (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
                (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
                 c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
               (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
                e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
              (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
               g_TrootmI_re_tmp * o_TrootmI_re_tmp);
        }
      }
      for (j = 0; j < 4; j++) {
        a2 = TrootmI[j].re;
        TrootmI_re_tmp = TrootmI[j].im;
        b_TrootmI_re_tmp = TrootmI[j + 4].re;
        c_TrootmI_re_tmp = TrootmI[j + 4].im;
        d_TrootmI_re_tmp = TrootmI[j + 8].re;
        e_TrootmI_re_tmp = TrootmI[j + 8].im;
        f_TrootmI_re_tmp = TrootmI[j + 12].re;
        g_TrootmI_re_tmp = TrootmI[j + 12].im;
        for (int i{0}; i < 4; i++) {
          ind = i << 2;
          h_TrootmI_re_tmp = b_TrootmI[ind].im;
          i_TrootmI_re_tmp = b_TrootmI[ind].re;
          j_TrootmI_re_tmp = b_TrootmI[ind + 1].im;
          k_TrootmI_re_tmp = b_TrootmI[ind + 1].re;
          l_TrootmI_re_tmp = b_TrootmI[ind + 2].im;
          m_TrootmI_re_tmp = b_TrootmI[ind + 2].re;
          n_TrootmI_re_tmp = b_TrootmI[ind + 3].im;
          o_TrootmI_re_tmp = b_TrootmI[ind + 3].re;
          ind += j;
          c_tmp[ind].re =
              (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
                (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
                 c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
               (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
                e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
              (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
               g_TrootmI_re_tmp * n_TrootmI_re_tmp);
          c_tmp[ind].im =
              (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
                (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
                 c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
               (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
                e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
              (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
               g_TrootmI_re_tmp * o_TrootmI_re_tmp);
        }
      }
      d3 = rt_powd_snf(c_norm(c_tmp), 0.33333333333333331);
    }
    for (j = 0; j < 4; j++) {
      a2 = TrootmI[j].re;
      TrootmI_re_tmp = TrootmI[j].im;
      b_TrootmI_re_tmp = TrootmI[j + 4].re;
      c_TrootmI_re_tmp = TrootmI[j + 4].im;
      d_TrootmI_re_tmp = TrootmI[j + 8].re;
      e_TrootmI_re_tmp = TrootmI[j + 8].im;
      f_TrootmI_re_tmp = TrootmI[j + 12].re;
      g_TrootmI_re_tmp = TrootmI[j + 12].im;
      for (int i{0}; i < 4; i++) {
        ind = i << 2;
        h_TrootmI_re_tmp = TrootmI[ind].im;
        i_TrootmI_re_tmp = TrootmI[ind].re;
        j_TrootmI_re_tmp = TrootmI[ind + 1].im;
        k_TrootmI_re_tmp = TrootmI[ind + 1].re;
        l_TrootmI_re_tmp = TrootmI[ind + 2].im;
        m_TrootmI_re_tmp = TrootmI[ind + 2].re;
        n_TrootmI_re_tmp = TrootmI[ind + 3].im;
        o_TrootmI_re_tmp = TrootmI[ind + 3].re;
        ind += j;
        c_tmp[ind].re =
            (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
              (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
               c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
             (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
              e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
            (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
             g_TrootmI_re_tmp * n_TrootmI_re_tmp);
        c_tmp[ind].im =
            (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
              (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
               c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
             (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
              e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
            (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
             g_TrootmI_re_tmp * o_TrootmI_re_tmp);
      }
    }
    for (j = 0; j < 4; j++) {
      a2 = c_tmp[j].re;
      TrootmI_re_tmp = c_tmp[j].im;
      b_TrootmI_re_tmp = c_tmp[j + 4].re;
      c_TrootmI_re_tmp = c_tmp[j + 4].im;
      d_TrootmI_re_tmp = c_tmp[j + 8].re;
      e_TrootmI_re_tmp = c_tmp[j + 8].im;
      f_TrootmI_re_tmp = c_tmp[j + 12].re;
      g_TrootmI_re_tmp = c_tmp[j + 12].im;
      for (int i{0}; i < 4; i++) {
        ind = i << 2;
        h_TrootmI_re_tmp = c_tmp[ind].im;
        i_TrootmI_re_tmp = c_tmp[ind].re;
        j_TrootmI_re_tmp = c_tmp[ind + 1].im;
        k_TrootmI_re_tmp = c_tmp[ind + 1].re;
        l_TrootmI_re_tmp = c_tmp[ind + 2].im;
        m_TrootmI_re_tmp = c_tmp[ind + 2].re;
        n_TrootmI_re_tmp = c_tmp[ind + 3].im;
        o_TrootmI_re_tmp = c_tmp[ind + 3].re;
        ind += j;
        b_c_tmp[ind].re =
            (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
              (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
               c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
             (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
              e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
            (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
             g_TrootmI_re_tmp * n_TrootmI_re_tmp);
        b_c_tmp[ind].im =
            (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
              (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
               c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
             (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
              e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
            (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
             g_TrootmI_re_tmp * o_TrootmI_re_tmp);
      }
    }
    d4 = rt_powd_snf(c_norm(b_c_tmp), 0.25);
    a3 = std::fmax(d3, d4);
    guard1 = false;
    guard2 = false;
    if (a3 <= 0.28790937142411938) {
      bool exitg3;
      ind = 0;
      j = 3;
      exitg3 = false;
      while ((!exitg3) && (j < 8)) {
        if (a3 <= dv2[j - 1]) {
          ind = j;
          exitg3 = true;
        } else {
          j++;
        }
      }
      if (ind <= 6) {
        m = ind;
        exitg2 = true;
      } else {
        if ((a3 / 2.0 <= 0.12764048108067749) && (k < 2)) {
          more = true;
          k++;
        }
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
    if (guard2) {
      if (!more) {
        for (j = 0; j < 4; j++) {
          a2 = TrootmI[j].re;
          TrootmI_re_tmp = TrootmI[j].im;
          b_TrootmI_re_tmp = TrootmI[j + 4].re;
          c_TrootmI_re_tmp = TrootmI[j + 4].im;
          d_TrootmI_re_tmp = TrootmI[j + 8].re;
          e_TrootmI_re_tmp = TrootmI[j + 8].im;
          f_TrootmI_re_tmp = TrootmI[j + 12].re;
          g_TrootmI_re_tmp = TrootmI[j + 12].im;
          for (int i{0}; i < 4; i++) {
            ind = i << 2;
            h_TrootmI_re_tmp = TrootmI[ind].im;
            i_TrootmI_re_tmp = TrootmI[ind].re;
            j_TrootmI_re_tmp = TrootmI[ind + 1].im;
            k_TrootmI_re_tmp = TrootmI[ind + 1].re;
            l_TrootmI_re_tmp = TrootmI[ind + 2].im;
            m_TrootmI_re_tmp = TrootmI[ind + 2].re;
            n_TrootmI_re_tmp = TrootmI[ind + 3].im;
            o_TrootmI_re_tmp = TrootmI[ind + 3].re;
            ind += j;
            c_tmp[ind].re =
                (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
                  (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
                   c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
                 (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
                  e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
                (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
                 g_TrootmI_re_tmp * n_TrootmI_re_tmp);
            c_tmp[ind].im =
                (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
                  (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
                   c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
                 (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
                  e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
                (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
                 g_TrootmI_re_tmp * o_TrootmI_re_tmp);
          }
        }
        for (j = 0; j < 4; j++) {
          a2 = c_tmp[j].re;
          TrootmI_re_tmp = c_tmp[j].im;
          b_TrootmI_re_tmp = c_tmp[j + 4].re;
          c_TrootmI_re_tmp = c_tmp[j + 4].im;
          d_TrootmI_re_tmp = c_tmp[j + 8].re;
          e_TrootmI_re_tmp = c_tmp[j + 8].im;
          f_TrootmI_re_tmp = c_tmp[j + 12].re;
          g_TrootmI_re_tmp = c_tmp[j + 12].im;
          for (int i{0}; i < 4; i++) {
            ind = i << 2;
            h_TrootmI_re_tmp = c_tmp[ind].im;
            i_TrootmI_re_tmp = c_tmp[ind].re;
            j_TrootmI_re_tmp = c_tmp[ind + 1].im;
            k_TrootmI_re_tmp = c_tmp[ind + 1].re;
            l_TrootmI_re_tmp = c_tmp[ind + 2].im;
            m_TrootmI_re_tmp = c_tmp[ind + 2].re;
            n_TrootmI_re_tmp = c_tmp[ind + 3].im;
            o_TrootmI_re_tmp = c_tmp[ind + 3].re;
            ind += j;
            b_c_tmp[ind].re =
                (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
                  (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
                   c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
                 (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
                  e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
                (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
                 g_TrootmI_re_tmp * n_TrootmI_re_tmp);
            b_c_tmp[ind].im =
                (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
                  (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
                   c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
                 (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
                  e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
                (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
                 g_TrootmI_re_tmp * o_TrootmI_re_tmp);
          }
        }
        for (j = 0; j < 4; j++) {
          a2 = TrootmI[j].re;
          TrootmI_re_tmp = TrootmI[j].im;
          b_TrootmI_re_tmp = TrootmI[j + 4].re;
          c_TrootmI_re_tmp = TrootmI[j + 4].im;
          d_TrootmI_re_tmp = TrootmI[j + 8].re;
          e_TrootmI_re_tmp = TrootmI[j + 8].im;
          f_TrootmI_re_tmp = TrootmI[j + 12].re;
          g_TrootmI_re_tmp = TrootmI[j + 12].im;
          for (int i{0}; i < 4; i++) {
            ind = i << 2;
            h_TrootmI_re_tmp = b_c_tmp[ind].im;
            i_TrootmI_re_tmp = b_c_tmp[ind].re;
            j_TrootmI_re_tmp = b_c_tmp[ind + 1].im;
            k_TrootmI_re_tmp = b_c_tmp[ind + 1].re;
            l_TrootmI_re_tmp = b_c_tmp[ind + 2].im;
            m_TrootmI_re_tmp = b_c_tmp[ind + 2].re;
            n_TrootmI_re_tmp = b_c_tmp[ind + 3].im;
            o_TrootmI_re_tmp = b_c_tmp[ind + 3].re;
            ind += j;
            b_TrootmI[ind].re =
                (((a2 * i_TrootmI_re_tmp - TrootmI_re_tmp * h_TrootmI_re_tmp) +
                  (b_TrootmI_re_tmp * k_TrootmI_re_tmp -
                   c_TrootmI_re_tmp * j_TrootmI_re_tmp)) +
                 (d_TrootmI_re_tmp * m_TrootmI_re_tmp -
                  e_TrootmI_re_tmp * l_TrootmI_re_tmp)) +
                (f_TrootmI_re_tmp * o_TrootmI_re_tmp -
                 g_TrootmI_re_tmp * n_TrootmI_re_tmp);
            b_TrootmI[ind].im =
                (((a2 * h_TrootmI_re_tmp + TrootmI_re_tmp * i_TrootmI_re_tmp) +
                  (b_TrootmI_re_tmp * j_TrootmI_re_tmp +
                   c_TrootmI_re_tmp * k_TrootmI_re_tmp)) +
                 (d_TrootmI_re_tmp * l_TrootmI_re_tmp +
                  e_TrootmI_re_tmp * m_TrootmI_re_tmp)) +
                (f_TrootmI_re_tmp * n_TrootmI_re_tmp +
                 g_TrootmI_re_tmp * o_TrootmI_re_tmp);
          }
        }
        a2 = std::fmin(a3, std::fmax(d4, rt_powd_snf(c_norm(b_TrootmI), 0.2)));
        if (a2 <= 0.28790937142411938) {
          m = 7;
          if (a2 <= 0.20609626234528361) {
            m = 6;
          }
          exitg2 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }
    if (guard1) {
      if (s == 100) {
        exitflag = 1;
        m = 7;
        exitg2 = true;
      } else {
        sqrtmTriRecursive(T, 1, 1, 4);
        for (j = 0; j < 16; j++) {
          TrootmI[j].re = T[j].re - static_cast<double>(TrootmI_tmp[j].re);
          TrootmI[j].im = T[j].im - static_cast<double>(TrootmI_tmp[j].im);
        }
        s++;
      }
    }
  }
  return s;
}

static int logmParams(double T[16], creal_T d[4], int &m, int &exitflag)
{
  double TrootmI[16];
  double c_tmp[16];
  double x[16];
  double a2;
  double b_s;
  double c_d;
  double d1;
  double d3;
  int ind;
  int j;
  int k;
  int s;
  int s0;
  signed char b_I[16];
  bool exitg2;
  bool foundm;
  m = 1;
  exitflag = 0;
  for (j = 0; j < 16; j++) {
    b_I[j] = 0;
  }
  b_I[0] = 1;
  b_I[5] = 1;
  b_I[10] = 1;
  b_I[15] = 1;
  s = 0;
  creal_T b_d[4];
  int exitg1;
  do {
    exitg1 = 0;
    b_d[0].re = d[0].re - 1.0;
    b_d[0].im = d[0].im;
    b_d[1].re = d[1].re - 1.0;
    b_d[1].im = d[1].im;
    b_d[2].re = d[2].re - 1.0;
    b_d[2].im = d[2].im;
    b_d[3].re = d[3].re - 1.0;
    b_d[3].im = d[3].im;
    if ((b_norm(b_d) > 0.28790937142411938) && (s < 100)) {
      internal::scalar::b_sqrt(d[0]);
      internal::scalar::b_sqrt(d[1]);
      internal::scalar::b_sqrt(d[2]);
      internal::scalar::b_sqrt(d[3]);
      s++;
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);
  s0 = s;
  if (s == 100) {
    exitflag = 1;
  }
  for (k = 0; k < s; k++) {
    sqrtmTriRecursive(T, 1, 1, 4);
  }
  for (j = 0; j < 16; j++) {
    TrootmI[j] = T[j] - static_cast<double>(b_I[j]);
  }
  for (j = 0; j < 4; j++) {
    for (int i{0}; i < 4; i++) {
      ind = i << 2;
      c_tmp[j + ind] =
          ((TrootmI[j] * TrootmI[ind] + TrootmI[j + 4] * TrootmI[ind + 1]) +
           TrootmI[j + 8] * TrootmI[ind + 2]) +
          TrootmI[j + 12] * TrootmI[ind + 3];
    }
  }
  for (j = 0; j < 4; j++) {
    a2 = TrootmI[j];
    b_s = TrootmI[j + 4];
    c_d = TrootmI[j + 8];
    d1 = TrootmI[j + 12];
    for (int i{0}; i < 4; i++) {
      ind = i << 2;
      x[j + ind] =
          ((a2 * c_tmp[ind] + b_s * c_tmp[ind + 1]) + c_d * c_tmp[ind + 2]) +
          d1 * c_tmp[ind + 3];
    }
  }
  a2 = 0.0;
  j = 0;
  exitg2 = false;
  while ((!exitg2) && (j < 4)) {
    ind = j << 2;
    b_s = ((std::abs(x[ind]) + std::abs(x[ind + 1])) + std::abs(x[ind + 2])) +
          std::abs(x[ind + 3]);
    if (std::isnan(b_s)) {
      a2 = rtNaN;
      exitg2 = true;
    } else {
      if (b_s > a2) {
        a2 = b_s;
      }
      j++;
    }
  }
  d3 = rt_powd_snf(a2, 0.33333333333333331);
  a2 = 0.0;
  j = 0;
  exitg2 = false;
  while ((!exitg2) && (j < 4)) {
    ind = j << 2;
    b_s = ((std::abs(c_tmp[ind]) + std::abs(c_tmp[ind + 1])) +
           std::abs(c_tmp[ind + 2])) +
          std::abs(c_tmp[ind + 3]);
    if (std::isnan(b_s)) {
      a2 = rtNaN;
      exitg2 = true;
    } else {
      if (b_s > a2) {
        a2 = b_s;
      }
      j++;
    }
  }
  a2 = std::fmax(std::sqrt(a2), d3);
  if (a2 <= 1.5869707387720632E-5) {
    foundm = true;
  } else if (a2 <= 0.0023138078842429789) {
    m = 2;
    foundm = true;
  } else {
    foundm = false;
  }
  k = 0;
  exitg2 = false;
  while ((!exitg2) && (!foundm)) {
    double a3;
    double d4;
    bool exitg3;
    bool guard1;
    bool guard2;
    bool more;
    more = false;
    if (s > s0) {
      for (j = 0; j < 4; j++) {
        for (int i{0}; i < 4; i++) {
          ind = i << 2;
          c_tmp[j + ind] =
              ((TrootmI[j] * TrootmI[ind] + TrootmI[j + 4] * TrootmI[ind + 1]) +
               TrootmI[j + 8] * TrootmI[ind + 2]) +
              TrootmI[j + 12] * TrootmI[ind + 3];
        }
      }
      for (j = 0; j < 4; j++) {
        a2 = TrootmI[j];
        b_s = TrootmI[j + 4];
        c_d = TrootmI[j + 8];
        d1 = TrootmI[j + 12];
        for (int i{0}; i < 4; i++) {
          ind = i << 2;
          x[j + ind] = ((a2 * c_tmp[ind] + b_s * c_tmp[ind + 1]) +
                        c_d * c_tmp[ind + 2]) +
                       d1 * c_tmp[ind + 3];
        }
      }
      a2 = 0.0;
      j = 0;
      exitg3 = false;
      while ((!exitg3) && (j < 4)) {
        ind = j << 2;
        b_s =
            ((std::abs(x[ind]) + std::abs(x[ind + 1])) + std::abs(x[ind + 2])) +
            std::abs(x[ind + 3]);
        if (std::isnan(b_s)) {
          a2 = rtNaN;
          exitg3 = true;
        } else {
          if (b_s > a2) {
            a2 = b_s;
          }
          j++;
        }
      }
      d3 = rt_powd_snf(a2, 0.33333333333333331);
    }
    for (j = 0; j < 4; j++) {
      for (int i{0}; i < 4; i++) {
        ind = i << 2;
        c_tmp[j + ind] =
            ((TrootmI[j] * TrootmI[ind] + TrootmI[j + 4] * TrootmI[ind + 1]) +
             TrootmI[j + 8] * TrootmI[ind + 2]) +
            TrootmI[j + 12] * TrootmI[ind + 3];
      }
    }
    for (j = 0; j < 4; j++) {
      for (int i{0}; i < 4; i++) {
        ind = i << 2;
        x[j + ind] = ((c_tmp[j] * c_tmp[ind] + c_tmp[j + 4] * c_tmp[ind + 1]) +
                      c_tmp[j + 8] * c_tmp[ind + 2]) +
                     c_tmp[j + 12] * c_tmp[ind + 3];
      }
    }
    a2 = 0.0;
    j = 0;
    exitg3 = false;
    while ((!exitg3) && (j < 4)) {
      ind = j << 2;
      b_s = ((std::abs(x[ind]) + std::abs(x[ind + 1])) + std::abs(x[ind + 2])) +
            std::abs(x[ind + 3]);
      if (std::isnan(b_s)) {
        a2 = rtNaN;
        exitg3 = true;
      } else {
        if (b_s > a2) {
          a2 = b_s;
        }
        j++;
      }
    }
    d4 = rt_powd_snf(a2, 0.25);
    a3 = std::fmax(d3, d4);
    guard1 = false;
    guard2 = false;
    if (a3 <= 0.28790937142411938) {
      ind = 0;
      j = 3;
      exitg3 = false;
      while ((!exitg3) && (j < 8)) {
        if (a3 <= dv2[j - 1]) {
          ind = j;
          exitg3 = true;
        } else {
          j++;
        }
      }
      if (ind <= 6) {
        m = ind;
        exitg2 = true;
      } else {
        if ((a3 / 2.0 <= 0.12764048108067749) && (k < 2)) {
          more = true;
          k++;
        }
        guard2 = true;
      }
    } else {
      guard2 = true;
    }
    if (guard2) {
      if (!more) {
        double b_c_tmp[16];
        for (j = 0; j < 4; j++) {
          for (int i{0}; i < 4; i++) {
            ind = i << 2;
            c_tmp[j + ind] = ((TrootmI[j] * TrootmI[ind] +
                               TrootmI[j + 4] * TrootmI[ind + 1]) +
                              TrootmI[j + 8] * TrootmI[ind + 2]) +
                             TrootmI[j + 12] * TrootmI[ind + 3];
          }
        }
        for (j = 0; j < 4; j++) {
          for (int i{0}; i < 4; i++) {
            ind = i << 2;
            b_c_tmp[j + ind] =
                ((c_tmp[j] * c_tmp[ind] + c_tmp[j + 4] * c_tmp[ind + 1]) +
                 c_tmp[j + 8] * c_tmp[ind + 2]) +
                c_tmp[j + 12] * c_tmp[ind + 3];
          }
        }
        for (j = 0; j < 4; j++) {
          a2 = TrootmI[j];
          b_s = TrootmI[j + 4];
          c_d = TrootmI[j + 8];
          d1 = TrootmI[j + 12];
          for (int i{0}; i < 4; i++) {
            ind = i << 2;
            x[j + ind] = ((a2 * b_c_tmp[ind] + b_s * b_c_tmp[ind + 1]) +
                          c_d * b_c_tmp[ind + 2]) +
                         d1 * b_c_tmp[ind + 3];
          }
        }
        a2 = 0.0;
        j = 0;
        exitg3 = false;
        while ((!exitg3) && (j < 4)) {
          ind = j << 2;
          b_s = ((std::abs(x[ind]) + std::abs(x[ind + 1])) +
                 std::abs(x[ind + 2])) +
                std::abs(x[ind + 3]);
          if (std::isnan(b_s)) {
            a2 = rtNaN;
            exitg3 = true;
          } else {
            if (b_s > a2) {
              a2 = b_s;
            }
            j++;
          }
        }
        a2 = std::fmin(a3, std::fmax(d4, rt_powd_snf(a2, 0.2)));
        if (a2 <= 0.28790937142411938) {
          m = 7;
          if (a2 <= 0.20609626234528361) {
            m = 6;
          }
          exitg2 = true;
        } else {
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
    }
    if (guard1) {
      if (s == 100) {
        exitflag = 1;
        m = 7;
        exitg2 = true;
      } else {
        sqrtmTriRecursive(T, 1, 1, 4);
        for (j = 0; j < 16; j++) {
          TrootmI[j] = T[j] - static_cast<double>(b_I[j]);
        }
        s++;
      }
    }
  }
  return s;
}

void logm(const double A[16], creal_T L[16])
{
  creal_T U[16];
  creal_T mu1;
  double Q[16];
  double T[16];
  double a;
  double b;
  double cs;
  double d;
  double r;
  double rt1i;
  double rt2i;
  double rt2r;
  double sn;
  double t1_re;
  int j;
  bool p;
  p = true;
  for (j = 0; j < 16; j++) {
    L[j].re = 0.0;
    L[j].im = 0.0;
    if (p) {
      d = A[j];
      if (std::isinf(d) || std::isnan(d)) {
        p = false;
      }
    } else {
      p = false;
    }
  }
  if (!p) {
    for (int i{0}; i < 16; i++) {
      L[i].re = rtNaN;
      L[i].im = 0.0;
    }
  } else {
    creal_T b_T[16];
    creal_T b_d[4];
    double U_re_tmp;
    double b_U_re_tmp;
    double c_U_re_tmp;
    double re;
    double rt1r;
    double s;
    int exitg2;
    int i;
    int i1;
    int mm1;
    bool exitg1;
    bool guard1;
    bool guard2;
    bool isPrincipalLog;
    bool schurA;
    schurA = true;
    j = 3;
    while (schurA && (j <= 4)) {
      mm1 = j;
      while (schurA && (mm1 <= 4)) {
        schurA = (A[(mm1 + ((j - 3) << 2)) - 1] == 0.0);
        mm1++;
      }
      j++;
    }
    if (schurA) {
      j = 0;
      exitg1 = false;
      while ((!exitg1) && (j < 3)) {
        i = j + (j << 2);
        d = A[i + 1];
        if (d != 0.0) {
          if ((j + 1 != 3) && (A[(j + ((j + 1) << 2)) + 2] != 0.0)) {
            schurA = false;
            exitg1 = true;
          } else {
            i1 = j + ((j + 1) << 2);
            if (A[i] != A[i1 + 1]) {
              schurA = false;
              exitg1 = true;
            } else {
              t1_re = A[i1];
              if (std::isnan(d)) {
                r = rtNaN;
              } else if (d < 0.0) {
                r = -1.0;
              } else {
                r = (d > 0.0);
              }
              if (std::isnan(t1_re)) {
                d = rtNaN;
              } else if (t1_re < 0.0) {
                d = -1.0;
              } else {
                d = (t1_re > 0.0);
              }
              if (r * d != -1.0) {
                schurA = false;
                exitg1 = true;
              } else {
                j++;
              }
            }
          }
        } else {
          j++;
        }
      }
    }
    if (schurA) {
      std::copy(&A[0], &A[16], &T[0]);
    } else {
      schur(A, Q, T);
    }
    ordeig(T, b_d);
    isPrincipalLog = true;
    mm1 = 0;
    exitg1 = false;
    while ((!exitg1) && (mm1 < 4)) {
      if ((b_d[mm1].re <= 0.0) && (b_d[mm1].im == 0.0)) {
        isPrincipalLog = false;
        exitg1 = true;
      } else {
        mm1++;
      }
    }
    p = true;
    j = 0;
    exitg1 = false;
    while ((!exitg1) && (j < 4)) {
      mm1 = 0;
      do {
        exitg2 = 0;
        if (mm1 <= j) {
          if (!(A[mm1 + (j << 2)] == A[j + (mm1 << 2)])) {
            p = false;
            exitg2 = 1;
          } else {
            mm1++;
          }
        } else {
          j++;
          exitg2 = 2;
        }
      } while (exitg2 == 0);
      if (exitg2 == 1) {
        exitg1 = true;
      }
    }
    guard1 = false;
    guard2 = false;
    if (p) {
      guard2 = true;
    } else {
      j = 0;
      int exitg3;
      do {
        exitg3 = 0;
        if (j < 4) {
          mm1 = 0;
          do {
            exitg2 = 0;
            if (mm1 <= j - 1) {
              if (T[mm1 + (j << 2)] != 0.0) {
                guard1 = true;
                exitg2 = 1;
              } else {
                mm1++;
              }
            } else {
              exitg2 = 2;
            }
          } while (exitg2 == 0);
          if (exitg2 == 1) {
            exitg3 = 1;
          } else if ((j + 1 != 4) && (T[(j + (j << 2)) + 1] != 0.0)) {
            guard1 = true;
            exitg3 = 1;
          } else {
            j++;
          }
        } else {
          guard2 = true;
          exitg3 = 1;
        }
      } while (exitg3 == 0);
    }
    if (guard2) {
      if (schurA) {
        L[0] = b_d[0];
        b_log(L[0]);
        L[5] = b_d[1];
        b_log(L[5]);
        L[10] = b_d[2];
        b_log(L[10]);
        L[15] = b_d[3];
        b_log(L[15]);
      } else {
        for (j = 0; j < 4; j++) {
          mu1 = b_d[j];
          b_log(mu1);
          i = j << 2;
          d = Q[i];
          L[i].re = d * mu1.re;
          L[i].im = d * mu1.im;
          b_T[i].re = Q[j];
          b_T[i].im = 0.0;
          d = Q[i + 1];
          L[i + 1].re = d * mu1.re;
          L[i + 1].im = d * mu1.im;
          b_T[i + 1].re = Q[j + 4];
          b_T[i + 1].im = 0.0;
          d = Q[i + 2];
          L[i + 2].re = d * mu1.re;
          L[i + 2].im = d * mu1.im;
          b_T[i + 2].re = Q[j + 8];
          b_T[i + 2].im = 0.0;
          d = Q[i + 3];
          L[i + 3].re = d * mu1.re;
          L[i + 3].im = d * mu1.im;
          b_T[i + 3].re = Q[j + 12];
          b_T[i + 3].im = 0.0;
        }
        for (i = 0; i < 4; i++) {
          r = L[i].re;
          s = L[i].im;
          t1_re = L[i + 4].re;
          rt1r = L[i + 4].im;
          rt2r = L[i + 8].re;
          rt2i = L[i + 8].im;
          cs = L[i + 12].re;
          sn = L[i + 12].im;
          for (i1 = 0; i1 < 4; i1++) {
            j = i1 << 2;
            a = b_T[j].im;
            b = b_T[j].re;
            rt1i = b_T[j + 1].im;
            d = b_T[j + 1].re;
            re = b_T[j + 2].im;
            U_re_tmp = b_T[j + 2].re;
            b_U_re_tmp = b_T[j + 3].im;
            c_U_re_tmp = b_T[j + 3].re;
            j += i;
            U[j].re = (((r * b - s * a) + (t1_re * d - rt1r * rt1i)) +
                       (rt2r * U_re_tmp - rt2i * re)) +
                      (cs * c_U_re_tmp - sn * b_U_re_tmp);
            U[j].im = (((r * a + s * b) + (t1_re * rt1i + rt1r * d)) +
                       (rt2r * re + rt2i * U_re_tmp)) +
                      (cs * b_U_re_tmp + sn * c_U_re_tmp);
          }
        }
        std::copy(&U[0], &U[16], &L[0]);
      }
    }
    if (guard1) {
      if (isPrincipalLog) {
        double Lr[16];
        computeLogOfSchurForm(T, b_d, Lr);
        if (schurA) {
          for (i = 0; i < 16; i++) {
            L[i].re = Lr[i];
            L[i].im = 0.0;
          }
        } else {
          for (i = 0; i < 4; i++) {
            d = Q[i];
            t1_re = Q[i + 4];
            r = Q[i + 8];
            s = Q[i + 12];
            for (i1 = 0; i1 < 4; i1++) {
              j = i1 << 2;
              T[i + j] = ((d * Lr[j] + t1_re * Lr[j + 1]) + r * Lr[j + 2]) +
                         s * Lr[j + 3];
            }
            d = T[i];
            t1_re = T[i + 4];
            r = T[i + 8];
            s = T[i + 12];
            for (i1 = 0; i1 < 4; i1++) {
              j = i + (i1 << 2);
              L[j].re = ((d * Q[i1] + t1_re * Q[i1 + 4]) + r * Q[i1 + 8]) +
                        s * Q[i1 + 12];
              L[j].im = 0.0;
            }
          }
        }
      } else {
        if (schurA) {
          std::memset(&Q[0], 0, 16U * sizeof(double));
          Q[0] = 1.0;
          Q[5] = 1.0;
          Q[10] = 1.0;
          Q[15] = 1.0;
        }
        for (i = 0; i < 16; i++) {
          b_T[i].re = T[i];
          b_T[i].im = 0.0;
          U[i].re = Q[i];
          U[i].im = 0.0;
        }
        for (int m{2}; m >= 0; m--) {
          mm1 = m + 1;
          i = m << 2;
          i1 = m + i;
          d = T[i1 + 1];
          if (d != 0.0) {
            int b_tmp_tmp;
            int re_tmp;
            a = T[i1];
            b_tmp_tmp = (m + 1) << 2;
            j = m + b_tmp_tmp;
            b = T[j];
            r = d;
            s = T[j + 1];
            t1_re = s;
            rt1r = internal::reflapack::xdlanv2(a, b, r, t1_re, rt1i, rt2r,
                                                rt2i, cs, sn);
            t1_re = rt1r - s;
            a = std::abs(t1_re);
            b = std::abs(rt1i);
            if (a < b) {
              a /= b;
              a = b * std::sqrt(a * a + 1.0);
            } else if (a > b) {
              b /= a;
              a *= std::sqrt(b * b + 1.0);
            } else if (std::isnan(b)) {
              a = rtNaN;
            } else {
              a *= 1.4142135623730951;
            }
            b = std::abs(d);
            if (a < b) {
              a /= b;
              r = b * std::sqrt(a * a + 1.0);
            } else if (a > b) {
              b /= a;
              r = a * std::sqrt(b * b + 1.0);
            } else if (std::isnan(b)) {
              r = rtNaN;
            } else {
              r = a * 1.4142135623730951;
            }
            if (rt1i == 0.0) {
              re = t1_re / r;
              rt1i = 0.0;
            } else if (t1_re == 0.0) {
              re = 0.0;
              rt1i /= r;
            } else {
              re = t1_re / r;
              rt1i /= r;
            }
            s = d / r;
            for (j = mm1; j < 5; j++) {
              re_tmp = m + ((j - 1) << 2);
              t1_re = b_T[re_tmp].re;
              r = b_T[re_tmp].im;
              rt1r = b_T[re_tmp + 1].re;
              rt2r = b_T[re_tmp + 1].im;
              b_T[re_tmp].re = (re * t1_re + rt1i * r) + s * rt1r;
              b_T[re_tmp].im = (re * r - rt1i * t1_re) + s * rt2r;
              rt2i = re * rt1r - rt1i * rt2r;
              cs = re * rt2r + rt1i * rt1r;
              b_T[re_tmp + 1].re = rt2i - s * t1_re;
              b_T[re_tmp + 1].im = cs - s * r;
            }
            for (mm1 = 0; mm1 <= m + 1; mm1++) {
              re_tmp = mm1 + i;
              t1_re = b_T[re_tmp].re;
              r = b_T[re_tmp].im;
              j = mm1 + b_tmp_tmp;
              rt1r = b_T[j].re;
              rt2r = b_T[j].im;
              rt2i = re * t1_re - rt1i * r;
              cs = re * r + rt1i * t1_re;
              b_T[re_tmp].re = rt2i + s * rt1r;
              b_T[re_tmp].im = cs + s * rt2r;
              b_T[j].re = (re * rt1r + rt1i * rt2r) - s * t1_re;
              b_T[j].im = (re * rt2r - rt1i * rt1r) - s * r;
            }
            t1_re = U[i].re;
            r = U[i].im;
            rt1r = U[b_tmp_tmp].re;
            rt2r = U[b_tmp_tmp].im;
            rt2i = re * t1_re - rt1i * r;
            cs = re * r + rt1i * t1_re;
            U[i].re = rt2i + s * rt1r;
            U[i].im = cs + s * rt2r;
            U[b_tmp_tmp].re = (re * rt1r + rt1i * rt2r) - s * t1_re;
            U[b_tmp_tmp].im = (re * rt2r - rt1i * rt1r) - s * r;
            t1_re = U[i + 1].re;
            r = U[i + 1].im;
            rt1r = U[b_tmp_tmp + 1].re;
            rt2r = U[b_tmp_tmp + 1].im;
            rt2i = re * t1_re - rt1i * r;
            cs = re * r + rt1i * t1_re;
            U[i + 1].re = rt2i + s * rt1r;
            U[i + 1].im = cs + s * rt2r;
            U[b_tmp_tmp + 1].re = (re * rt1r + rt1i * rt2r) - s * t1_re;
            U[b_tmp_tmp + 1].im = (re * rt2r - rt1i * rt1r) - s * r;
            t1_re = U[i + 2].re;
            r = U[i + 2].im;
            rt1r = U[b_tmp_tmp + 2].re;
            rt2r = U[b_tmp_tmp + 2].im;
            rt2i = re * t1_re - rt1i * r;
            cs = re * r + rt1i * t1_re;
            U[i + 2].re = rt2i + s * rt1r;
            U[i + 2].im = cs + s * rt2r;
            U[b_tmp_tmp + 2].re = (re * rt1r + rt1i * rt2r) - s * t1_re;
            U[b_tmp_tmp + 2].im = (re * rt2r - rt1i * rt1r) - s * r;
            t1_re = U[i + 3].re;
            r = U[i + 3].im;
            rt1r = U[b_tmp_tmp + 3].re;
            rt2r = U[b_tmp_tmp + 3].im;
            rt2i = re * t1_re - rt1i * r;
            cs = re * r + rt1i * t1_re;
            U[i + 3].re = rt2i + s * rt1r;
            U[i + 3].im = cs + s * rt2r;
            U[b_tmp_tmp + 3].re = (re * rt1r + rt1i * rt2r) - s * t1_re;
            U[b_tmp_tmp + 3].im = (re * rt2r - rt1i * rt1r) - s * r;
            b_T[i1 + 1].re = 0.0;
            b_T[i1 + 1].im = 0.0;
          }
        }
        computeLogOfSchurForm(b_T, b_d, L);
        for (i = 0; i < 4; i++) {
          r = U[i].re;
          s = U[i].im;
          t1_re = U[i + 4].re;
          rt1r = U[i + 4].im;
          rt2r = U[i + 8].re;
          rt2i = U[i + 8].im;
          cs = U[i + 12].re;
          sn = U[i + 12].im;
          for (i1 = 0; i1 < 4; i1++) {
            j = i1 << 2;
            a = L[j].im;
            b = L[j].re;
            rt1i = L[j + 1].im;
            d = L[j + 1].re;
            re = L[j + 2].im;
            U_re_tmp = L[j + 2].re;
            b_U_re_tmp = L[j + 3].im;
            c_U_re_tmp = L[j + 3].re;
            j += i;
            b_T[j].re = (((r * b - s * a) + (t1_re * d - rt1r * rt1i)) +
                         (rt2r * U_re_tmp - rt2i * re)) +
                        (cs * c_U_re_tmp - sn * b_U_re_tmp);
            b_T[j].im = (((r * a + s * b) + (t1_re * rt1i + rt1r * d)) +
                         (rt2r * re + rt2i * U_re_tmp)) +
                        (cs * b_U_re_tmp + sn * c_U_re_tmp);
          }
        }
        for (i = 0; i < 4; i++) {
          t1_re = b_T[i].re;
          rt1r = b_T[i].im;
          rt2r = b_T[i + 4].re;
          rt2i = b_T[i + 4].im;
          cs = b_T[i + 8].re;
          sn = b_T[i + 8].im;
          a = b_T[i + 12].re;
          b = b_T[i + 12].im;
          for (i1 = 0; i1 < 4; i1++) {
            r = U[i1].re;
            s = -U[i1].im;
            re = t1_re * r - rt1r * s;
            rt1i = t1_re * s + rt1r * r;
            r = U[i1 + 4].re;
            s = -U[i1 + 4].im;
            re += rt2r * r - rt2i * s;
            rt1i += rt2r * s + rt2i * r;
            r = U[i1 + 8].re;
            s = -U[i1 + 8].im;
            re += cs * r - sn * s;
            rt1i += cs * s + sn * r;
            r = U[i1 + 12].re;
            s = -U[i1 + 12].im;
            re += a * r - b * s;
            rt1i += a * s + b * r;
            j = i + (i1 << 2);
            L[j].re = re;
            L[j].im = rt1i;
          }
        }
      }
    }
  }
}

} // namespace coder

// End of code generation (logm.cpp)
