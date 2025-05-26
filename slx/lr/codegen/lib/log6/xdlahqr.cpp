//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xdlahqr.cpp
//
// Code generation for function 'xdlahqr'
//

// Include files
#include "xdlahqr.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
namespace reflapack {
int xdlahqr(int ilo, int ihi, double h[4], double &z, double wr[2],
            double wi[2])
{
  double aa;
  double ab_tmp;
  double cs;
  double sn;
  int i;
  int info;
  int k;
  z = 1.0;
  info = 0;
  i = static_cast<unsigned char>(ilo - 1);
  for (int b_i{0}; b_i < i; b_i++) {
    wr[b_i] = h[b_i + (b_i << 1)];
    wi[b_i] = 0.0;
  }
  if (ihi + 1 <= 2) {
    wr[1] = h[3];
    wi[1] = 0.0;
  }
  if (ilo == ihi) {
    wr[ilo - 1] = h[(ilo + ((ilo - 1) << 1)) - 1];
    wi[ilo - 1] = 0.0;
  } else {
    double smlnum;
    smlnum = 2.2250738585072014E-308 *
             (static_cast<double>((ihi - ilo) + 1) / 2.2204460492503131E-16);
    for (int b_i{ihi - 1}; b_i + 1 >= ilo; b_i = k - 2) {
      double bb;
      double d;
      double s;
      bool exitg1;
      k = b_i + 1;
      exitg1 = false;
      while ((!exitg1) && (k > ilo)) {
        d = std::abs(h[1]);
        if (d <= smlnum) {
          exitg1 = true;
        } else {
          bb = std::abs(h[3]);
          if (d <= 2.2204460492503131E-16 * (std::abs(h[0]) + bb)) {
            ab_tmp = std::abs(h[2]);
            s = std::abs(h[0] - h[3]);
            aa = std::fmax(bb, s);
            bb = std::fmin(bb, s);
            s = aa + bb;
            if (std::fmin(d, ab_tmp) * (std::fmax(d, ab_tmp) / s) <=
                std::fmax(smlnum, 2.2204460492503131E-16 * (bb * (aa / s)))) {
              exitg1 = true;
            } else {
              k = 1;
            }
          } else {
            k = 1;
          }
        }
      }
      if (k > ilo) {
        h[1] = 0.0;
      }
      if (k == b_i + 1) {
        wr[b_i] = h[b_i + (b_i << 1)];
        wi[b_i] = 0.0;
      } else if (k == b_i) {
        int i1;
        i = b_i << 1;
        d = h[i];
        bb = h[b_i];
        i1 = b_i + i;
        s = h[i1];
        wr[0] = xdlanv2(h[0], d, bb, s, wi[0], ab_tmp, aa, cs, sn);
        wr[b_i] = ab_tmp;
        wi[b_i] = aa;
        h[i] = d;
        h[b_i] = bb;
        h[i1] = s;
      }
    }
  }
  return info;
}

} // namespace reflapack
} // namespace internal
} // namespace coder

// End of code generation (xdlahqr.cpp)
