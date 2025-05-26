//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// sylvesterRecursive.cpp
//
// Code generation for function 'sylvesterRecursive'
//

// Include files
#include "sylvesterRecursive.h"
#include "log6_data.h"
#include "rt_nonfinite.h"
#include "xgetrf.h"
#include <cmath>

// Function Declarations
namespace coder {
static void sylvesterTriKernel(int ia0, int ja0, int ib0, int jb0,
                               creal_T C[16], int ic0, int jc0, int m, int n);

static void sylvesterTriKernel(int ia0, int ja0, int ib0, int jb0, double C[16],
                               int ic0, int jc0, int m, int n);

} // namespace coder

// Function Definitions
namespace coder {
static void sylvesterTriKernel(int ia0, int ja0, int ib0, int jb0,
                               creal_T C[16], int ic0, int jc0, int m, int n)
{
  for (int ii{m - 2}; ii + 2 >= 1; ii--) {
    for (int jj{-1}; jj + 2 <= n; jj++) {
      double c_C_re_tmp;
      double cmod_im;
      double cmod_re;
      double d_C_re_tmp;
      double e_C_re_tmp;
      double f_C_re_tmp;
      double re;
      int C_re_tmp;
      int b_C_re_tmp;
      int b_cmod_re_tmp_tmp;
      int cmod_re_tmp;
      int cmod_re_tmp_tmp;
      int i;
      cmod_re_tmp_tmp = (jc0 + jj) << 2;
      b_cmod_re_tmp_tmp = ic0 + ii;
      cmod_re_tmp = b_cmod_re_tmp_tmp + cmod_re_tmp_tmp;
      cmod_re = C[cmod_re_tmp].re;
      cmod_im = C[cmod_re_tmp].im;
      i = ii + 3;
      for (int kk{i}; kk <= m; kk++) {
        C_re_tmp = (ia0 + ii) + (((ja0 + kk) - 2) << 2);
        b_C_re_tmp = ((ic0 + kk) + cmod_re_tmp_tmp) - 2;
        c_C_re_tmp = C[C_re_tmp].re;
        d_C_re_tmp = C[b_C_re_tmp].im;
        e_C_re_tmp = C[C_re_tmp].im;
        f_C_re_tmp = C[b_C_re_tmp].re;
        cmod_re -= c_C_re_tmp * f_C_re_tmp - e_C_re_tmp * d_C_re_tmp;
        cmod_im -= c_C_re_tmp * d_C_re_tmp + e_C_re_tmp * f_C_re_tmp;
      }
      i = static_cast<unsigned char>(jj + 1);
      for (int kk{0}; kk < i; kk++) {
        C_re_tmp = b_cmod_re_tmp_tmp + (((jc0 + kk) - 1) << 2);
        b_C_re_tmp = ((ib0 + kk) + ((jb0 + jj) << 2)) - 1;
        c_C_re_tmp = C[C_re_tmp].re;
        d_C_re_tmp = C[b_C_re_tmp].im;
        e_C_re_tmp = C[C_re_tmp].im;
        f_C_re_tmp = C[b_C_re_tmp].re;
        cmod_re -= c_C_re_tmp * f_C_re_tmp - e_C_re_tmp * d_C_re_tmp;
        cmod_im -= c_C_re_tmp * d_C_re_tmp + e_C_re_tmp * f_C_re_tmp;
      }
      cmod_re_tmp_tmp = (ia0 + ii) + ((ja0 + ii) << 2);
      b_cmod_re_tmp_tmp = (ib0 + jj) + ((jb0 + jj) << 2);
      e_C_re_tmp = C[cmod_re_tmp_tmp].re + C[b_cmod_re_tmp_tmp].re;
      f_C_re_tmp = C[cmod_re_tmp_tmp].im + C[b_cmod_re_tmp_tmp].im;
      if (f_C_re_tmp == 0.0) {
        if (cmod_im == 0.0) {
          re = cmod_re / e_C_re_tmp;
          c_C_re_tmp = 0.0;
        } else if (cmod_re == 0.0) {
          re = 0.0;
          c_C_re_tmp = cmod_im / e_C_re_tmp;
        } else {
          re = cmod_re / e_C_re_tmp;
          c_C_re_tmp = cmod_im / e_C_re_tmp;
        }
      } else if (e_C_re_tmp == 0.0) {
        if (cmod_re == 0.0) {
          re = cmod_im / f_C_re_tmp;
          c_C_re_tmp = 0.0;
        } else if (cmod_im == 0.0) {
          re = 0.0;
          c_C_re_tmp = -(cmod_re / f_C_re_tmp);
        } else {
          re = cmod_im / f_C_re_tmp;
          c_C_re_tmp = -(cmod_re / f_C_re_tmp);
        }
      } else {
        double brm;
        brm = std::abs(e_C_re_tmp);
        c_C_re_tmp = std::abs(f_C_re_tmp);
        if (brm > c_C_re_tmp) {
          d_C_re_tmp = f_C_re_tmp / e_C_re_tmp;
          c_C_re_tmp = e_C_re_tmp + d_C_re_tmp * f_C_re_tmp;
          re = (cmod_re + d_C_re_tmp * cmod_im) / c_C_re_tmp;
          c_C_re_tmp = (cmod_im - d_C_re_tmp * cmod_re) / c_C_re_tmp;
        } else if (c_C_re_tmp == brm) {
          if (e_C_re_tmp > 0.0) {
            d_C_re_tmp = 0.5;
          } else {
            d_C_re_tmp = -0.5;
          }
          if (f_C_re_tmp > 0.0) {
            c_C_re_tmp = 0.5;
          } else {
            c_C_re_tmp = -0.5;
          }
          re = (cmod_re * d_C_re_tmp + cmod_im * c_C_re_tmp) / brm;
          c_C_re_tmp = (cmod_im * d_C_re_tmp - cmod_re * c_C_re_tmp) / brm;
        } else {
          d_C_re_tmp = e_C_re_tmp / f_C_re_tmp;
          c_C_re_tmp = f_C_re_tmp + d_C_re_tmp * e_C_re_tmp;
          re = (d_C_re_tmp * cmod_re + cmod_im) / c_C_re_tmp;
          c_C_re_tmp = (d_C_re_tmp * cmod_im - cmod_re) / c_C_re_tmp;
        }
      }
      C[cmod_re_tmp].re = re;
      C[cmod_re_tmp].im = c_C_re_tmp;
    }
  }
}

static void sylvesterTriKernel(int ia0, int ja0, int ib0, int jb0, double C[16],
                               int ic0, int jc0, int m, int n)
{
  double xxab[4];
  double B[2];
  int ii;
  ii = m - 2;
  while (ii + 2 >= 1) {
    int jj;
    bool blockA;
    jj = -1;
    if ((ii + 2 != 1) && (C[(ia0 + ii) + (((ja0 + ii) - 1) << 2)] != 0.0)) {
      blockA = true;
    } else {
      blockA = false;
    }
    while (jj + 2 <= n) {
      bool blockB;
      if ((jj + 2 != n) && (C[((ib0 + jj) + ((jb0 + jj) << 2)) + 1] != 0.0)) {
        blockB = true;
      } else {
        blockB = false;
      }
      if (!blockA) {
        if (!blockB) {
          double temp;
          int cmod1_tmp_tmp;
          int i;
          int kAcol;
          int r2;
          kAcol = (jc0 + jj) << 2;
          r2 = ic0 + ii;
          cmod1_tmp_tmp = r2 + kAcol;
          temp = C[cmod1_tmp_tmp];
          i = ii + 3;
          for (int kk{i}; kk <= m; kk++) {
            temp -= C[(ia0 + ii) + (((ja0 + kk) - 2) << 2)] *
                    C[((ic0 + kk) + kAcol) - 2];
          }
          i = static_cast<unsigned char>(jj + 1);
          for (int kk{0}; kk < i; kk++) {
            temp -= C[r2 + (((jc0 + kk) - 1) << 2)] *
                    C[((ib0 + kk) + ((jb0 + jj) << 2)) - 1];
          }
          C[cmod1_tmp_tmp] = temp / (C[(ia0 + ii) + ((ja0 + ii) << 2)] +
                                     C[(ib0 + jj) + ((jb0 + jj) << 2)]);
          jj++;
        } else {
          double c_cmod1_tmp;
          double cmod1;
          double cmod2;
          double temp;
          int KM_tmp_tmp;
          int b_cmod1_tmp;
          int cmod1_tmp;
          int cmod3_tmp_tmp;
          int i;
          int kAcol;
          int r2;
          cmod1_tmp = ic0 + ii;
          b_cmod1_tmp = jc0 + jj;
          r2 = b_cmod1_tmp << 2;
          cmod3_tmp_tmp = cmod1_tmp + r2;
          cmod1 = C[cmod3_tmp_tmp];
          kAcol = (b_cmod1_tmp + 1) << 2;
          KM_tmp_tmp = cmod1_tmp + kAcol;
          cmod2 = C[KM_tmp_tmp];
          i = ii + 3;
          for (int kk{i}; kk <= m; kk++) {
            temp = C[(ia0 + ii) + (((ja0 + kk) - 2) << 2)];
            b_cmod1_tmp = ic0 + kk;
            cmod1 -= temp * C[(b_cmod1_tmp + r2) - 2];
            cmod2 -= temp * C[(b_cmod1_tmp + kAcol) - 2];
          }
          i = static_cast<unsigned char>(jj + 1);
          for (int kk{0}; kk < i; kk++) {
            temp = C[cmod1_tmp + (((jc0 + kk) - 1) << 2)];
            b_cmod1_tmp = ib0 + kk;
            r2 = jb0 + jj;
            cmod1 -= temp * C[(b_cmod1_tmp + (r2 << 2)) - 1];
            cmod2 -= temp * C[(b_cmod1_tmp + ((r2 + 1) << 2)) - 1];
          }
          int cmod1_tmp_tmp;
          kAcol = ib0 + jj;
          r2 = jb0 + jj;
          cmod1_tmp_tmp = kAcol + (r2 << 2);
          temp = C[(ia0 + ii) + ((ja0 + ii) << 2)];
          kAcol += (r2 + 1) << 2;
          c_cmod1_tmp = temp + C[cmod1_tmp_tmp];
          xxab[0] = c_cmod1_tmp;
          xxab[1] = C[kAcol];
          xxab[2] = C[cmod1_tmp_tmp + 1];
          xxab[3] = temp + C[kAcol + 1];
          B[0] = cmod1;
          B[1] = cmod2;
          if (std::abs(C[kAcol]) > std::abs(c_cmod1_tmp)) {
            kAcol = 1;
            r2 = 0;
          } else {
            kAcol = 0;
            r2 = 1;
          }
          temp = xxab[r2] / xxab[kAcol];
          c_cmod1_tmp = xxab[kAcol + 2];
          temp =
              (B[r2] - B[kAcol] * temp) / (xxab[r2 + 2] - temp * c_cmod1_tmp);
          C[cmod3_tmp_tmp] = (B[kAcol] - temp * c_cmod1_tmp) / xxab[kAcol];
          C[KM_tmp_tmp] = temp;
          jj += 2;
        }
      } else if (!blockB) {
        double c_cmod1_tmp;
        double cmod1;
        double cmod2;
        double temp;
        int b_cmod1_tmp;
        int cmod1_tmp;
        int i;
        int kAcol;
        int r2;
        r2 = (jc0 + jj) << 2;
        kAcol = ic0 + ii;
        cmod1_tmp = kAcol + r2;
        cmod1 = C[cmod1_tmp - 1];
        cmod2 = C[cmod1_tmp];
        i = ii + 3;
        for (int kk{i}; kk <= m; kk++) {
          b_cmod1_tmp = (ia0 + ii) + (((ja0 + kk) - 2) << 2);
          temp = C[((ic0 + kk) + r2) - 2];
          cmod1 -= C[b_cmod1_tmp - 1] * temp;
          cmod2 -= C[b_cmod1_tmp] * temp;
        }
        i = static_cast<unsigned char>(jj + 1);
        for (int kk{0}; kk < i; kk++) {
          b_cmod1_tmp = kAcol + (((jc0 + kk) - 1) << 2);
          temp = C[((ib0 + kk) + ((jb0 + jj) << 2)) - 1];
          cmod1 -= C[b_cmod1_tmp - 1] * temp;
          cmod2 -= C[b_cmod1_tmp] * temp;
        }
        int cmod1_tmp_tmp;
        cmod1_tmp_tmp = ia0 + ii;
        kAcol = ja0 + ii;
        r2 = cmod1_tmp_tmp + ((kAcol - 1) << 2);
        temp = C[(ib0 + jj) + ((jb0 + jj) << 2)];
        kAcol = cmod1_tmp_tmp + (kAcol << 2);
        c_cmod1_tmp = C[r2 - 1] + temp;
        xxab[0] = c_cmod1_tmp;
        xxab[1] = C[r2];
        xxab[2] = C[kAcol - 1];
        xxab[3] = C[kAcol] + temp;
        B[0] = cmod1;
        B[1] = cmod2;
        if (std::abs(C[r2]) > std::abs(c_cmod1_tmp)) {
          kAcol = 1;
          r2 = 0;
        } else {
          kAcol = 0;
          r2 = 1;
        }
        temp = xxab[r2] / xxab[kAcol];
        c_cmod1_tmp = xxab[kAcol + 2];
        temp = (B[r2] - B[kAcol] * temp) / (xxab[r2 + 2] - temp * c_cmod1_tmp);
        C[cmod1_tmp - 1] = (B[kAcol] - temp * c_cmod1_tmp) / xxab[kAcol];
        C[cmod1_tmp] = temp;
        jj++;
      } else {
        double KM[16];
        double c_cmod1_tmp;
        double cmod1;
        double cmod2;
        double cmod2_tmp;
        double cmod3;
        double cmod4;
        double temp;
        int ipiv[4];
        int b_cmod1_tmp;
        int cmod1_tmp;
        int cmod1_tmp_tmp;
        int cmod3_tmp;
        int cmod3_tmp_tmp;
        int i;
        int kAcol;
        int r2;
        r2 = ic0 + ii;
        kAcol = jc0 + jj;
        cmod1_tmp_tmp = kAcol << 2;
        cmod1_tmp = r2 + cmod1_tmp_tmp;
        cmod1 = C[cmod1_tmp - 1];
        cmod2 = C[cmod1_tmp];
        cmod3_tmp_tmp = (kAcol + 1) << 2;
        cmod3_tmp = r2 + cmod3_tmp_tmp;
        cmod3 = C[cmod3_tmp - 1];
        cmod4 = C[cmod3_tmp];
        i = ii + 3;
        for (int kk{i}; kk <= m; kk++) {
          b_cmod1_tmp = (ia0 + ii) + (((ja0 + kk) - 2) << 2);
          kAcol = ic0 + kk;
          temp = C[(kAcol + cmod1_tmp_tmp) - 2];
          c_cmod1_tmp = C[b_cmod1_tmp - 1];
          cmod1 -= c_cmod1_tmp * temp;
          cmod2_tmp = C[b_cmod1_tmp];
          cmod2 -= cmod2_tmp * temp;
          temp = C[(kAcol + cmod3_tmp_tmp) - 2];
          cmod3 -= c_cmod1_tmp * temp;
          cmod4 -= cmod2_tmp * temp;
        }
        i = static_cast<unsigned char>(jj + 1);
        for (int kk{0}; kk < i; kk++) {
          b_cmod1_tmp = r2 + (((jc0 + kk) - 1) << 2);
          kAcol = ib0 + kk;
          cmod1_tmp_tmp = jb0 + jj;
          temp = C[(kAcol + (cmod1_tmp_tmp << 2)) - 1];
          c_cmod1_tmp = C[b_cmod1_tmp - 1];
          cmod1 -= c_cmod1_tmp * temp;
          cmod2_tmp = C[b_cmod1_tmp];
          cmod2 -= cmod2_tmp * temp;
          temp = C[(kAcol + ((cmod1_tmp_tmp + 1) << 2)) - 1];
          cmod3 -= c_cmod1_tmp * temp;
          cmod4 -= cmod2_tmp * temp;
        }
        int KM_tmp_tmp;
        r2 = ia0 + ii;
        kAcol = ja0 + ii;
        cmod1_tmp_tmp = ib0 + jj;
        cmod3_tmp_tmp = jb0 + jj;
        b_cmod1_tmp = cmod1_tmp_tmp + (cmod3_tmp_tmp << 2);
        temp = C[b_cmod1_tmp];
        KM_tmp_tmp = r2 + ((kAcol - 1) << 2);
        c_cmod1_tmp = C[KM_tmp_tmp - 1];
        KM[0] = c_cmod1_tmp + temp;
        r2 += kAcol << 2;
        cmod2_tmp = C[r2];
        KM[5] = cmod2_tmp + temp;
        cmod1_tmp_tmp += (cmod3_tmp_tmp + 1) << 2;
        temp = C[cmod1_tmp_tmp + 1];
        KM[10] = c_cmod1_tmp + temp;
        KM[15] = cmod2_tmp + temp;
        temp = C[KM_tmp_tmp];
        KM[1] = temp;
        c_cmod1_tmp = C[r2 - 1];
        KM[4] = c_cmod1_tmp;
        KM[11] = temp;
        KM[14] = c_cmod1_tmp;
        temp = C[cmod1_tmp_tmp];
        KM[2] = temp;
        c_cmod1_tmp = C[b_cmod1_tmp + 1];
        KM[8] = c_cmod1_tmp;
        KM[7] = temp;
        KM[13] = c_cmod1_tmp;
        KM[3] = 0.0;
        KM[6] = 0.0;
        KM[9] = 0.0;
        KM[12] = 0.0;
        xxab[0] = cmod1;
        xxab[1] = cmod2;
        xxab[2] = cmod3;
        xxab[3] = cmod4;
        internal::lapack::xgetrf(KM, ipiv);
        if (ipiv[0] != 1) {
          xxab[0] = xxab[ipiv[0] - 1];
          xxab[ipiv[0] - 1] = cmod1;
        }
        if (ipiv[1] != 2) {
          temp = xxab[1];
          xxab[1] = xxab[ipiv[1] - 1];
          xxab[ipiv[1] - 1] = temp;
        }
        if (ipiv[2] != 3) {
          temp = xxab[2];
          xxab[2] = xxab[ipiv[2] - 1];
          xxab[ipiv[2] - 1] = temp;
        }
        for (r2 = 0; r2 < 4; r2++) {
          kAcol = r2 << 2;
          if (xxab[r2] != 0.0) {
            i = r2 + 2;
            for (cmod1_tmp_tmp = i; cmod1_tmp_tmp < 5; cmod1_tmp_tmp++) {
              xxab[cmod1_tmp_tmp - 1] -=
                  xxab[r2] * KM[(cmod1_tmp_tmp + kAcol) - 1];
            }
          }
        }
        for (r2 = 3; r2 >= 0; r2--) {
          kAcol = r2 << 2;
          temp = xxab[r2];
          if (temp != 0.0) {
            temp /= KM[r2 + kAcol];
            xxab[r2] = temp;
            for (cmod1_tmp_tmp = 0; cmod1_tmp_tmp < r2; cmod1_tmp_tmp++) {
              xxab[cmod1_tmp_tmp] -= xxab[r2] * KM[cmod1_tmp_tmp + kAcol];
            }
          }
        }
        C[cmod1_tmp - 1] = xxab[0];
        C[cmod1_tmp] = xxab[1];
        C[cmod3_tmp - 1] = xxab[2];
        C[cmod3_tmp] = xxab[3];
        jj += 2;
      }
    }
    if (blockA) {
      ii -= 2;
    } else {
      ii--;
    }
  }
}

void sylvesterRecursive(int ia0, int ja0, int ib0, int jb0, creal_T C[16],
                        int ic0, int jc0, int m, int n)
{
  if ((m >= n) && (m > 8)) {
    int aStart;
    int i;
    int n1;
    int xStart;
    n1 = m / 2;
    i = ja0 + n1;
    xStart = m - n1;
    sylvesterRecursive(ia0 + n1, i, ib0, jb0, C, ic0 + n1, jc0, xStart, n);
    aStart = ((i - 1) << 2) + ia0;
    for (int kk{0}; kk < n; kk++) {
      int iy0_tmp;
      iy0_tmp = (((jc0 + kk) - 1) << 2) + ic0;
      if (xStart != 0) {
        int ix;
        ix = iy0_tmp + n1;
        i = aStart + ((xStart - 1) << 2);
        for (int iac{aStart}; iac <= i; iac += 4) {
          double b_c_re_tmp;
          double c_im;
          double c_re;
          int c_re_tmp;
          c_re_tmp = (ix + ((iac - aStart) >> 2)) - 1;
          c_im = C[c_re_tmp].im;
          b_c_re_tmp = C[c_re_tmp].re;
          c_re = -b_c_re_tmp - 0.0 * c_im;
          c_im = -c_im + 0.0 * b_c_re_tmp;
          c_re_tmp = iac + n1;
          for (int ia{iac}; ia < c_re_tmp; ia++) {
            double C_re_tmp;
            int i1;
            b_c_re_tmp = C[ia - 1].re;
            C_re_tmp = C[ia - 1].im;
            i1 = ((iy0_tmp + ia) - iac) - 1;
            C[i1].re += b_c_re_tmp * c_re - C_re_tmp * c_im;
            C[i1].im += b_c_re_tmp * c_im + C_re_tmp * c_re;
          }
        }
      }
    }
    sylvesterRecursive(ia0, ja0, ib0, jb0, C, ic0, jc0, n1, n);
  } else if (n > 8) {
    int n1;
    int xStart;
    n1 = n / 2;
    sylvesterRecursive(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n1);
    xStart = ((jc0 - 1) << 2) + ic0;
    for (int kk{n1}; kk < n; kk++) {
      int aStart;
      aStart = ((((jc0 + kk) - 1) << 2) + ic0) - 1;
      if (m != 0) {
        int i;
        int ix;
        ix = (((jb0 + kk) - 1) << 2) + ib0;
        i = xStart + ((n1 - 1) << 2);
        for (int iac{xStart}; iac <= i; iac += 4) {
          double b_c_re_tmp;
          double c_im;
          double c_re;
          int c_re_tmp;
          c_re_tmp = (ix + ((iac - xStart) >> 2)) - 1;
          c_im = C[c_re_tmp].im;
          b_c_re_tmp = C[c_re_tmp].re;
          c_re = -b_c_re_tmp - 0.0 * c_im;
          c_im = -c_im + 0.0 * b_c_re_tmp;
          c_re_tmp = iac + m;
          for (int ia{iac}; ia < c_re_tmp; ia++) {
            double C_re_tmp;
            int i1;
            b_c_re_tmp = C[ia - 1].re;
            C_re_tmp = C[ia - 1].im;
            i1 = (aStart + ia) - iac;
            C[i1].re += b_c_re_tmp * c_re - C_re_tmp * c_im;
            C[i1].im += b_c_re_tmp * c_im + C_re_tmp * c_re;
          }
        }
      }
    }
    sylvesterRecursive(ia0, ja0, ib0 + n1, jb0 + n1, C, ic0, jc0 + n1, m,
                       n - n1);
  } else {
    sylvesterTriKernel(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n);
  }
}

void sylvesterRecursive(int ia0, int ja0, int ib0, int jb0, double C[16],
                        int ic0, int jc0, int m, int n)
{
  if ((m >= n) && (m > 8)) {
    int aStart;
    int i;
    int n1;
    int xStart;
    n1 = m / 2;
    if (C[((ia0 + n1) + (((ja0 + n1) - 2) << 2)) - 1] != 0.0) {
      n1++;
    }
    i = ja0 + n1;
    xStart = m - n1;
    sylvesterRecursive(ia0 + n1, i, ib0, jb0, C, ic0 + n1, jc0, xStart, n);
    aStart = ((i - 1) << 2) + ia0;
    for (int kk{0}; kk < n; kk++) {
      int iy0_tmp;
      iy0_tmp = (((jc0 + kk) - 1) << 2) + ic0;
      if (xStart != 0) {
        int ix;
        ix = iy0_tmp + n1;
        i = aStart + ((xStart - 1) << 2);
        for (int iac{aStart}; iac <= i; iac += 4) {
          double c;
          int i1;
          c = -C[(ix + ((iac - aStart) >> 2)) - 1];
          i1 = iac + n1;
          for (int ia{iac}; ia < i1; ia++) {
            int i2;
            i2 = ((iy0_tmp + ia) - iac) - 1;
            C[i2] += C[ia - 1] * c;
          }
        }
      }
    }
    sylvesterRecursive(ia0, ja0, ib0, jb0, C, ic0, jc0, n1, n);
  } else if (n > 8) {
    int n1;
    int xStart;
    n1 = n / 2;
    if (C[((ib0 + n1) + (((jb0 + n1) - 2) << 2)) - 1] != 0.0) {
      n1++;
    }
    sylvesterRecursive(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n1);
    xStart = ((jc0 - 1) << 2) + ic0;
    for (int kk{n1}; kk < n; kk++) {
      int aStart;
      aStart = ((((jc0 + kk) - 1) << 2) + ic0) - 1;
      if (m != 0) {
        int i;
        int ix;
        ix = (((jb0 + kk) - 1) << 2) + ib0;
        i = xStart + ((n1 - 1) << 2);
        for (int iac{xStart}; iac <= i; iac += 4) {
          double c;
          int i1;
          c = -C[(ix + ((iac - xStart) >> 2)) - 1];
          i1 = iac + m;
          for (int ia{iac}; ia < i1; ia++) {
            int i2;
            i2 = (aStart + ia) - iac;
            C[i2] += C[ia - 1] * c;
          }
        }
      }
    }
    sylvesterRecursive(ia0, ja0, ib0 + n1, jb0 + n1, C, ic0, jc0 + n1, m,
                       n - n1);
  } else {
    sylvesterTriKernel(ia0, ja0, ib0, jb0, C, ic0, jc0, m, n);
  }
}

} // namespace coder

// End of code generation (sylvesterRecursive.cpp)
