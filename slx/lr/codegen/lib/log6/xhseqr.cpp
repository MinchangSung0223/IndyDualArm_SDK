//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xhseqr.cpp
//
// Code generation for function 'xhseqr'
//

// Include files
#include "xhseqr.h"
#include "rt_nonfinite.h"
#include "xdlanv2.h"
#include "xzlarfg.h"
#include <cmath>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace lapack {
int xhseqr(double h[16], double z[16])
{
  double v[3];
  double aa;
  double d;
  double h12;
  double h21;
  double h22;
  double rt2r;
  double s;
  double tst;
  int i;
  int info;
  int kdefl;
  bool exitg1;
  info = 0;
  h[3] = 0.0;
  h[2] = 0.0;
  h[7] = 0.0;
  kdefl = 0;
  i = 3;
  exitg1 = false;
  while ((!exitg1) && (i + 1 >= 1)) {
    int b_i;
    int b_k;
    int i1;
    int i2;
    int its;
    int k;
    int l;
    int nr;
    int tst_tmp_tmp;
    bool converged;
    bool exitg2;
    l = 1;
    converged = false;
    its = 0;
    exitg2 = false;
    while ((!exitg2) && (its < 301)) {
      bool exitg3;
      k = i;
      exitg3 = false;
      while ((!exitg3) && (k + 1 > l)) {
        b_i = k + ((k - 1) << 2);
        d = std::abs(h[b_i]);
        if (d <= 4.0083367200179456E-292) {
          exitg3 = true;
        } else {
          tst_tmp_tmp = k + (k << 2);
          h21 = std::abs(h[tst_tmp_tmp]);
          tst = std::abs(h[b_i - 1]) + h21;
          if (tst == 0.0) {
            if (k - 1 >= 1) {
              tst = std::abs(h[(k + ((k - 2) << 2)) - 1]);
            }
            if (k + 2 <= 4) {
              tst += std::abs(h[tst_tmp_tmp + 1]);
            }
          }
          if (d <= 2.2204460492503131E-16 * tst) {
            h12 = std::abs(h[tst_tmp_tmp - 1]);
            tst = std::abs(h[b_i - 1] - h[tst_tmp_tmp]);
            aa = std::fmax(h21, tst);
            tst = std::fmin(h21, tst);
            s = aa + tst;
            if (std::fmin(d, h12) * (std::fmax(d, h12) / s) <=
                std::fmax(4.0083367200179456E-292,
                          2.2204460492503131E-16 * (tst * (aa / s)))) {
              exitg3 = true;
            } else {
              k--;
            }
          } else {
            k--;
          }
        }
      }
      l = k + 1;
      if (k + 1 > 1) {
        h[k + ((k - 1) << 2)] = 0.0;
      }
      if (k + 1 >= i) {
        converged = true;
        exitg2 = true;
      } else {
        __m128d r;
        double rt1r;
        int m;
        kdefl++;
        if (kdefl - kdefl / 20 * 20 == 0) {
          s = std::abs(h[i + ((i - 1) << 2)]) +
              std::abs(h[(i + ((i - 2) << 2)) - 1]);
          tst = 0.75 * s + h[i + (i << 2)];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = tst;
        } else if (kdefl - kdefl / 10 * 10 == 0) {
          tst_tmp_tmp = k + (k << 2);
          s = std::abs(h[tst_tmp_tmp + 1]) +
              std::abs(h[(k + ((k + 1) << 2)) + 2]);
          tst = 0.75 * s + h[tst_tmp_tmp];
          h12 = -0.4375 * s;
          h21 = s;
          h22 = tst;
        } else {
          tst_tmp_tmp = i + ((i - 1) << 2);
          tst = h[tst_tmp_tmp - 1];
          h21 = h[tst_tmp_tmp];
          tst_tmp_tmp = i + (i << 2);
          h12 = h[tst_tmp_tmp - 1];
          h22 = h[tst_tmp_tmp];
        }
        s = ((std::abs(tst) + std::abs(h12)) + std::abs(h21)) + std::abs(h22);
        if (s == 0.0) {
          rt1r = 0.0;
          tst = 0.0;
          rt2r = 0.0;
          h21 = 0.0;
        } else {
          tst /= s;
          h21 /= s;
          h12 /= s;
          h22 /= s;
          aa = (tst + h22) / 2.0;
          tst = (tst - aa) * (h22 - aa) - h12 * h21;
          h21 = std::sqrt(std::abs(tst));
          if (tst >= 0.0) {
            rt1r = aa * s;
            rt2r = rt1r;
            tst = h21 * s;
            h21 = -tst;
          } else {
            rt1r = aa + h21;
            rt2r = aa - h21;
            if (std::abs(rt1r - h22) <= std::abs(rt2r - h22)) {
              rt1r *= s;
              rt2r = rt1r;
            } else {
              rt2r *= s;
              rt1r = rt2r;
            }
            tst = 0.0;
            h21 = 0.0;
          }
        }
        m = i - 1;
        exitg3 = false;
        while ((!exitg3) && (m >= k + 1)) {
          tst_tmp_tmp = m + ((m - 1) << 2);
          h12 = h[tst_tmp_tmp - 1];
          aa = h12 - rt2r;
          s = (std::abs(aa) + std::abs(h21)) + std::abs(h[tst_tmp_tmp]);
          h22 = h[tst_tmp_tmp] / s;
          nr = m + (m << 2);
          v[0] = (h22 * h[nr - 1] + aa * (aa / s)) - tst * (h21 / s);
          v[1] = h22 * (((h12 + h[nr]) - rt1r) - rt2r);
          v[2] = h22 * h[nr + 1];
          s = (std::abs(v[0]) + std::abs(v[1])) + std::abs(v[2]);
          r = _mm_loadu_pd(&v[0]);
          _mm_storeu_pd(&v[0], _mm_div_pd(r, _mm_set1_pd(s)));
          v[2] /= s;
          if ((m == k + 1) ||
              (std::abs(h[m - 1]) * (std::abs(v[1]) + std::abs(v[2])) <=
               2.2204460492503131E-16 * std::abs(v[0]) *
                   ((std::abs(h[0]) + std::abs(h[tst_tmp_tmp - 1])) +
                    std::abs(h[nr])))) {
            exitg3 = true;
          } else {
            m--;
          }
        }
        for (int c_k{m}; c_k <= i; c_k++) {
          tst_tmp_tmp = (i - c_k) + 2;
          if (tst_tmp_tmp >= 3) {
            nr = 3;
          } else {
            nr = tst_tmp_tmp;
          }
          if (c_k > m) {
            tst_tmp_tmp = (((c_k - 2) << 2) + c_k) - 1;
            for (b_k = 0; b_k < nr; b_k++) {
              v[b_k] = h[tst_tmp_tmp + b_k];
            }
          }
          tst = v[0];
          aa = reflapack::xzlarfg(nr, tst, v);
          if (c_k > m) {
            b_i = c_k + ((c_k - 2) << 2);
            h[b_i - 1] = tst;
            h[b_i] = 0.0;
            if (c_k < i) {
              h[c_k + 1] = 0.0;
            }
          } else if (m > k + 1) {
            h[c_k - 1] *= 1.0 - aa;
          }
          d = v[1];
          tst = aa * v[1];
          if (nr == 3) {
            __m128d r1;
            __m128d r2;
            __m128d r3;
            h22 = v[2];
            h12 = aa * v[2];
            for (int j{c_k}; j < 5; j++) {
              b_i = c_k + ((j - 1) << 2);
              rt2r = h[b_i - 1];
              rt1r = h[b_i];
              s = h[b_i + 1];
              h21 = (rt2r + d * rt1r) + h22 * s;
              rt2r -= h21 * aa;
              h[b_i - 1] = rt2r;
              rt1r -= h21 * tst;
              h[b_i] = rt1r;
              s -= h21 * h12;
              h[b_i + 1] = s;
            }
            if (c_k + 3 <= i + 1) {
              b_i = c_k + 2;
            } else {
              b_i = i;
            }
            tst_tmp_tmp = ((b_i + 1) / 2) << 1;
            nr = tst_tmp_tmp - 2;
            for (int j{0}; j <= nr; j += 2) {
              i1 = j + (c_k << 2);
              r = _mm_loadu_pd(&h[i1]);
              i2 = j + ((c_k + 1) << 2);
              r1 = _mm_loadu_pd(&h[i2]);
              b_k = j + ((c_k - 1) << 2);
              r2 = _mm_loadu_pd(&h[b_k]);
              r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(_mm_set1_pd(d), r)),
                              _mm_mul_pd(_mm_set1_pd(h22), r1));
              _mm_storeu_pd(&h[b_k],
                            _mm_sub_pd(r2, _mm_mul_pd(r3, _mm_set1_pd(aa))));
              _mm_storeu_pd(&h[i1],
                            _mm_sub_pd(r, _mm_mul_pd(r3, _mm_set1_pd(tst))));
              _mm_storeu_pd(&h[i2],
                            _mm_sub_pd(r1, _mm_mul_pd(r3, _mm_set1_pd(h12))));
            }
            for (int j{tst_tmp_tmp}; j <= b_i; j++) {
              i1 = j + ((c_k - 1) << 2);
              rt2r = h[i1];
              i2 = j + (c_k << 2);
              rt1r = h[i2];
              b_k = j + ((c_k + 1) << 2);
              s = h[b_k];
              h21 = (rt2r + d * rt1r) + h22 * s;
              rt2r -= h21 * aa;
              h[i1] = rt2r;
              rt1r -= h21 * tst;
              h[i2] = rt1r;
              s -= h21 * h12;
              h[b_k] = s;
            }
            __m128d r4;
            __m128d r5;
            __m128d r6;
            __m128d r7;
            __m128d r8;
            b_i = c_k << 2;
            r = _mm_loadu_pd(&z[b_i]);
            i1 = (c_k + 1) << 2;
            r1 = _mm_loadu_pd(&z[i1]);
            i2 = (c_k - 1) << 2;
            r2 = _mm_loadu_pd(&z[i2]);
            r4 = _mm_set1_pd(v[1]);
            r5 = _mm_set1_pd(v[2]);
            r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r4, r)),
                            _mm_mul_pd(r5, r1));
            r6 = _mm_set1_pd(aa);
            _mm_storeu_pd(&z[i2], _mm_sub_pd(r2, _mm_mul_pd(r3, r6)));
            r7 = _mm_set1_pd(tst);
            _mm_storeu_pd(&z[b_i], _mm_sub_pd(r, _mm_mul_pd(r3, r7)));
            r8 = _mm_set1_pd(h12);
            _mm_storeu_pd(&z[i1], _mm_sub_pd(r1, _mm_mul_pd(r3, r8)));
            r = _mm_loadu_pd(&z[b_i + 2]);
            r1 = _mm_loadu_pd(&z[i1 + 2]);
            r2 = _mm_loadu_pd(&z[i2 + 2]);
            r3 = _mm_add_pd(_mm_add_pd(r2, _mm_mul_pd(r4, r)),
                            _mm_mul_pd(r5, r1));
            _mm_storeu_pd(&z[i2 + 2], _mm_sub_pd(r2, _mm_mul_pd(r3, r6)));
            _mm_storeu_pd(&z[b_i + 2], _mm_sub_pd(r, _mm_mul_pd(r3, r7)));
            _mm_storeu_pd(&z[i1 + 2], _mm_sub_pd(r1, _mm_mul_pd(r3, r8)));
          } else if (nr == 2) {
            __m128d r1;
            __m128d r2;
            for (int j{c_k}; j < 5; j++) {
              b_i = c_k + ((j - 1) << 2);
              h22 = h[b_i - 1];
              rt2r = h[b_i];
              h21 = h22 + d * rt2r;
              h22 -= h21 * aa;
              h[b_i - 1] = h22;
              rt2r -= h21 * tst;
              h[b_i] = rt2r;
            }
            tst_tmp_tmp = ((i + 1) / 2) << 1;
            nr = tst_tmp_tmp - 2;
            for (int j{0}; j <= nr; j += 2) {
              b_i = j + (c_k << 2);
              r = _mm_loadu_pd(&h[b_i]);
              i1 = j + ((c_k - 1) << 2);
              r1 = _mm_loadu_pd(&h[i1]);
              r2 = _mm_add_pd(r1, _mm_mul_pd(_mm_set1_pd(d), r));
              _mm_storeu_pd(&h[i1],
                            _mm_sub_pd(r1, _mm_mul_pd(r2, _mm_set1_pd(aa))));
              _mm_storeu_pd(&h[b_i],
                            _mm_sub_pd(r, _mm_mul_pd(r2, _mm_set1_pd(tst))));
            }
            for (int j{tst_tmp_tmp}; j <= i; j++) {
              b_i = j + ((c_k - 1) << 2);
              h22 = h[b_i];
              i1 = j + (c_k << 2);
              rt2r = h[i1];
              h21 = h22 + d * rt2r;
              h22 -= h21 * aa;
              h[b_i] = h22;
              rt2r -= h21 * tst;
              h[i1] = rt2r;
            }
            __m128d r3;
            __m128d r4;
            __m128d r5;
            b_i = c_k << 2;
            r = _mm_loadu_pd(&z[b_i]);
            i1 = (c_k - 1) << 2;
            r1 = _mm_loadu_pd(&z[i1]);
            r3 = _mm_set1_pd(v[1]);
            r2 = _mm_add_pd(r1, _mm_mul_pd(r3, r));
            r4 = _mm_set1_pd(aa);
            _mm_storeu_pd(&z[i1], _mm_sub_pd(r1, _mm_mul_pd(r2, r4)));
            r5 = _mm_set1_pd(tst);
            _mm_storeu_pd(&z[b_i], _mm_sub_pd(r, _mm_mul_pd(r2, r5)));
            r = _mm_loadu_pd(&z[b_i + 2]);
            r1 = _mm_loadu_pd(&z[i1 + 2]);
            r2 = _mm_add_pd(r1, _mm_mul_pd(r3, r));
            _mm_storeu_pd(&z[i1 + 2], _mm_sub_pd(r1, _mm_mul_pd(r2, r4)));
            _mm_storeu_pd(&z[b_i + 2], _mm_sub_pd(r, _mm_mul_pd(r2, r5)));
          }
        }
        its++;
      }
    }
    if (!converged) {
      info = i + 1;
      exitg1 = true;
    } else {
      if ((l != i + 1) && (l == i)) {
        b_i = i << 2;
        i1 = i + b_i;
        d = h[i1 - 1];
        i2 = (i - 1) << 2;
        b_k = i + i2;
        h22 = h[b_k];
        rt2r = h[i1];
        reflapack::xdlanv2(h[b_k - 1], d, h22, rt2r, s, tst, h21, h12, aa);
        h[i1 - 1] = d;
        h[b_k] = h22;
        h[i1] = rt2r;
        if (i + 1 < 4) {
          i1 = 2 - i;
          tst_tmp_tmp = ((i + 1) << 2) + i;
          for (k = 0; k <= i1; k++) {
            nr = tst_tmp_tmp + (k << 2);
            tst = h[nr];
            h21 = h[nr - 1];
            h[nr] = h12 * tst - aa * h21;
            h[nr - 1] = h12 * h21 + aa * tst;
          }
        }
        if (i - 1 >= 1) {
          for (k = 0; k <= i - 2; k++) {
            tst_tmp_tmp = b_i + k;
            tst = h[tst_tmp_tmp];
            nr = i2 + k;
            h21 = h[nr];
            h[tst_tmp_tmp] = h12 * tst - aa * h21;
            h[nr] = h12 * h21 + aa * tst;
          }
        }
        tst = h12 * z[i2] + aa * z[b_i];
        z[b_i] = h12 * z[b_i] - aa * z[i2];
        z[i2] = tst;
        tst = z[b_i + 1];
        h21 = z[i2 + 1];
        z[b_i + 1] = h12 * tst - aa * h21;
        z[i2 + 1] = h12 * h21 + aa * tst;
        tst = z[b_i + 2];
        h21 = z[i2 + 2];
        z[b_i + 2] = h12 * tst - aa * h21;
        z[i2 + 2] = h12 * h21 + aa * tst;
        tst = z[b_i + 3];
        h21 = z[i2 + 3];
        z[b_i + 3] = h12 * tst - aa * h21;
        z[i2 + 3] = h12 * h21 + aa * tst;
      }
      kdefl = 0;
      i = l - 2;
    }
  }
  for (int j{0}; j < 2; j++) {
    for (i = j + 3; i < 5; i++) {
      h[(i + (j << 2)) - 1] = 0.0;
    }
  }
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder

// End of code generation (xhseqr.cpp)
