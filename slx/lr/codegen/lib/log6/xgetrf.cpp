//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgetrf.cpp
//
// Code generation for function 'xgetrf'
//

// Include files
#include "xgetrf.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <emmintrin.h>

// Function Definitions
namespace coder {
namespace internal {
namespace lapack {
int xgetrf(double A[16], int ipiv[4])
{
  static const int offsets[4]{0, 1, 2, 3};
  int info;
  _mm_storeu_si128(
      (__m128i *)&ipiv[0],
      _mm_add_epi32(
          _mm_set1_epi32(1),
          _mm_add_epi32(_mm_set1_epi32(0),
                        _mm_loadu_si128((const __m128i *)&offsets[0]))));
  info = 0;
  for (int j{0}; j < 3; j++) {
    double smax;
    int a;
    int b_tmp;
    int i;
    int jA;
    int jp1j;
    int mmj_tmp;
    mmj_tmp = 2 - j;
    b_tmp = j * 5;
    jp1j = b_tmp + 2;
    jA = 5 - j;
    a = 0;
    smax = std::abs(A[b_tmp]);
    for (int k{2}; k < jA; k++) {
      double s;
      s = std::abs(A[(b_tmp + k) - 1]);
      if (s > smax) {
        a = k - 1;
        smax = s;
      }
    }
    if (A[b_tmp + a] != 0.0) {
      if (a != 0) {
        jA = j + a;
        ipiv[j] = jA + 1;
        smax = A[j];
        A[j] = A[jA];
        A[jA] = smax;
        smax = A[j + 4];
        A[j + 4] = A[jA + 4];
        A[jA + 4] = smax;
        smax = A[j + 8];
        A[j + 8] = A[jA + 8];
        A[jA + 8] = smax;
        smax = A[j + 12];
        A[j + 12] = A[jA + 12];
        A[jA + 12] = smax;
      }
      i = (b_tmp - j) + 4;
      for (jA = jp1j; jA <= i; jA++) {
        A[jA - 1] /= A[b_tmp];
      }
    } else {
      info = j + 1;
    }
    jA = b_tmp;
    for (int k{0}; k <= mmj_tmp; k++) {
      smax = A[(b_tmp + (k << 2)) + 4];
      if (smax != 0.0) {
        i = jA + 6;
        a = (jA - j) + 8;
        for (jp1j = i; jp1j <= a; jp1j++) {
          A[jp1j - 1] += A[((b_tmp + jp1j) - jA) - 5] * -smax;
        }
      }
      jA += 4;
    }
  }
  if ((info == 0) && (!(A[15] != 0.0))) {
    info = 4;
  }
  return info;
}

} // namespace lapack
} // namespace internal
} // namespace coder

// End of code generation (xgetrf.cpp)
