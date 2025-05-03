/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'FD/DualArm/Solver Configuration'.
 */

#include <math.h>
#include <string.h>
#include "pm_std.h"
#include "pm_default_allocator.h"
#include "sm_std.h"
#include "ne_std.h"
#include "ssc_dae.h"
#include "sm_ssci_run_time_errors.h"
#include "sm_RuntimeDerivedValuesBundle.h"

void FD_552be714_1_computeRuntimeParameters(const real_T t0[], real_T out[])
{
  real_T t11[1];
  real_T t12[1];
  real_T t13[1];
  real_T t14[1];
  real_T t18[1];
  real_T t19[1];
  real_T t2[1];
  real_T t25[1];
  real_T t4[1];
  real_T t6[1];
  real_T t7[1];
  real_T t8[1];
  real_T t9[1];
  t2[0UL] = t0[0UL];
  t18[0UL] = t0[1UL];
  t4[0UL] = t0[2UL];
  t25[0UL] = t0[3UL];
  t6[0UL] = t0[4UL];
  t7[0UL] = t0[5UL];
  t8[0UL] = t0[6UL];
  t9[0UL] = t0[7UL];
  t19[0UL] = t0[8UL];
  t11[0UL] = t0[9UL];
  t12[0UL] = t0[10UL];
  t13[0UL] = t0[11UL];
  memcpy(&t14[0], &t8[0], 8U);
  memcpy(&t8[0], &t2[0], 8U);
  memcpy(&t2[0], &t9[0], 8U);
  memcpy(&t9[0], &t18[0], 8U);
  memcpy(&t18[0], &t19[0], 8U);
  memcpy(&t19[0], &t13[0], 8U);
  memcpy(&t13[0], &t4[0], 8U);
  memcpy(&t4[0], &t7[0], 8U);
  memcpy(&t7[0], &t12[0], 8U);
  memcpy(&t12[0], &t11[0], 8U);
  memcpy(&t11[0], &t25[0], 8U);
  memcpy(&t25[0], &t6[0], 8U);
  out[0UL] = t14[0UL];
  out[1UL] = t8[0UL];
  out[2UL] = t2[0UL];
  out[3UL] = t9[0UL];
  out[4UL] = t18[0UL];
  out[5UL] = t19[0UL];
  out[6UL] = t13[0UL];
  out[7UL] = t4[0UL];
  out[8UL] = t7[0UL];
  out[9UL] = t12[0UL];
  out[10UL] = t11[0UL];
  out[11UL] = t25[0UL];
}

void FD_552be714_1_computeAsmRuntimeDerivedValuesDoubles(const double *rtp,
  double *rtdvd)
{
  boolean_T bb[2];
  double xx[13];
  xx[0] = rtp[0];
  bb[0] = sm_core_math_anyIsInf(1, xx + 0);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = 0.0;
  xx[1] = !bb[0] && !bb[1] ? rtp[0] : xx[0];
  xx[2] = rtp[1];
  bb[0] = sm_core_math_anyIsInf(1, xx + 2);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 2);
  xx[2] = !bb[0] && !bb[1] ? rtp[1] : xx[0];
  xx[3] = rtp[2];
  bb[0] = sm_core_math_anyIsInf(1, xx + 3);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 3);
  xx[3] = !bb[0] && !bb[1] ? rtp[2] : xx[0];
  xx[4] = rtp[3];
  bb[0] = sm_core_math_anyIsInf(1, xx + 4);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 4);
  xx[4] = !bb[0] && !bb[1] ? rtp[3] : xx[0];
  xx[5] = rtp[4];
  bb[0] = sm_core_math_anyIsInf(1, xx + 5);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 5);
  xx[5] = !bb[0] && !bb[1] ? rtp[4] : xx[0];
  xx[6] = rtp[5];
  bb[0] = sm_core_math_anyIsInf(1, xx + 6);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 6);
  xx[6] = !bb[0] && !bb[1] ? rtp[5] : xx[0];
  xx[7] = rtp[6];
  bb[0] = sm_core_math_anyIsInf(1, xx + 7);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 7);
  xx[7] = !bb[0] && !bb[1] ? rtp[6] : xx[0];
  xx[8] = rtp[7];
  bb[0] = sm_core_math_anyIsInf(1, xx + 8);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 8);
  xx[8] = !bb[0] && !bb[1] ? rtp[7] : xx[0];
  xx[9] = rtp[8];
  bb[0] = sm_core_math_anyIsInf(1, xx + 9);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 9);
  xx[9] = !bb[0] && !bb[1] ? rtp[8] : xx[0];
  xx[10] = rtp[9];
  bb[0] = sm_core_math_anyIsInf(1, xx + 10);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 10);
  xx[10] = !bb[0] && !bb[1] ? rtp[9] : xx[0];
  xx[11] = rtp[10];
  bb[0] = sm_core_math_anyIsInf(1, xx + 11);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 11);
  xx[11] = !bb[0] && !bb[1] ? rtp[10] : xx[0];
  xx[12] = rtp[11];
  bb[0] = sm_core_math_anyIsInf(1, xx + 12);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 12);
  xx[12] = !bb[0] && !bb[1] ? rtp[11] : xx[0];
  rtdvd[0] = xx[1];
  rtdvd[1] = xx[2];
  rtdvd[2] = xx[3];
  rtdvd[3] = xx[4];
  rtdvd[4] = xx[5];
  rtdvd[5] = xx[6];
  rtdvd[6] = xx[7];
  rtdvd[7] = xx[8];
  rtdvd[8] = xx[9];
  rtdvd[9] = xx[10];
  rtdvd[10] = xx[11];
  rtdvd[11] = xx[12];
  rtdvd[12] = xx[1];
  rtdvd[13] = xx[2];
  rtdvd[14] = xx[3];
  rtdvd[15] = xx[4];
  rtdvd[16] = xx[5];
  rtdvd[17] = xx[6];
  rtdvd[18] = xx[7];
  rtdvd[19] = xx[8];
  rtdvd[20] = xx[9];
  rtdvd[21] = xx[10];
  rtdvd[22] = xx[11];
  rtdvd[23] = xx[12];
}

void FD_552be714_1_computeAsmRuntimeDerivedValuesInts(const double *rtp, int
  *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void FD_552be714_1_computeAsmRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  FD_552be714_1_computeAsmRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  FD_552be714_1_computeAsmRuntimeDerivedValuesInts(rtp, rtdv->mInts.mValues);
}

void FD_552be714_1_computeSimRuntimeDerivedValuesDoubles(const double *rtp,
  double *rtdvd)
{
  (void) rtp;
  (void) rtdvd;
}

void FD_552be714_1_computeSimRuntimeDerivedValuesInts(const double *rtp, int
  *rtdvi)
{
  (void) rtp;
  (void) rtdvi;
}

void FD_552be714_1_computeSimRuntimeDerivedValues(const double *rtp,
  RuntimeDerivedValuesBundle *rtdv)
{
  FD_552be714_1_computeSimRuntimeDerivedValuesDoubles(rtp,
    rtdv->mDoubles.mValues);
  FD_552be714_1_computeSimRuntimeDerivedValuesInts(rtp, rtdv->mInts.mValues);
}
