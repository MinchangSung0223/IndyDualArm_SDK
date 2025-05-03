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

void FD_552be714_1_validateRuntimeParameters(const double *rtp, int32_T
  *satFlags)
{
  boolean_T bb[24];
  double xx[1];
  xx[0] = rtp[0];
  bb[0] = sm_core_math_anyIsInf(1, xx + 0);
  bb[1] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[1];
  bb[2] = sm_core_math_anyIsInf(1, xx + 0);
  bb[3] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[2];
  bb[4] = sm_core_math_anyIsInf(1, xx + 0);
  bb[5] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[3];
  bb[6] = sm_core_math_anyIsInf(1, xx + 0);
  bb[7] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[4];
  bb[8] = sm_core_math_anyIsInf(1, xx + 0);
  bb[9] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[5];
  bb[10] = sm_core_math_anyIsInf(1, xx + 0);
  bb[11] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[6];
  bb[12] = sm_core_math_anyIsInf(1, xx + 0);
  bb[13] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[7];
  bb[14] = sm_core_math_anyIsInf(1, xx + 0);
  bb[15] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[8];
  bb[16] = sm_core_math_anyIsInf(1, xx + 0);
  bb[17] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[9];
  bb[18] = sm_core_math_anyIsInf(1, xx + 0);
  bb[19] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[10];
  bb[20] = sm_core_math_anyIsInf(1, xx + 0);
  bb[21] = sm_core_math_anyIsNaN(1, xx + 0);
  xx[0] = rtp[11];
  bb[22] = sm_core_math_anyIsInf(1, xx + 0);
  bb[23] = sm_core_math_anyIsNaN(1, xx + 0);
  satFlags[0] = !bb[0] ? 1 : 0;
  satFlags[1] = !bb[1] ? 1 : 0;
  satFlags[2] = !bb[2] ? 1 : 0;
  satFlags[3] = !bb[3] ? 1 : 0;
  satFlags[4] = !bb[4] ? 1 : 0;
  satFlags[5] = !bb[5] ? 1 : 0;
  satFlags[6] = !bb[6] ? 1 : 0;
  satFlags[7] = !bb[7] ? 1 : 0;
  satFlags[8] = !bb[8] ? 1 : 0;
  satFlags[9] = !bb[9] ? 1 : 0;
  satFlags[10] = !bb[10] ? 1 : 0;
  satFlags[11] = !bb[11] ? 1 : 0;
  satFlags[12] = !bb[12] ? 1 : 0;
  satFlags[13] = !bb[13] ? 1 : 0;
  satFlags[14] = !bb[14] ? 1 : 0;
  satFlags[15] = !bb[15] ? 1 : 0;
  satFlags[16] = !bb[16] ? 1 : 0;
  satFlags[17] = !bb[17] ? 1 : 0;
  satFlags[18] = !bb[18] ? 1 : 0;
  satFlags[19] = !bb[19] ? 1 : 0;
  satFlags[20] = !bb[20] ? 1 : 0;
  satFlags[21] = !bb[21] ? 1 : 0;
  satFlags[22] = !bb[22] ? 1 : 0;
  satFlags[23] = !bb[23] ? 1 : 0;
}

const NeAssertData FD_552be714_1_assertData[24] = {
  { "FD/DualArm/Left/l_joint_0", 0, 0, "DualArm.Left.l_joint_0", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Left/l_joint_0", 0, 0, "DualArm.Left.l_joint_0", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Left/l_joint_1", 0, 0, "DualArm.Left.l_joint_1", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Left/l_joint_1", 0, 0, "DualArm.Left.l_joint_1", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Left/l_joint_2", 0, 0, "DualArm.Left.l_joint_2", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Left/l_joint_2", 0, 0, "DualArm.Left.l_joint_2", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Left/l_joint_3", 0, 0, "DualArm.Left.l_joint_3", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Left/l_joint_3", 0, 0, "DualArm.Left.l_joint_3", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Left/l_joint_4", 0, 0, "DualArm.Left.l_joint_4", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Left/l_joint_4", 0, 0, "DualArm.Left.l_joint_4", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Left/l_joint_5", 0, 0, "DualArm.Left.l_joint_5", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Left/l_joint_5", 0, 0, "DualArm.Left.l_joint_5", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Right/r_joint_0", 0, 0, "DualArm.Right.r_joint_0", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Right/r_joint_0", 0, 0, "DualArm.Right.r_joint_0", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Right/r_joint_1", 0, 0, "DualArm.Right.r_joint_1", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Right/r_joint_1", 0, 0, "DualArm.Right.r_joint_1", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Right/r_joint_2", 0, 0, "DualArm.Right.r_joint_2", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Right/r_joint_2", 0, 0, "DualArm.Right.r_joint_2", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Right/r_joint_3", 0, 0, "DualArm.Right.r_joint_3", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Right/r_joint_3", 0, 0, "DualArm.Right.r_joint_3", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Right/r_joint_4", 0, 0, "DualArm.Right.r_joint_4", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Right/r_joint_4", 0, 0, "DualArm.Right.r_joint_4", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" },

  { "FD/DualArm/Right/r_joint_5", 0, 0, "DualArm.Right.r_joint_5", "", false,
    "The parameter Rz/Position contains an Inf value, which is not allowed.",
    "sm:model:evaluate:InvalidValueInf" },

  { "FD/DualArm/Right/r_joint_5", 0, 0, "DualArm.Right.r_joint_5", "", false,
    "The parameter Rz/Position contains a NaN value, which is not allowed.",
    "sm:model:evaluate:InvalidValueNaN" }
};
