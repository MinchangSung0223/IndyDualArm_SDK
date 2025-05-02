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
#include "sm_CTarget.h"

static void setTargets_0(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[0];
}

static void setTargets_2(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[1];
}

static void setTargets_4(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[2];
}

static void setTargets_6(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[3];
}

static void setTargets_8(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[4];
}

static void setTargets_10(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[5];
}

static void setTargets_12(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[6];
}

static void setTargets_14(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[7];
}

static void setTargets_16(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[8];
}

static void setTargets_18(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[9];
}

static void setTargets_20(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[10];
}

static void setTargets_22(const RuntimeDerivedValuesBundle *rtdv, real_T *values,
  double *auxData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) rtdvi;
  (void) auxData;
  values[0] = rtdvd[11];
}

void FD_cd27307_1_setTargets(const RuntimeDerivedValuesBundle *rtdv, CTarget
  *targets)
{
  setTargets_0(rtdv, sm_core_SmRealVector_nonConstValues(&targets[0].mValue),
               targets[0].mAuxiliaryTargetData);
  setTargets_2(rtdv, sm_core_SmRealVector_nonConstValues(&targets[2].mValue),
               targets[2].mAuxiliaryTargetData);
  setTargets_4(rtdv, sm_core_SmRealVector_nonConstValues(&targets[4].mValue),
               targets[4].mAuxiliaryTargetData);
  setTargets_6(rtdv, sm_core_SmRealVector_nonConstValues(&targets[6].mValue),
               targets[6].mAuxiliaryTargetData);
  setTargets_8(rtdv, sm_core_SmRealVector_nonConstValues(&targets[8].mValue),
               targets[8].mAuxiliaryTargetData);
  setTargets_10(rtdv, sm_core_SmRealVector_nonConstValues(&targets[10].mValue),
                targets[10].mAuxiliaryTargetData);
  setTargets_12(rtdv, sm_core_SmRealVector_nonConstValues(&targets[12].mValue),
                targets[12].mAuxiliaryTargetData);
  setTargets_14(rtdv, sm_core_SmRealVector_nonConstValues(&targets[14].mValue),
                targets[14].mAuxiliaryTargetData);
  setTargets_16(rtdv, sm_core_SmRealVector_nonConstValues(&targets[16].mValue),
                targets[16].mAuxiliaryTargetData);
  setTargets_18(rtdv, sm_core_SmRealVector_nonConstValues(&targets[18].mValue),
                targets[18].mAuxiliaryTargetData);
  setTargets_20(rtdv, sm_core_SmRealVector_nonConstValues(&targets[20].mValue),
                targets[20].mAuxiliaryTargetData);
  setTargets_22(rtdv, sm_core_SmRealVector_nonConstValues(&targets[22].mValue),
                targets[22].mAuxiliaryTargetData);
}

void FD_cd27307_1_resetAsmStateVector(const void *mech, double *state)
{
  double xx[1];
  (void) mech;
  xx[0] = 0.0;
  state[0] = xx[0];
  state[1] = xx[0];
  state[2] = xx[0];
  state[3] = xx[0];
  state[4] = xx[0];
  state[5] = xx[0];
  state[6] = xx[0];
  state[7] = xx[0];
  state[8] = xx[0];
  state[9] = xx[0];
  state[10] = xx[0];
  state[11] = xx[0];
  state[12] = xx[0];
  state[13] = xx[0];
  state[14] = xx[0];
  state[15] = xx[0];
  state[16] = xx[0];
  state[17] = xx[0];
  state[18] = xx[0];
  state[19] = xx[0];
  state[20] = xx[0];
  state[21] = xx[0];
  state[22] = xx[0];
  state[23] = xx[0];
}

void FD_cd27307_1_initializeTrackedAngleState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *modeVector, const double
  *motionData, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
}

void FD_cd27307_1_computeDiscreteState(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const int *modeVector, double *state)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
}

void FD_cd27307_1_adjustPosition(const void *mech, const double *dofDeltas,
  double *state)
{
  (void) mech;
  state[0] = state[0] + dofDeltas[0];
  state[2] = state[2] + dofDeltas[1];
  state[4] = state[4] + dofDeltas[2];
  state[6] = state[6] + dofDeltas[3];
  state[8] = state[8] + dofDeltas[4];
  state[10] = state[10] + dofDeltas[5];
  state[12] = state[12] + dofDeltas[6];
  state[14] = state[14] + dofDeltas[7];
  state[16] = state[16] + dofDeltas[8];
  state[18] = state[18] + dofDeltas[9];
  state[20] = state[20] + dofDeltas[10];
  state[22] = state[22] + dofDeltas[11];
}

static void perturbAsmJointPrimitiveState_2_0(double mag, double *state)
{
  state[0] = state[0] + mag;
}

static void perturbAsmJointPrimitiveState_2_0v(double mag, double *state)
{
  state[0] = state[0] + mag;
  state[1] = state[1] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_3_0(double mag, double *state)
{
  state[2] = state[2] + mag;
}

static void perturbAsmJointPrimitiveState_3_0v(double mag, double *state)
{
  state[2] = state[2] + mag;
  state[3] = state[3] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_4_0(double mag, double *state)
{
  state[4] = state[4] + mag;
}

static void perturbAsmJointPrimitiveState_4_0v(double mag, double *state)
{
  state[4] = state[4] + mag;
  state[5] = state[5] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_5_0(double mag, double *state)
{
  state[6] = state[6] + mag;
}

static void perturbAsmJointPrimitiveState_5_0v(double mag, double *state)
{
  state[6] = state[6] + mag;
  state[7] = state[7] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_6_0(double mag, double *state)
{
  state[8] = state[8] + mag;
}

static void perturbAsmJointPrimitiveState_6_0v(double mag, double *state)
{
  state[8] = state[8] + mag;
  state[9] = state[9] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_7_0(double mag, double *state)
{
  state[10] = state[10] + mag;
}

static void perturbAsmJointPrimitiveState_7_0v(double mag, double *state)
{
  state[10] = state[10] + mag;
  state[11] = state[11] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_10_0(double mag, double *state)
{
  state[12] = state[12] + mag;
}

static void perturbAsmJointPrimitiveState_10_0v(double mag, double *state)
{
  state[12] = state[12] + mag;
  state[13] = state[13] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_11_0(double mag, double *state)
{
  state[14] = state[14] + mag;
}

static void perturbAsmJointPrimitiveState_11_0v(double mag, double *state)
{
  state[14] = state[14] + mag;
  state[15] = state[15] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_12_0(double mag, double *state)
{
  state[16] = state[16] + mag;
}

static void perturbAsmJointPrimitiveState_12_0v(double mag, double *state)
{
  state[16] = state[16] + mag;
  state[17] = state[17] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_13_0(double mag, double *state)
{
  state[18] = state[18] + mag;
}

static void perturbAsmJointPrimitiveState_13_0v(double mag, double *state)
{
  state[18] = state[18] + mag;
  state[19] = state[19] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_14_0(double mag, double *state)
{
  state[20] = state[20] + mag;
}

static void perturbAsmJointPrimitiveState_14_0v(double mag, double *state)
{
  state[20] = state[20] + mag;
  state[21] = state[21] - 0.875 * mag;
}

static void perturbAsmJointPrimitiveState_15_0(double mag, double *state)
{
  state[22] = state[22] + mag;
}

static void perturbAsmJointPrimitiveState_15_0v(double mag, double *state)
{
  state[22] = state[22] + mag;
  state[23] = state[23] - 0.875 * mag;
}

void FD_cd27307_1_perturbAsmJointPrimitiveState(const void *mech, size_t
  stageIdx, size_t primIdx, double mag, boolean_T doPerturbVelocity, double
  *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) mag;
  (void) doPerturbVelocity;
  (void) state;
  switch ((stageIdx * 6 + primIdx) * 2 + (doPerturbVelocity ? 1 : 0))
  {
   case 24:
    perturbAsmJointPrimitiveState_2_0(mag, state);
    break;

   case 25:
    perturbAsmJointPrimitiveState_2_0v(mag, state);
    break;

   case 36:
    perturbAsmJointPrimitiveState_3_0(mag, state);
    break;

   case 37:
    perturbAsmJointPrimitiveState_3_0v(mag, state);
    break;

   case 48:
    perturbAsmJointPrimitiveState_4_0(mag, state);
    break;

   case 49:
    perturbAsmJointPrimitiveState_4_0v(mag, state);
    break;

   case 60:
    perturbAsmJointPrimitiveState_5_0(mag, state);
    break;

   case 61:
    perturbAsmJointPrimitiveState_5_0v(mag, state);
    break;

   case 72:
    perturbAsmJointPrimitiveState_6_0(mag, state);
    break;

   case 73:
    perturbAsmJointPrimitiveState_6_0v(mag, state);
    break;

   case 84:
    perturbAsmJointPrimitiveState_7_0(mag, state);
    break;

   case 85:
    perturbAsmJointPrimitiveState_7_0v(mag, state);
    break;

   case 120:
    perturbAsmJointPrimitiveState_10_0(mag, state);
    break;

   case 121:
    perturbAsmJointPrimitiveState_10_0v(mag, state);
    break;

   case 132:
    perturbAsmJointPrimitiveState_11_0(mag, state);
    break;

   case 133:
    perturbAsmJointPrimitiveState_11_0v(mag, state);
    break;

   case 144:
    perturbAsmJointPrimitiveState_12_0(mag, state);
    break;

   case 145:
    perturbAsmJointPrimitiveState_12_0v(mag, state);
    break;

   case 156:
    perturbAsmJointPrimitiveState_13_0(mag, state);
    break;

   case 157:
    perturbAsmJointPrimitiveState_13_0v(mag, state);
    break;

   case 168:
    perturbAsmJointPrimitiveState_14_0(mag, state);
    break;

   case 169:
    perturbAsmJointPrimitiveState_14_0v(mag, state);
    break;

   case 180:
    perturbAsmJointPrimitiveState_15_0(mag, state);
    break;

   case 181:
    perturbAsmJointPrimitiveState_15_0v(mag, state);
    break;
  }
}

void FD_cd27307_1_computePosDofBlendMatrix(const void *mech, size_t stageIdx,
  size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void FD_cd27307_1_computeVelDofBlendMatrix(const void *mech, size_t stageIdx,
  size_t primIdx, const double *state, int partialType, double *matrix)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) state;
  (void) partialType;
  (void) matrix;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void FD_cd27307_1_projectPartiallyTargetedPos(const void *mech, size_t stageIdx,
  size_t primIdx, const double *origState, int partialType, double *state)
{
  (void) mech;
  (void) stageIdx;
  (void) primIdx;
  (void) origState;
  (void) partialType;
  (void) state;
  switch ((stageIdx * 6 + primIdx))
  {
  }
}

void FD_cd27307_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[269];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = 0.0;
  xx[1] = - 0.970673997023057;
  xx[2] = 0.2333331049091575;
  xx[3] = - 0.02952992534078491;
  xx[4] = - 0.0497557752035139;
  xx[5] = 0.5;
  xx[6] = xx[5] * state[0];
  xx[7] = 0.05190792756592435;
  xx[8] = sin(xx[6]);
  xx[9] = 0.4534718248765359;
  xx[10] = 0.8897577597857493;
  xx[11] = cos(xx[6]);
  xx[12] = - (xx[7] * xx[8]);
  xx[13] = - (xx[9] * xx[8]);
  xx[14] = xx[10] * xx[8];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 11, xx + 15);
  xx[11] = - 0.2990853783794172;
  xx[12] = - 0.660803415531852;
  xx[13] = - 0.5376259398076141;
  xx[14] = 0.4299361944603635;
  xx[6] = xx[5] * state[2];
  xx[8] = 8.980391557070891e-5;
  xx[19] = sin(xx[6]);
  xx[20] = 0.1473220231652822;
  xx[21] = 0.9890885771384406;
  xx[22] = cos(xx[6]);
  xx[23] = xx[8] * xx[19];
  xx[24] = xx[20] * xx[19];
  xx[25] = xx[21] * xx[19];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 22, xx + 26);
  xx[6] = - 0.0225816401000646;
  xx[19] = - 0.1970218576898142;
  xx[22] = 0.147775021985604;
  xx[30] = - 0.9989949449774413;
  xx[31] = 0.04261109427110332;
  xx[32] = - 5.10759332169909e-4;
  xx[33] = - 0.01389725438503987;
  xx[23] = xx[5] * state[4];
  xx[24] = 1.993195962123751e-3;
  xx[25] = sin(xx[23]);
  xx[34] = 0.06253409983738777;
  xx[35] = 0.9980408376050472;
  xx[36] = cos(xx[23]);
  xx[37] = xx[24] * xx[25];
  xx[38] = xx[34] * xx[25];
  xx[39] = xx[35] * xx[25];
  pm_math_Quaternion_compose_ra(xx + 30, xx + 36, xx + 40);
  xx[23] = - 0.4500027249445239;
  xx[25] = - 4.383470068433541e-3;
  xx[36] = - 0.03014270612706048;
  xx[44] = - 0.5680366360873395;
  xx[45] = 0.4357610792850483;
  xx[46] = 0.3894605422737099;
  xx[47] = - 0.5794541809796333;
  xx[37] = xx[5] * state[6];
  xx[38] = 2.96940332536416e-3;
  xx[39] = sin(xx[37]);
  xx[48] = 0.3169967519577546;
  xx[49] = 0.9484219745936537;
  xx[50] = cos(xx[37]);
  xx[51] = xx[38] * xx[39];
  xx[52] = xx[48] * xx[39];
  xx[53] = xx[49] * xx[39];
  pm_math_Quaternion_compose_ra(xx + 44, xx + 50, xx + 54);
  xx[37] = - 0.2670444337519779;
  xx[39] = 2.798919062483627e-3;
  xx[50] = - 0.07478928034262089;
  xx[58] = - 0.5969110954493415;
  xx[59] = - 0.3731944491449939;
  xx[60] = - 0.4511361566395476;
  xx[61] = 0.5485428109357824;
  xx[51] = xx[5] * state[8];
  xx[52] = 1.216154953217517e-3;
  xx[53] = sin(xx[51]);
  xx[62] = 0.1290362120704933;
  xx[63] = 0.9916391364511732;
  xx[64] = cos(xx[51]);
  xx[65] = xx[52] * xx[53];
  xx[66] = - (xx[62] * xx[53]);
  xx[67] = xx[63] * xx[53];
  pm_math_Quaternion_compose_ra(xx + 58, xx + 64, xx + 68);
  xx[51] = 2.479139776831147e-4;
  xx[53] = - 0.08180985272046187;
  xx[64] = 0.1148568088382922;
  xx[72] = - 0.5321891960505517;
  xx[73] = 0.4623445245673553;
  xx[74] = 0.5363301322358599;
  xx[75] = - 0.4640713193739929;
  xx[65] = xx[5] * state[10];
  xx[66] = 1.647994705501673e-4;
  xx[67] = sin(xx[65]);
  xx[76] = 3.37884491615367e-3;
  xx[77] = 0.9999942781077135;
  xx[78] = cos(xx[65]);
  xx[79] = xx[66] * xx[67];
  xx[80] = - (xx[76] * xx[67]);
  xx[81] = xx[77] * xx[67];
  pm_math_Quaternion_compose_ra(xx + 72, xx + 78, xx + 82);
  xx[65] = - 0.1679129639661445;
  xx[67] = - 9.92497576990382e-3;
  xx[78] = 0.06849621440686403;
  xx[79] = - 0.9999780459768537;
  xx[80] = - 1.689918118835772e-3;
  xx[81] = - 7.157440094880491e-5;
  xx[86] = - 6.4067634707608e-3;
  xx[87] = 9.887968233010034e-6;
  xx[88] = - 2.027306949692202e-4;
  xx[89] = 0.05999965668646281;
  xx[90] = xx[5] * state[12];
  xx[91] = sin(xx[90]);
  xx[92] = cos(xx[90]);
  xx[93] = - (xx[7] * xx[91]);
  xx[94] = - (xx[9] * xx[91]);
  xx[95] = xx[10] * xx[91];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 92, xx + 96);
  xx[1] = xx[5] * state[14];
  xx[2] = sin(xx[1]);
  xx[90] = cos(xx[1]);
  xx[91] = xx[8] * xx[2];
  xx[92] = xx[20] * xx[2];
  xx[93] = xx[21] * xx[2];
  pm_math_Quaternion_compose_ra(xx + 11, xx + 90, xx + 1);
  xx[11] = xx[5] * state[16];
  xx[12] = sin(xx[11]);
  xx[90] = cos(xx[11]);
  xx[91] = xx[24] * xx[12];
  xx[92] = xx[34] * xx[12];
  xx[93] = xx[35] * xx[12];
  pm_math_Quaternion_compose_ra(xx + 30, xx + 90, xx + 11);
  xx[30] = xx[5] * state[18];
  xx[31] = sin(xx[30]);
  xx[90] = cos(xx[30]);
  xx[91] = xx[38] * xx[31];
  xx[92] = xx[48] * xx[31];
  xx[93] = xx[49] * xx[31];
  pm_math_Quaternion_compose_ra(xx + 44, xx + 90, xx + 30);
  xx[44] = xx[5] * state[20];
  xx[45] = sin(xx[44]);
  xx[90] = cos(xx[44]);
  xx[91] = xx[52] * xx[45];
  xx[92] = - (xx[62] * xx[45]);
  xx[93] = xx[63] * xx[45];
  pm_math_Quaternion_compose_ra(xx + 58, xx + 90, xx + 44);
  xx[58] = xx[5] * state[22];
  xx[5] = sin(xx[58]);
  xx[90] = cos(xx[58]);
  xx[91] = xx[66] * xx[5];
  xx[92] = - (xx[76] * xx[5]);
  xx[93] = xx[77] * xx[5];
  pm_math_Quaternion_compose_ra(xx + 72, xx + 90, xx + 58);
  xx[5] = 7.771333802811866e-3;
  xx[72] = 0.01535402110476336;
  xx[73] = 0.8649924090759147;
  xx[74] = 0.5014897732236276;
  xx[75] = 0.1551;
  xx[90] = 1.2924;
  xx[91] = xx[5];
  xx[92] = xx[72];
  xx[93] = xx[73];
  xx[94] = xx[74];
  pm_math_Quaternion_compose_ra(xx + 91, xx + 15, xx + 100);
  xx[91] = 4.537052817905202e-8;
  xx[92] = 0.2245174368406531;
  xx[93] = 1.252635449675976;
  pm_math_Quaternion_compose_ra(xx + 100, xx + 26, xx + 104);
  xx[108] = xx[6];
  xx[109] = xx[19];
  xx[110] = xx[22];
  pm_math_Quaternion_xform_ra(xx + 100, xx + 108, xx + 111);
  xx[94] = xx[111] + xx[91];
  xx[95] = xx[112] + xx[92];
  xx[111] = xx[113] + xx[93];
  pm_math_Quaternion_compose_ra(xx + 104, xx + 40, xx + 112);
  xx[116] = xx[23];
  xx[117] = xx[25];
  xx[118] = xx[36];
  pm_math_Quaternion_xform_ra(xx + 104, xx + 116, xx + 119);
  xx[122] = xx[119] + xx[94];
  xx[123] = xx[120] + xx[95];
  xx[119] = xx[121] + xx[111];
  pm_math_Quaternion_compose_ra(xx + 112, xx + 54, xx + 124);
  xx[128] = xx[37];
  xx[129] = xx[39];
  xx[130] = xx[50];
  pm_math_Quaternion_xform_ra(xx + 112, xx + 128, xx + 131);
  xx[120] = xx[131] + xx[122];
  xx[121] = xx[132] + xx[123];
  xx[131] = xx[133] + xx[119];
  pm_math_Quaternion_compose_ra(xx + 124, xx + 68, xx + 132);
  xx[136] = xx[51];
  xx[137] = xx[53];
  xx[138] = xx[64];
  pm_math_Quaternion_xform_ra(xx + 124, xx + 136, xx + 139);
  xx[142] = xx[139] + xx[120];
  xx[143] = xx[140] + xx[121];
  xx[139] = xx[141] + xx[131];
  pm_math_Quaternion_compose_ra(xx + 132, xx + 82, xx + 144);
  xx[148] = xx[65];
  xx[149] = xx[67];
  xx[150] = xx[78];
  pm_math_Quaternion_xform_ra(xx + 132, xx + 148, xx + 151);
  xx[140] = xx[151] + xx[142];
  xx[141] = xx[152] + xx[143];
  xx[151] = xx[153] + xx[139];
  xx[152] = xx[79];
  xx[153] = xx[80];
  xx[154] = xx[81];
  xx[155] = xx[86];
  pm_math_Quaternion_compose_ra(xx + 144, xx + 152, xx + 156);
  pm_math_Quaternion_xform_ra(xx + 144, xx + 87, xx + 160);
  xx[163] = 0.5014897757632328;
  xx[164] = 0.8649924140934843;
  xx[165] = - 0.01535373842965771;
  xx[166] = - 7.771169918512899e-3;
  pm_math_Quaternion_compose_ra(xx + 163, xx + 96, xx + 167);
  xx[171] = 8.673617379884035e-19;
  xx[172] = 0.222348141939397;
  xx[173] = 1.253878091873601;
  pm_math_Quaternion_compose_ra(xx + 167, xx + 1, xx + 174);
  pm_math_Quaternion_xform_ra(xx + 167, xx + 108, xx + 178);
  xx[181] = xx[178] + xx[171];
  xx[182] = xx[179] - xx[172];
  xx[178] = xx[180] + xx[173];
  pm_math_Quaternion_compose_ra(xx + 174, xx + 11, xx + 183);
  pm_math_Quaternion_xform_ra(xx + 174, xx + 116, xx + 187);
  xx[179] = xx[187] + xx[181];
  xx[180] = xx[188] + xx[182];
  xx[187] = xx[189] + xx[178];
  pm_math_Quaternion_compose_ra(xx + 183, xx + 30, xx + 188);
  pm_math_Quaternion_xform_ra(xx + 183, xx + 128, xx + 192);
  xx[195] = xx[192] + xx[179];
  xx[196] = xx[193] + xx[180];
  xx[192] = xx[194] + xx[187];
  pm_math_Quaternion_compose_ra(xx + 188, xx + 44, xx + 197);
  pm_math_Quaternion_xform_ra(xx + 188, xx + 136, xx + 201);
  xx[193] = xx[201] + xx[195];
  xx[194] = xx[202] + xx[196];
  xx[201] = xx[203] + xx[192];
  pm_math_Quaternion_compose_ra(xx + 197, xx + 58, xx + 202);
  pm_math_Quaternion_xform_ra(xx + 197, xx + 148, xx + 206);
  xx[209] = xx[206] + xx[193];
  xx[210] = xx[207] + xx[194];
  xx[206] = xx[208] + xx[201];
  pm_math_Quaternion_compose_ra(xx + 202, xx + 152, xx + 211);
  pm_math_Quaternion_xform_ra(xx + 202, xx + 87, xx + 215);
  xx[207] = - (xx[7] * state[1]);
  xx[208] = - (xx[9] * state[1]);
  xx[218] = xx[10] * state[1];
  xx[219] = xx[207];
  xx[220] = xx[208];
  xx[221] = xx[218];
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 219, xx + 222);
  xx[225] = xx[222] + xx[8] * state[3];
  xx[226] = xx[223] + xx[20] * state[3];
  xx[222] = xx[224] + xx[21] * state[3];
  pm_math_Vector3_cross_ra(xx + 219, xx + 108, xx + 227);
  pm_math_Quaternion_inverseXform_ra(xx + 26, xx + 227, xx + 219);
  xx[227] = xx[225];
  xx[228] = xx[226];
  xx[229] = xx[222];
  pm_math_Quaternion_inverseXform_ra(xx + 40, xx + 227, xx + 230);
  xx[223] = xx[230] + xx[24] * state[5];
  xx[224] = xx[231] + xx[34] * state[5];
  xx[230] = xx[232] + xx[35] * state[5];
  pm_math_Vector3_cross_ra(xx + 227, xx + 116, xx + 231);
  xx[227] = xx[231] + xx[219];
  xx[228] = xx[232] + xx[220];
  xx[229] = xx[233] + xx[221];
  pm_math_Quaternion_inverseXform_ra(xx + 40, xx + 227, xx + 231);
  xx[227] = xx[223];
  xx[228] = xx[224];
  xx[229] = xx[230];
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 227, xx + 234);
  xx[237] = xx[234] + xx[38] * state[7];
  xx[238] = xx[235] + xx[48] * state[7];
  xx[234] = xx[236] + xx[49] * state[7];
  pm_math_Vector3_cross_ra(xx + 227, xx + 128, xx + 239);
  xx[227] = xx[239] + xx[231];
  xx[228] = xx[240] + xx[232];
  xx[229] = xx[241] + xx[233];
  pm_math_Quaternion_inverseXform_ra(xx + 54, xx + 227, xx + 239);
  xx[227] = xx[237];
  xx[228] = xx[238];
  xx[229] = xx[234];
  pm_math_Quaternion_inverseXform_ra(xx + 68, xx + 227, xx + 242);
  xx[235] = xx[242] + xx[52] * state[9];
  xx[236] = xx[243] - xx[62] * state[9];
  xx[242] = xx[244] + xx[63] * state[9];
  pm_math_Vector3_cross_ra(xx + 227, xx + 136, xx + 243);
  xx[227] = xx[243] + xx[239];
  xx[228] = xx[244] + xx[240];
  xx[229] = xx[245] + xx[241];
  pm_math_Quaternion_inverseXform_ra(xx + 68, xx + 227, xx + 243);
  xx[227] = xx[235];
  xx[228] = xx[236];
  xx[229] = xx[242];
  pm_math_Quaternion_inverseXform_ra(xx + 82, xx + 227, xx + 246);
  xx[249] = xx[246] + xx[66] * state[11];
  xx[250] = xx[247] - xx[76] * state[11];
  xx[246] = xx[248] + xx[77] * state[11];
  pm_math_Vector3_cross_ra(xx + 227, xx + 148, xx + 251);
  xx[227] = xx[251] + xx[243];
  xx[228] = xx[252] + xx[244];
  xx[229] = xx[253] + xx[245];
  pm_math_Quaternion_inverseXform_ra(xx + 82, xx + 227, xx + 251);
  xx[227] = xx[249];
  xx[228] = xx[250];
  xx[229] = xx[246];
  pm_math_Quaternion_inverseXform_ra(xx + 152, xx + 227, xx + 254);
  pm_math_Vector3_cross_ra(xx + 227, xx + 87, xx + 257);
  xx[227] = xx[257] + xx[251];
  xx[228] = xx[258] + xx[252];
  xx[229] = xx[259] + xx[253];
  pm_math_Quaternion_inverseXform_ra(xx + 152, xx + 227, xx + 257);
  xx[227] = - (xx[7] * state[13]);
  xx[7] = - (xx[9] * state[13]);
  xx[9] = xx[10] * state[13];
  xx[260] = xx[227];
  xx[261] = xx[7];
  xx[262] = xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 260, xx + 263);
  xx[10] = xx[263] + xx[8] * state[15];
  xx[8] = xx[264] + xx[20] * state[15];
  xx[20] = xx[265] + xx[21] * state[15];
  pm_math_Vector3_cross_ra(xx + 260, xx + 108, xx + 263);
  pm_math_Quaternion_inverseXform_ra(xx + 1, xx + 263, xx + 108);
  xx[260] = xx[10];
  xx[261] = xx[8];
  xx[262] = xx[20];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 260, xx + 263);
  xx[21] = xx[263] + xx[24] * state[17];
  xx[24] = xx[264] + xx[34] * state[17];
  xx[34] = xx[265] + xx[35] * state[17];
  pm_math_Vector3_cross_ra(xx + 260, xx + 116, xx + 263);
  xx[116] = xx[263] + xx[108];
  xx[117] = xx[264] + xx[109];
  xx[118] = xx[265] + xx[110];
  pm_math_Quaternion_inverseXform_ra(xx + 11, xx + 116, xx + 260);
  xx[116] = xx[21];
  xx[117] = xx[24];
  xx[118] = xx[34];
  pm_math_Quaternion_inverseXform_ra(xx + 30, xx + 116, xx + 263);
  xx[35] = xx[263] + xx[38] * state[19];
  xx[38] = xx[264] + xx[48] * state[19];
  xx[48] = xx[265] + xx[49] * state[19];
  pm_math_Vector3_cross_ra(xx + 116, xx + 128, xx + 263);
  xx[116] = xx[263] + xx[260];
  xx[117] = xx[264] + xx[261];
  xx[118] = xx[265] + xx[262];
  pm_math_Quaternion_inverseXform_ra(xx + 30, xx + 116, xx + 128);
  xx[116] = xx[35];
  xx[117] = xx[38];
  xx[118] = xx[48];
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 116, xx + 263);
  xx[49] = xx[263] + xx[52] * state[21];
  xx[52] = xx[264] - xx[62] * state[21];
  xx[62] = xx[265] + xx[63] * state[21];
  pm_math_Vector3_cross_ra(xx + 116, xx + 136, xx + 263);
  xx[116] = xx[263] + xx[128];
  xx[117] = xx[264] + xx[129];
  xx[118] = xx[265] + xx[130];
  pm_math_Quaternion_inverseXform_ra(xx + 44, xx + 116, xx + 136);
  xx[116] = xx[49];
  xx[117] = xx[52];
  xx[118] = xx[62];
  pm_math_Quaternion_inverseXform_ra(xx + 58, xx + 116, xx + 263);
  xx[63] = xx[263] + xx[66] * state[23];
  xx[66] = xx[264] - xx[76] * state[23];
  xx[76] = xx[265] + xx[77] * state[23];
  pm_math_Vector3_cross_ra(xx + 116, xx + 148, xx + 263);
  xx[116] = xx[263] + xx[136];
  xx[117] = xx[264] + xx[137];
  xx[118] = xx[265] + xx[138];
  pm_math_Quaternion_inverseXform_ra(xx + 58, xx + 116, xx + 148);
  xx[116] = xx[63];
  xx[117] = xx[66];
  xx[118] = xx[76];
  pm_math_Quaternion_inverseXform_ra(xx + 152, xx + 116, xx + 263);
  pm_math_Vector3_cross_ra(xx + 116, xx + 87, xx + 266);
  xx[116] = xx[266] + xx[148];
  xx[117] = xx[267] + xx[149];
  xx[118] = xx[268] + xx[150];
  pm_math_Quaternion_inverseXform_ra(xx + 152, xx + 116, xx + 266);
  motionData[0] = - 0.999851918871201;
  motionData[1] = 1.093796538566551e-4;
  motionData[2] = 0.01442285491200129;
  motionData[3] = - 9.386672565723609e-3;
  motionData[4] = xx[0];
  motionData[5] = xx[0];
  motionData[6] = xx[0];
  motionData[7] = - 1.638776411230433e-7;
  motionData[8] = - 0.03070491224467679;
  motionData[9] = - 0.8647774288970013;
  motionData[10] = - 0.5012157288377052;
  motionData[11] = 0.04018382505706871;
  motionData[12] = 0.1544400450337887;
  motionData[13] = 1.291854211083334;
  motionData[14] = xx[15];
  motionData[15] = xx[16];
  motionData[16] = xx[17];
  motionData[17] = xx[18];
  motionData[18] = 2.307150791739089e-3;
  motionData[19] = - 3.915937173055021e-5;
  motionData[20] = 0.07996671508676459;
  motionData[21] = xx[26];
  motionData[22] = xx[27];
  motionData[23] = xx[28];
  motionData[24] = xx[29];
  motionData[25] = xx[6];
  motionData[26] = xx[19];
  motionData[27] = xx[22];
  motionData[28] = xx[40];
  motionData[29] = xx[41];
  motionData[30] = xx[42];
  motionData[31] = xx[43];
  motionData[32] = xx[23];
  motionData[33] = xx[25];
  motionData[34] = xx[36];
  motionData[35] = xx[54];
  motionData[36] = xx[55];
  motionData[37] = xx[56];
  motionData[38] = xx[57];
  motionData[39] = xx[37];
  motionData[40] = xx[39];
  motionData[41] = xx[50];
  motionData[42] = xx[68];
  motionData[43] = xx[69];
  motionData[44] = xx[70];
  motionData[45] = xx[71];
  motionData[46] = xx[51];
  motionData[47] = xx[53];
  motionData[48] = xx[64];
  motionData[49] = xx[82];
  motionData[50] = xx[83];
  motionData[51] = xx[84];
  motionData[52] = xx[85];
  motionData[53] = xx[65];
  motionData[54] = xx[67];
  motionData[55] = xx[78];
  motionData[56] = xx[79];
  motionData[57] = xx[80];
  motionData[58] = xx[81];
  motionData[59] = xx[86];
  motionData[60] = xx[87];
  motionData[61] = xx[88];
  motionData[62] = xx[89];
  motionData[63] = - 0.5014694013345645;
  motionData[64] = - 0.8646629748469581;
  motionData[65] = 0.01623710111010919;
  motionData[66] = 0.0249546789500938;
  motionData[67] = 0.03436021702402126;
  motionData[68] = - 0.1557052843344028;
  motionData[69] = 1.29187035337003;
  motionData[70] = xx[96];
  motionData[71] = xx[97];
  motionData[72] = xx[98];
  motionData[73] = xx[99];
  motionData[74] = 2.235052329497243e-3;
  motionData[75] = - 3.793564136397051e-5;
  motionData[76] = 0.07746775524030319;
  motionData[77] = xx[1];
  motionData[78] = xx[2];
  motionData[79] = xx[3];
  motionData[80] = xx[4];
  motionData[81] = xx[6];
  motionData[82] = xx[19];
  motionData[83] = xx[22];
  motionData[84] = xx[11];
  motionData[85] = xx[12];
  motionData[86] = xx[13];
  motionData[87] = xx[14];
  motionData[88] = xx[23];
  motionData[89] = xx[25];
  motionData[90] = xx[36];
  motionData[91] = xx[30];
  motionData[92] = xx[31];
  motionData[93] = xx[32];
  motionData[94] = xx[33];
  motionData[95] = xx[37];
  motionData[96] = xx[39];
  motionData[97] = xx[50];
  motionData[98] = xx[44];
  motionData[99] = xx[45];
  motionData[100] = xx[46];
  motionData[101] = xx[47];
  motionData[102] = xx[51];
  motionData[103] = xx[53];
  motionData[104] = xx[64];
  motionData[105] = xx[58];
  motionData[106] = xx[59];
  motionData[107] = xx[60];
  motionData[108] = xx[61];
  motionData[109] = xx[65];
  motionData[110] = xx[67];
  motionData[111] = xx[78];
  motionData[112] = xx[79];
  motionData[113] = xx[80];
  motionData[114] = xx[81];
  motionData[115] = xx[86];
  motionData[116] = xx[87];
  motionData[117] = xx[88];
  motionData[118] = xx[89];
  motionData[119] = xx[5];
  motionData[120] = xx[72];
  motionData[121] = xx[73];
  motionData[122] = xx[74];
  motionData[123] = xx[0];
  motionData[124] = xx[75];
  motionData[125] = xx[90];
  motionData[126] = xx[100];
  motionData[127] = xx[101];
  motionData[128] = xx[102];
  motionData[129] = xx[103];
  motionData[130] = xx[91];
  motionData[131] = xx[92];
  motionData[132] = xx[93];
  motionData[133] = xx[104];
  motionData[134] = xx[105];
  motionData[135] = xx[106];
  motionData[136] = xx[107];
  motionData[137] = xx[94];
  motionData[138] = xx[95];
  motionData[139] = xx[111];
  motionData[140] = xx[112];
  motionData[141] = xx[113];
  motionData[142] = xx[114];
  motionData[143] = xx[115];
  motionData[144] = xx[122];
  motionData[145] = xx[123];
  motionData[146] = xx[119];
  motionData[147] = xx[124];
  motionData[148] = xx[125];
  motionData[149] = xx[126];
  motionData[150] = xx[127];
  motionData[151] = xx[120];
  motionData[152] = xx[121];
  motionData[153] = xx[131];
  motionData[154] = xx[132];
  motionData[155] = xx[133];
  motionData[156] = xx[134];
  motionData[157] = xx[135];
  motionData[158] = xx[142];
  motionData[159] = xx[143];
  motionData[160] = xx[139];
  motionData[161] = xx[144];
  motionData[162] = xx[145];
  motionData[163] = xx[146];
  motionData[164] = xx[147];
  motionData[165] = xx[140];
  motionData[166] = xx[141];
  motionData[167] = xx[151];
  motionData[168] = xx[156];
  motionData[169] = xx[157];
  motionData[170] = xx[158];
  motionData[171] = xx[159];
  motionData[172] = xx[160] + xx[140];
  motionData[173] = xx[161] + xx[141];
  motionData[174] = xx[162] + xx[151];
  motionData[175] = xx[163];
  motionData[176] = xx[164];
  motionData[177] = xx[165];
  motionData[178] = xx[166];
  motionData[179] = xx[0];
  motionData[180] = - xx[75];
  motionData[181] = xx[90];
  motionData[182] = xx[167];
  motionData[183] = xx[168];
  motionData[184] = xx[169];
  motionData[185] = xx[170];
  motionData[186] = xx[171];
  motionData[187] = - xx[172];
  motionData[188] = xx[173];
  motionData[189] = xx[174];
  motionData[190] = xx[175];
  motionData[191] = xx[176];
  motionData[192] = xx[177];
  motionData[193] = xx[181];
  motionData[194] = xx[182];
  motionData[195] = xx[178];
  motionData[196] = xx[183];
  motionData[197] = xx[184];
  motionData[198] = xx[185];
  motionData[199] = xx[186];
  motionData[200] = xx[179];
  motionData[201] = xx[180];
  motionData[202] = xx[187];
  motionData[203] = xx[188];
  motionData[204] = xx[189];
  motionData[205] = xx[190];
  motionData[206] = xx[191];
  motionData[207] = xx[195];
  motionData[208] = xx[196];
  motionData[209] = xx[192];
  motionData[210] = xx[197];
  motionData[211] = xx[198];
  motionData[212] = xx[199];
  motionData[213] = xx[200];
  motionData[214] = xx[193];
  motionData[215] = xx[194];
  motionData[216] = xx[201];
  motionData[217] = xx[202];
  motionData[218] = xx[203];
  motionData[219] = xx[204];
  motionData[220] = xx[205];
  motionData[221] = xx[209];
  motionData[222] = xx[210];
  motionData[223] = xx[206];
  motionData[224] = xx[211];
  motionData[225] = xx[212];
  motionData[226] = xx[213];
  motionData[227] = xx[214];
  motionData[228] = xx[215] + xx[209];
  motionData[229] = xx[216] + xx[210];
  motionData[230] = xx[217] + xx[206];
  motionData[231] = xx[0];
  motionData[232] = xx[0];
  motionData[233] = xx[0];
  motionData[234] = xx[0];
  motionData[235] = xx[0];
  motionData[236] = xx[0];
  motionData[237] = xx[0];
  motionData[238] = xx[0];
  motionData[239] = xx[0];
  motionData[240] = xx[0];
  motionData[241] = xx[0];
  motionData[242] = xx[0];
  motionData[243] = xx[207];
  motionData[244] = xx[208];
  motionData[245] = xx[218];
  motionData[246] = xx[0];
  motionData[247] = xx[0];
  motionData[248] = xx[0];
  motionData[249] = xx[225];
  motionData[250] = xx[226];
  motionData[251] = xx[222];
  motionData[252] = xx[219];
  motionData[253] = xx[220];
  motionData[254] = xx[221];
  motionData[255] = xx[223];
  motionData[256] = xx[224];
  motionData[257] = xx[230];
  motionData[258] = xx[231];
  motionData[259] = xx[232];
  motionData[260] = xx[233];
  motionData[261] = xx[237];
  motionData[262] = xx[238];
  motionData[263] = xx[234];
  motionData[264] = xx[239];
  motionData[265] = xx[240];
  motionData[266] = xx[241];
  motionData[267] = xx[235];
  motionData[268] = xx[236];
  motionData[269] = xx[242];
  motionData[270] = xx[243];
  motionData[271] = xx[244];
  motionData[272] = xx[245];
  motionData[273] = xx[249];
  motionData[274] = xx[250];
  motionData[275] = xx[246];
  motionData[276] = xx[251];
  motionData[277] = xx[252];
  motionData[278] = xx[253];
  motionData[279] = xx[254];
  motionData[280] = xx[255];
  motionData[281] = xx[256];
  motionData[282] = xx[257];
  motionData[283] = xx[258];
  motionData[284] = xx[259];
  motionData[285] = xx[0];
  motionData[286] = xx[0];
  motionData[287] = xx[0];
  motionData[288] = xx[0];
  motionData[289] = xx[0];
  motionData[290] = xx[0];
  motionData[291] = xx[227];
  motionData[292] = xx[7];
  motionData[293] = xx[9];
  motionData[294] = xx[0];
  motionData[295] = xx[0];
  motionData[296] = xx[0];
  motionData[297] = xx[10];
  motionData[298] = xx[8];
  motionData[299] = xx[20];
  motionData[300] = xx[108];
  motionData[301] = xx[109];
  motionData[302] = xx[110];
  motionData[303] = xx[21];
  motionData[304] = xx[24];
  motionData[305] = xx[34];
  motionData[306] = xx[260];
  motionData[307] = xx[261];
  motionData[308] = xx[262];
  motionData[309] = xx[35];
  motionData[310] = xx[38];
  motionData[311] = xx[48];
  motionData[312] = xx[128];
  motionData[313] = xx[129];
  motionData[314] = xx[130];
  motionData[315] = xx[49];
  motionData[316] = xx[52];
  motionData[317] = xx[62];
  motionData[318] = xx[136];
  motionData[319] = xx[137];
  motionData[320] = xx[138];
  motionData[321] = xx[63];
  motionData[322] = xx[66];
  motionData[323] = xx[76];
  motionData[324] = xx[148];
  motionData[325] = xx[149];
  motionData[326] = xx[150];
  motionData[327] = xx[263];
  motionData[328] = xx[264];
  motionData[329] = xx[265];
  motionData[330] = xx[266];
  motionData[331] = xx[267];
  motionData[332] = xx[268];
}

size_t FD_cd27307_1_computeAssemblyPosError(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData, double *error)
{
  (void) mech;
  (void)rtdv;
  (void) modeVector;
  (void) motionData;
  (void) error;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t FD_cd27307_1_computeAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, boolean_T
  forVelocitySatisfaction, const double *state, const int *modeVector, const
  double *motionData, double *J)
{
  (void) mech;
  (void) rtdv;
  (void) state;
  (void) modeVector;
  (void) forVelocitySatisfaction;
  (void) motionData;
  (void) J;
  switch (constraintIdx)
  {
  }

  return 0;
}

size_t FD_cd27307_1_computeFullAssemblyJacobian(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, const int *modeVector,
  const double *motionData, double *J)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  (void) state;
  (void) modeVector;
  (void) motionData;
  (void) J;
  return 0;
}

boolean_T FD_cd27307_1_isInKinematicSingularity(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, size_t constraintIdx, const int *modeVector,
  const double *motionData)
{
  (void) mech;
  (void) rtdv;
  (void) modeVector;
  (void) motionData;
  switch (constraintIdx)
  {
  }

  return 0;
}

void FD_cd27307_1_convertStateVector(const void *asmMech, const
  RuntimeDerivedValuesBundle *rtdv, const void *simMech, const double *asmState,
  const int *asmModeVector, const int *simModeVector, double *simState)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  (void) asmMech;
  (void) rtdvd;
  (void) rtdvi;
  (void) simMech;
  (void) asmModeVector;
  (void) simModeVector;
  simState[0] = asmState[0];
  simState[1] = asmState[1];
  simState[2] = asmState[2];
  simState[3] = asmState[3];
  simState[4] = asmState[4];
  simState[5] = asmState[5];
  simState[6] = asmState[6];
  simState[7] = asmState[7];
  simState[8] = asmState[8];
  simState[9] = asmState[9];
  simState[10] = asmState[10];
  simState[11] = asmState[11];
  simState[12] = asmState[12];
  simState[13] = asmState[13];
  simState[14] = asmState[14];
  simState[15] = asmState[15];
  simState[16] = asmState[16];
  simState[17] = asmState[17];
  simState[18] = asmState[18];
  simState[19] = asmState[19];
  simState[20] = asmState[20];
  simState[21] = asmState[21];
  simState[22] = asmState[22];
  simState[23] = asmState[23];
}
