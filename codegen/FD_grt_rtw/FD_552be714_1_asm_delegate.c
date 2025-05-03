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

void FD_552be714_1_setTargets(const RuntimeDerivedValuesBundle *rtdv, CTarget
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

void FD_552be714_1_resetAsmStateVector(const void *mech, double *state)
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

void FD_552be714_1_initializeTrackedAngleState(const void *mech, const
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

void FD_552be714_1_computeDiscreteState(const void *mech, const
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

void FD_552be714_1_adjustPosition(const void *mech, const double *dofDeltas,
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

void FD_552be714_1_perturbAsmJointPrimitiveState(const void *mech, size_t
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

void FD_552be714_1_computePosDofBlendMatrix(const void *mech, size_t stageIdx,
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

void FD_552be714_1_computeVelDofBlendMatrix(const void *mech, size_t stageIdx,
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

void FD_552be714_1_projectPartiallyTargetedPos(const void *mech, size_t stageIdx,
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

void FD_552be714_1_propagateMotion(const void *mech, const
  RuntimeDerivedValuesBundle *rtdv, const double *state, double *motionData)
{
  const double *rtdvd = rtdv->mDoubles.mValues;
  const int *rtdvi = rtdv->mInts.mValues;
  double xx[309];
  (void) mech;
  (void) rtdvd;
  (void) rtdvi;
  xx[0] = 0.0;
  xx[1] = - 0.9829667740446056;
  xx[2] = 0.1821490196642661;
  xx[3] = 2.783635067711009e-3;
  xx[4] = - 0.02429623706456826;
  xx[5] = 0.5;
  xx[6] = xx[5] * state[0];
  xx[7] = 0.02392502936126694;
  xx[8] = sin(xx[6]);
  xx[9] = 0.35682893815498;
  xx[10] = 0.9338633207623329;
  xx[11] = cos(xx[6]);
  xx[12] = - (xx[7] * xx[8]);
  xx[13] = - (xx[9] * xx[8]);
  xx[14] = xx[10] * xx[8];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 11, xx + 15);
  xx[6] = 2.25350470908894e-3;
  xx[11] = 5.987450582022713e-3;
  xx[12] = 0.08741767895831867;
  xx[13] = - 0.10827874942964;
  pm_math_Quaternion_xform_ra(xx + 15, xx + 11, xx + 19);
  xx[8] = - (xx[6] + xx[19]);
  xx[14] = 3.440332745613401e-5;
  xx[22] = xx[14] - xx[20];
  xx[23] = 0.04533488045895317;
  xx[19] = xx[23] - xx[21];
  xx[24] = - 0.4095471207038393;
  xx[25] = - 0.5847136051092298;
  xx[26] = - 0.5476119269922152;
  xx[27] = 0.4364657298565779;
  xx[20] = xx[5] * state[2];
  xx[21] = 0.078167008186978;
  xx[28] = sin(xx[20]);
  xx[29] = 2.856546254231279e-4;
  xx[30] = 0.9969402375431197;
  xx[31] = cos(xx[20]);
  xx[32] = xx[21] * xx[28];
  xx[33] = xx[29] * xx[28];
  xx[34] = xx[30] * xx[28];
  pm_math_Quaternion_compose_ra(xx + 24, xx + 31, xx + 35);
  xx[20] = 5.8520263439862e-3;
  xx[31] = 0.2917565183728275;
  xx[32] = - 6.203349285939817e-5;
  xx[33] = - 0.06784399294262955;
  pm_math_Quaternion_xform_ra(xx + 35, xx + 31, xx + 39);
  xx[28] = - (xx[20] + xx[39]);
  xx[34] = 0.09337994929512419;
  xx[42] = - (xx[34] + xx[40]);
  xx[43] = 0.06005735603810682;
  xx[39] = xx[43] - xx[41];
  xx[44] = - 0.9783797318759546;
  xx[45] = 0.02243586449782985;
  xx[46] = - 0.2055333562023935;
  xx[47] = - 5.076586131480331e-3;
  xx[40] = xx[5] * state[4];
  xx[41] = 0.3296168505371159;
  xx[48] = sin(xx[40]);
  xx[49] = 0.04289886928108346;
  xx[50] = 0.9431396603135704;
  xx[51] = cos(xx[40]);
  xx[52] = - (xx[41] * xx[48]);
  xx[53] = - (xx[49] * xx[48]);
  xx[54] = xx[50] * xx[48];
  pm_math_Quaternion_compose_ra(xx + 44, xx + 51, xx + 55);
  xx[40] = 0.159250699658855;
  xx[51] = 0.1355744967355893;
  xx[52] = - 5.892129164777617e-3;
  xx[53] = 0.121338024222483;
  pm_math_Quaternion_xform_ra(xx + 55, xx + 51, xx + 59);
  xx[48] = - (xx[40] + xx[59]);
  xx[54] = 3.01967743008218e-5;
  xx[62] = - (xx[54] + xx[60]);
  xx[63] = 0.06307552675886077;
  xx[59] = - (xx[63] + xx[61]);
  xx[64] = - 0.6239725146564857;
  xx[65] = 0.3934418379080846;
  xx[66] = 0.3628487269116844;
  xx[67] = - 0.5693879367490505;
  xx[60] = xx[5] * state[6];
  xx[61] = 0.06905764132424259;
  xx[68] = sin(xx[60]);
  xx[69] = 0.6996517431390222;
  xx[70] = 0.7111388616137218;
  xx[71] = cos(xx[60]);
  xx[72] = - (xx[61] * xx[68]);
  xx[73] = xx[69] * xx[68];
  xx[74] = xx[70] * xx[68];
  pm_math_Quaternion_compose_ra(xx + 64, xx + 71, xx + 75);
  xx[60] = 0.09178277564968489;
  xx[71] = 0.01032865634959143;
  xx[72] = - 2.799981673523924e-3;
  xx[73] = - 0.103803911275921;
  pm_math_Quaternion_xform_ra(xx + 75, xx + 71, xx + 79);
  xx[68] = - (xx[60] + xx[79]);
  xx[74] = 1.575702046375558e-3;
  xx[82] = xx[74] - xx[80];
  xx[83] = 0.03730277188788071;
  xx[79] = - (xx[83] + xx[81]);
  xx[84] = - 0.7090454420135478;
  xx[85] = - 0.09771712505596977;
  xx[86] = - 0.08799345148381085;
  xx[87] = 0.6927936757264569;
  xx[80] = xx[5] * state[8];
  xx[81] = 0.5068089846600132;
  xx[88] = sin(xx[80]);
  xx[89] = 0.07069521833018202;
  xx[90] = 0.8591547236517614;
  xx[91] = cos(xx[80]);
  xx[92] = xx[81] * xx[88];
  xx[93] = - (xx[89] * xx[88]);
  xx[94] = xx[90] * xx[88];
  pm_math_Quaternion_compose_ra(xx + 84, xx + 91, xx + 95);
  xx[80] = 3.078347773072981e-3;
  xx[91] = 0.05178064963049919;
  xx[92] = 8.952325942534183e-3;
  xx[93] = - 0.1048343556145146;
  pm_math_Quaternion_xform_ra(xx + 95, xx + 91, xx + 99);
  xx[88] = - (xx[80] + xx[99]);
  xx[94] = 0.02618006350706806;
  xx[102] = - (xx[94] + xx[100]);
  xx[103] = 0.03461077007657011;
  xx[99] = xx[103] - xx[101];
  xx[104] = - 0.6092155953523121;
  xx[105] = 0.3401896210966002;
  xx[106] = 0.3770419488487212;
  xx[107] = - 0.609070397315481;
  xx[100] = xx[5] * state[10];
  xx[101] = 2.900602079164919e-3;
  xx[108] = sin(xx[100]);
  xx[109] = 0.02381723704035066;
  xx[110] = 0.9997121214266346;
  xx[111] = cos(xx[100]);
  xx[112] = xx[101] * xx[108];
  xx[113] = - (xx[109] * xx[108]);
  xx[114] = xx[110] * xx[108];
  pm_math_Quaternion_compose_ra(xx + 104, xx + 111, xx + 115);
  xx[100] = 0.0580752720787979;
  xx[111] = - 1.496353979004133e-4;
  xx[112] = 1.20198789844638e-3;
  xx[113] = - 0.03077076618082631;
  pm_math_Quaternion_xform_ra(xx + 115, xx + 111, xx + 119);
  xx[108] = - (xx[100] + xx[119]);
  xx[114] = 3.068452812081423e-3;
  xx[122] = - (xx[114] + xx[120]);
  xx[123] = 0.03929120542006859;
  xx[119] = xx[123] - xx[121];
  xx[120] = - 0.9996724082348869;
  xx[121] = - 0.01187363754226745;
  xx[124] = - 1.719307476161435e-3;
  xx[125] = 0.02260833755894023;
  xx[126] = 2.440072684948184e-5;
  xx[127] = - 2.270463239746596e-4;
  xx[128] = 0.02921196110477177;
  xx[129] = xx[5] * state[12];
  xx[130] = sin(xx[129]);
  xx[131] = cos(xx[129]);
  xx[132] = - (xx[7] * xx[130]);
  xx[133] = - (xx[9] * xx[130]);
  xx[134] = xx[10] * xx[130];
  pm_math_Quaternion_compose_ra(xx + 1, xx + 131, xx + 135);
  pm_math_Quaternion_xform_ra(xx + 135, xx + 11, xx + 1);
  xx[4] = - (xx[6] + xx[1]);
  xx[6] = xx[14] - xx[2];
  xx[1] = xx[23] - xx[3];
  xx[2] = xx[5] * state[14];
  xx[3] = sin(xx[2]);
  xx[11] = cos(xx[2]);
  xx[12] = xx[21] * xx[3];
  xx[13] = xx[29] * xx[3];
  xx[14] = xx[30] * xx[3];
  pm_math_Quaternion_compose_ra(xx + 24, xx + 11, xx + 129);
  pm_math_Quaternion_xform_ra(xx + 129, xx + 31, xx + 11);
  xx[2] = - (xx[20] + xx[11]);
  xx[3] = - (xx[34] + xx[12]);
  xx[11] = xx[43] - xx[13];
  xx[12] = xx[5] * state[16];
  xx[13] = sin(xx[12]);
  xx[23] = cos(xx[12]);
  xx[24] = - (xx[41] * xx[13]);
  xx[25] = - (xx[49] * xx[13]);
  xx[26] = xx[50] * xx[13];
  pm_math_Quaternion_compose_ra(xx + 44, xx + 23, xx + 31);
  pm_math_Quaternion_xform_ra(xx + 31, xx + 51, xx + 12);
  xx[20] = - (xx[40] + xx[12]);
  xx[23] = - (xx[54] + xx[13]);
  xx[12] = - (xx[63] + xx[14]);
  xx[13] = xx[5] * state[18];
  xx[14] = sin(xx[13]);
  xx[24] = cos(xx[13]);
  xx[25] = - (xx[61] * xx[14]);
  xx[26] = xx[69] * xx[14];
  xx[27] = xx[70] * xx[14];
  pm_math_Quaternion_compose_ra(xx + 64, xx + 24, xx + 43);
  pm_math_Quaternion_xform_ra(xx + 43, xx + 71, xx + 24);
  xx[13] = - (xx[60] + xx[24]);
  xx[14] = xx[74] - xx[25];
  xx[24] = - (xx[83] + xx[26]);
  xx[25] = xx[5] * state[20];
  xx[26] = sin(xx[25]);
  xx[51] = cos(xx[25]);
  xx[52] = xx[81] * xx[26];
  xx[53] = - (xx[89] * xx[26]);
  xx[54] = xx[90] * xx[26];
  pm_math_Quaternion_compose_ra(xx + 84, xx + 51, xx + 63);
  pm_math_Quaternion_xform_ra(xx + 63, xx + 91, xx + 25);
  xx[40] = - (xx[80] + xx[25]);
  xx[47] = - (xx[94] + xx[26]);
  xx[25] = xx[103] - xx[27];
  xx[26] = xx[5] * state[22];
  xx[5] = sin(xx[26]);
  xx[51] = cos(xx[26]);
  xx[52] = xx[101] * xx[5];
  xx[53] = - (xx[109] * xx[5]);
  xx[54] = xx[110] * xx[5];
  pm_math_Quaternion_compose_ra(xx + 104, xx + 51, xx + 71);
  pm_math_Quaternion_xform_ra(xx + 71, xx + 111, xx + 51);
  xx[5] = - (xx[100] + xx[51]);
  xx[26] = - (xx[114] + xx[52]);
  xx[27] = xx[123] - xx[53];
  xx[51] = 0.5015527894435715;
  xx[52] = - 0.8650478770170026;
  xx[53] = 9.976445339938241e-3;
  xx[54] = - 6.118856766501374e-3;
  xx[60] = 1.31966e-3;
  xx[67] = 0.1829794038889559;
  xx[80] = 1.276432289156824;
  pm_math_Quaternion_compose_ra(xx + 51, xx + 15, xx + 83);
  xx[91] = xx[8];
  xx[92] = xx[22];
  xx[93] = xx[19];
  pm_math_Quaternion_xform_ra(xx + 51, xx + 91, xx + 103);
  xx[87] = xx[103] + xx[60];
  xx[91] = xx[104] + xx[67];
  xx[92] = xx[105] + xx[80];
  pm_math_Quaternion_compose_ra(xx + 83, xx + 35, xx + 103);
  xx[111] = xx[28];
  xx[112] = xx[42];
  xx[113] = xx[39];
  pm_math_Quaternion_xform_ra(xx + 83, xx + 111, xx + 139);
  xx[93] = xx[139] + xx[87];
  xx[94] = xx[140] + xx[91];
  xx[100] = xx[141] + xx[92];
  pm_math_Quaternion_compose_ra(xx + 103, xx + 55, xx + 139);
  xx[143] = xx[48];
  xx[144] = xx[62];
  xx[145] = xx[59];
  pm_math_Quaternion_xform_ra(xx + 103, xx + 143, xx + 146);
  xx[107] = xx[146] + xx[93];
  xx[114] = xx[147] + xx[94];
  xx[123] = xx[148] + xx[100];
  pm_math_Quaternion_compose_ra(xx + 139, xx + 75, xx + 146);
  xx[150] = xx[68];
  xx[151] = xx[82];
  xx[152] = xx[79];
  pm_math_Quaternion_xform_ra(xx + 139, xx + 150, xx + 153);
  xx[133] = xx[153] + xx[107];
  xx[134] = xx[154] + xx[114];
  xx[153] = xx[155] + xx[123];
  pm_math_Quaternion_compose_ra(xx + 146, xx + 95, xx + 154);
  xx[158] = xx[88];
  xx[159] = xx[102];
  xx[160] = xx[99];
  pm_math_Quaternion_xform_ra(xx + 146, xx + 158, xx + 161);
  xx[164] = xx[161] + xx[133];
  xx[165] = xx[162] + xx[134];
  xx[161] = xx[163] + xx[153];
  pm_math_Quaternion_compose_ra(xx + 154, xx + 115, xx + 166);
  xx[170] = xx[108];
  xx[171] = xx[122];
  xx[172] = xx[119];
  pm_math_Quaternion_xform_ra(xx + 154, xx + 170, xx + 173);
  xx[162] = xx[173] + xx[164];
  xx[163] = xx[174] + xx[165];
  xx[173] = xx[175] + xx[161];
  xx[174] = xx[120];
  xx[175] = xx[121];
  xx[176] = xx[124];
  xx[177] = xx[125];
  pm_math_Quaternion_compose_ra(xx + 166, xx + 174, xx + 178);
  pm_math_Quaternion_xform_ra(xx + 166, xx + 126, xx + 182);
  xx[185] = 0.501317315387473;
  xx[186] = 0.865184361522414;
  xx[187] = 3.505811293856058e-4;
  xx[188] = 0.01169816065120726;
  xx[189] = 1.319659999999993e-3;
  xx[190] = 0.1829772367209632;
  xx[191] = 1.276428505906517;
  pm_math_Quaternion_compose_ra(xx + 185, xx + 135, xx + 192);
  xx[196] = xx[4];
  xx[197] = xx[6];
  xx[198] = xx[1];
  pm_math_Quaternion_xform_ra(xx + 185, xx + 196, xx + 199);
  xx[196] = xx[199] + xx[189];
  xx[197] = xx[200] - xx[190];
  xx[198] = xx[201] + xx[191];
  pm_math_Quaternion_compose_ra(xx + 192, xx + 129, xx + 199);
  xx[203] = xx[2];
  xx[204] = xx[3];
  xx[205] = xx[11];
  pm_math_Quaternion_xform_ra(xx + 192, xx + 203, xx + 206);
  xx[209] = xx[206] + xx[196];
  xx[210] = xx[207] + xx[197];
  xx[206] = xx[208] + xx[198];
  pm_math_Quaternion_compose_ra(xx + 199, xx + 31, xx + 211);
  xx[215] = xx[20];
  xx[216] = xx[23];
  xx[217] = xx[12];
  pm_math_Quaternion_xform_ra(xx + 199, xx + 215, xx + 218);
  xx[207] = xx[218] + xx[209];
  xx[208] = xx[219] + xx[210];
  xx[218] = xx[220] + xx[206];
  pm_math_Quaternion_compose_ra(xx + 211, xx + 43, xx + 219);
  xx[223] = xx[13];
  xx[224] = xx[14];
  xx[225] = xx[24];
  pm_math_Quaternion_xform_ra(xx + 211, xx + 223, xx + 226);
  xx[229] = xx[226] + xx[207];
  xx[230] = xx[227] + xx[208];
  xx[226] = xx[228] + xx[218];
  pm_math_Quaternion_compose_ra(xx + 219, xx + 63, xx + 231);
  xx[235] = xx[40];
  xx[236] = xx[47];
  xx[237] = xx[25];
  pm_math_Quaternion_xform_ra(xx + 219, xx + 235, xx + 238);
  xx[227] = xx[238] + xx[229];
  xx[228] = xx[239] + xx[230];
  xx[238] = xx[240] + xx[226];
  pm_math_Quaternion_compose_ra(xx + 231, xx + 71, xx + 239);
  xx[243] = xx[5];
  xx[244] = xx[26];
  xx[245] = xx[27];
  pm_math_Quaternion_xform_ra(xx + 231, xx + 243, xx + 246);
  xx[249] = xx[246] + xx[227];
  xx[250] = xx[247] + xx[228];
  xx[246] = xx[248] + xx[238];
  pm_math_Quaternion_compose_ra(xx + 239, xx + 174, xx + 251);
  pm_math_Quaternion_xform_ra(xx + 239, xx + 126, xx + 255);
  xx[247] = - (xx[7] * state[1]);
  xx[248] = - (xx[9] * state[1]);
  xx[258] = xx[10] * state[1];
  xx[259] = 0.04299917278162339;
  xx[260] = xx[259] * state[1];
  xx[261] = 3.00088822412269e-3;
  xx[262] = xx[261] * state[1];
  xx[263] = 4.502509766700054e-5;
  xx[264] = xx[263] * state[1];
  xx[265] = xx[247];
  xx[266] = xx[248];
  xx[267] = xx[258];
  pm_math_Quaternion_inverseXform_ra(xx + 35, xx + 265, xx + 268);
  xx[271] = xx[268] + xx[21] * state[3];
  xx[272] = xx[269] + xx[29] * state[3];
  xx[268] = xx[270] + xx[30] * state[3];
  pm_math_Vector3_cross_ra(xx + 265, xx + 111, xx + 273);
  xx[111] = xx[273] + xx[260];
  xx[112] = xx[274] - xx[262];
  xx[113] = xx[275] - xx[264];
  pm_math_Quaternion_inverseXform_ra(xx + 35, xx + 111, xx + 265);
  xx[111] = 4.246373471564165e-5;
  xx[112] = xx[265] - xx[111] * state[3];
  xx[113] = 0.296166974683144;
  xx[269] = xx[266] - xx[113] * state[3];
  xx[265] = 8.819057151475338e-5;
  xx[266] = xx[267] + xx[265] * state[3];
  xx[273] = xx[271];
  xx[274] = xx[272];
  xx[275] = xx[268];
  pm_math_Quaternion_inverseXform_ra(xx + 55, xx + 273, xx + 276);
  xx[267] = xx[276] - xx[41] * state[5];
  xx[270] = xx[277] - xx[49] * state[5];
  xx[276] = xx[278] + xx[50] * state[5];
  pm_math_Vector3_cross_ra(xx + 273, xx + 143, xx + 277);
  xx[143] = xx[277] + xx[112];
  xx[144] = xx[278] + xx[269];
  xx[145] = xx[279] + xx[266];
  pm_math_Quaternion_inverseXform_ra(xx + 55, xx + 143, xx + 273);
  xx[143] = 3.51836659046807e-4;
  xx[144] = xx[273] - xx[143] * state[5];
  xx[145] = 0.167860742192998;
  xx[277] = xx[274] - xx[145] * state[5];
  xx[273] = 7.758137671560606e-3;
  xx[274] = xx[275] - xx[273] * state[5];
  xx[278] = xx[267];
  xx[279] = xx[270];
  xx[280] = xx[276];
  pm_math_Quaternion_inverseXform_ra(xx + 75, xx + 278, xx + 281);
  xx[275] = xx[281] - xx[61] * state[7];
  xx[284] = xx[282] + xx[69] * state[7];
  xx[281] = xx[283] + xx[70] * state[7];
  pm_math_Vector3_cross_ra(xx + 278, xx + 150, xx + 285);
  xx[150] = xx[285] + xx[144];
  xx[151] = xx[286] + xx[277];
  xx[152] = xx[287] + xx[274];
  pm_math_Quaternion_inverseXform_ra(xx + 75, xx + 150, xx + 278);
  xx[150] = 0.07063541168899744;
  xx[151] = xx[278] + xx[150] * state[7];
  xx[152] = 1.766556455017367e-4;
  xx[282] = xx[279] - xx[152] * state[7];
  xx[278] = 7.033102289150907e-3;
  xx[279] = xx[280] + xx[278] * state[7];
  xx[285] = xx[275];
  xx[286] = xx[284];
  xx[287] = xx[281];
  pm_math_Quaternion_inverseXform_ra(xx + 95, xx + 285, xx + 288);
  xx[280] = xx[288] + xx[81] * state[9];
  xx[283] = xx[289] - xx[89] * state[9];
  xx[288] = xx[290] + xx[90] * state[9];
  pm_math_Vector3_cross_ra(xx + 285, xx + 158, xx + 289);
  xx[158] = xx[289] + xx[151];
  xx[159] = xx[290] + xx[282];
  xx[160] = xx[291] + xx[279];
  pm_math_Quaternion_inverseXform_ra(xx + 95, xx + 158, xx + 285);
  xx[158] = 2.801454625263955e-4;
  xx[159] = xx[285] + xx[158] * state[9];
  xx[160] = 0.09761858305027912;
  xx[289] = xx[286] - xx[160] * state[9];
  xx[285] = 8.197763552188046e-3;
  xx[286] = xx[287] - xx[285] * state[9];
  xx[290] = xx[280];
  xx[291] = xx[283];
  xx[292] = xx[288];
  pm_math_Quaternion_inverseXform_ra(xx + 115, xx + 290, xx + 293);
  xx[287] = xx[293] + xx[101] * state[11];
  xx[296] = xx[294] - xx[109] * state[11];
  xx[293] = xx[295] + xx[110] * state[11];
  pm_math_Vector3_cross_ra(xx + 290, xx + 170, xx + 297);
  xx[170] = xx[297] + xx[159];
  xx[171] = xx[298] + xx[289];
  xx[172] = xx[299] + xx[286];
  pm_math_Quaternion_inverseXform_ra(xx + 115, xx + 170, xx + 290);
  xx[170] = 4.687672398430269e-4;
  xx[171] = xx[290] + xx[170] * state[11];
  xx[172] = 6.033857271393841e-5;
  xx[294] = xx[291] + xx[172] * state[11];
  xx[290] = 7.741314405669247e-8;
  xx[291] = xx[292] + xx[290] * state[11];
  xx[297] = xx[287];
  xx[298] = xx[296];
  xx[299] = xx[293];
  pm_math_Quaternion_inverseXform_ra(xx + 174, xx + 297, xx + 300);
  pm_math_Vector3_cross_ra(xx + 297, xx + 126, xx + 303);
  xx[297] = xx[303] + xx[171];
  xx[298] = xx[304] + xx[294];
  xx[299] = xx[305] + xx[291];
  pm_math_Quaternion_inverseXform_ra(xx + 174, xx + 297, xx + 303);
  xx[292] = - (xx[7] * state[13]);
  xx[7] = - (xx[9] * state[13]);
  xx[9] = xx[10] * state[13];
  xx[10] = xx[259] * state[13];
  xx[259] = xx[261] * state[13];
  xx[261] = xx[263] * state[13];
  xx[297] = xx[292];
  xx[298] = xx[7];
  xx[299] = xx[9];
  pm_math_Quaternion_inverseXform_ra(xx + 129, xx + 297, xx + 306);
  xx[263] = xx[306] + xx[21] * state[15];
  xx[21] = xx[307] + xx[29] * state[15];
  xx[29] = xx[308] + xx[30] * state[15];
  pm_math_Vector3_cross_ra(xx + 297, xx + 203, xx + 306);
  xx[203] = xx[306] + xx[10];
  xx[204] = xx[307] - xx[259];
  xx[205] = xx[308] - xx[261];
  pm_math_Quaternion_inverseXform_ra(xx + 129, xx + 203, xx + 297);
  xx[30] = xx[297] - xx[111] * state[15];
  xx[111] = xx[298] - xx[113] * state[15];
  xx[113] = xx[299] + xx[265] * state[15];
  xx[203] = xx[263];
  xx[204] = xx[21];
  xx[205] = xx[29];
  pm_math_Quaternion_inverseXform_ra(xx + 31, xx + 203, xx + 297);
  xx[265] = xx[297] - xx[41] * state[17];
  xx[41] = xx[298] - xx[49] * state[17];
  xx[49] = xx[299] + xx[50] * state[17];
  pm_math_Vector3_cross_ra(xx + 203, xx + 215, xx + 297);
  xx[203] = xx[297] + xx[30];
  xx[204] = xx[298] + xx[111];
  xx[205] = xx[299] + xx[113];
  pm_math_Quaternion_inverseXform_ra(xx + 31, xx + 203, xx + 215);
  xx[50] = xx[215] - xx[143] * state[17];
  xx[143] = xx[216] - xx[145] * state[17];
  xx[145] = xx[217] - xx[273] * state[17];
  xx[203] = xx[265];
  xx[204] = xx[41];
  xx[205] = xx[49];
  pm_math_Quaternion_inverseXform_ra(xx + 43, xx + 203, xx + 215);
  xx[273] = xx[215] - xx[61] * state[19];
  xx[61] = xx[216] + xx[69] * state[19];
  xx[69] = xx[217] + xx[70] * state[19];
  pm_math_Vector3_cross_ra(xx + 203, xx + 223, xx + 215);
  xx[203] = xx[215] + xx[50];
  xx[204] = xx[216] + xx[143];
  xx[205] = xx[217] + xx[145];
  pm_math_Quaternion_inverseXform_ra(xx + 43, xx + 203, xx + 215);
  xx[70] = xx[215] + xx[150] * state[19];
  xx[150] = xx[216] - xx[152] * state[19];
  xx[152] = xx[217] + xx[278] * state[19];
  xx[203] = xx[273];
  xx[204] = xx[61];
  xx[205] = xx[69];
  pm_math_Quaternion_inverseXform_ra(xx + 63, xx + 203, xx + 215);
  xx[223] = xx[215] + xx[81] * state[21];
  xx[81] = xx[216] - xx[89] * state[21];
  xx[89] = xx[217] + xx[90] * state[21];
  pm_math_Vector3_cross_ra(xx + 203, xx + 235, xx + 215);
  xx[203] = xx[215] + xx[70];
  xx[204] = xx[216] + xx[150];
  xx[205] = xx[217] + xx[152];
  pm_math_Quaternion_inverseXform_ra(xx + 63, xx + 203, xx + 215);
  xx[90] = xx[215] + xx[158] * state[21];
  xx[158] = xx[216] - xx[160] * state[21];
  xx[160] = xx[217] - xx[285] * state[21];
  xx[203] = xx[223];
  xx[204] = xx[81];
  xx[205] = xx[89];
  pm_math_Quaternion_inverseXform_ra(xx + 71, xx + 203, xx + 215);
  xx[224] = xx[215] + xx[101] * state[23];
  xx[101] = xx[216] - xx[109] * state[23];
  xx[109] = xx[217] + xx[110] * state[23];
  pm_math_Vector3_cross_ra(xx + 203, xx + 243, xx + 215);
  xx[203] = xx[215] + xx[90];
  xx[204] = xx[216] + xx[158];
  xx[205] = xx[217] + xx[160];
  pm_math_Quaternion_inverseXform_ra(xx + 71, xx + 203, xx + 215);
  xx[110] = xx[215] + xx[170] * state[23];
  xx[170] = xx[216] + xx[172] * state[23];
  xx[172] = xx[217] + xx[290] * state[23];
  xx[203] = xx[224];
  xx[204] = xx[101];
  xx[205] = xx[109];
  pm_math_Quaternion_inverseXform_ra(xx + 174, xx + 203, xx + 215);
  pm_math_Vector3_cross_ra(xx + 203, xx + 126, xx + 235);
  xx[203] = xx[235] + xx[110];
  xx[204] = xx[236] + xx[170];
  xx[205] = xx[237] + xx[172];
  pm_math_Quaternion_inverseXform_ra(xx + 174, xx + 203, xx + 235);
  motionData[0] = - 0.999851918871201;
  motionData[1] = 1.093796538566551e-4;
  motionData[2] = 0.01442285491200129;
  motionData[3] = - 9.386672565723609e-3;
  motionData[4] = xx[0];
  motionData[5] = xx[0];
  motionData[6] = xx[0];
  motionData[7] = - 0.5013718130490967;
  motionData[8] = 0.8648595258380031;
  motionData[9] = - 0.02532938158317313;
  motionData[10] = - 1.651688754072407e-3;
  motionData[11] = 0.04156560465100582;
  motionData[12] = 0.1822975848369339;
  motionData[13] = 1.275853629388897;
  motionData[14] = xx[15];
  motionData[15] = xx[16];
  motionData[16] = xx[17];
  motionData[17] = xx[18];
  motionData[18] = xx[8];
  motionData[19] = xx[22];
  motionData[20] = xx[19];
  motionData[21] = xx[35];
  motionData[22] = xx[36];
  motionData[23] = xx[37];
  motionData[24] = xx[38];
  motionData[25] = xx[28];
  motionData[26] = xx[42];
  motionData[27] = xx[39];
  motionData[28] = xx[55];
  motionData[29] = xx[56];
  motionData[30] = xx[57];
  motionData[31] = xx[58];
  motionData[32] = xx[48];
  motionData[33] = xx[62];
  motionData[34] = xx[59];
  motionData[35] = xx[75];
  motionData[36] = xx[76];
  motionData[37] = xx[77];
  motionData[38] = xx[78];
  motionData[39] = xx[68];
  motionData[40] = xx[82];
  motionData[41] = xx[79];
  motionData[42] = xx[95];
  motionData[43] = xx[96];
  motionData[44] = xx[97];
  motionData[45] = xx[98];
  motionData[46] = xx[88];
  motionData[47] = xx[102];
  motionData[48] = xx[99];
  motionData[49] = xx[115];
  motionData[50] = xx[116];
  motionData[51] = xx[117];
  motionData[52] = xx[118];
  motionData[53] = xx[108];
  motionData[54] = xx[122];
  motionData[55] = xx[119];
  motionData[56] = xx[120];
  motionData[57] = xx[121];
  motionData[58] = xx[124];
  motionData[59] = xx[125];
  motionData[60] = xx[126];
  motionData[61] = xx[127];
  motionData[62] = xx[128];
  motionData[63] = - 0.5012531966104286;
  motionData[64] = - 0.8652830896240489;
  motionData[65] = 5.415257317145841e-4;
  motionData[66] = 5.487663288624312e-3;
  motionData[67] = 0.03469512793638958;
  motionData[68] = - 0.1835945565621305;
  motionData[69] = 1.275868891481225;
  motionData[70] = xx[135];
  motionData[71] = xx[136];
  motionData[72] = xx[137];
  motionData[73] = xx[138];
  motionData[74] = xx[4];
  motionData[75] = xx[6];
  motionData[76] = xx[1];
  motionData[77] = xx[129];
  motionData[78] = xx[130];
  motionData[79] = xx[131];
  motionData[80] = xx[132];
  motionData[81] = xx[2];
  motionData[82] = xx[3];
  motionData[83] = xx[11];
  motionData[84] = xx[31];
  motionData[85] = xx[32];
  motionData[86] = xx[33];
  motionData[87] = xx[34];
  motionData[88] = xx[20];
  motionData[89] = xx[23];
  motionData[90] = xx[12];
  motionData[91] = xx[43];
  motionData[92] = xx[44];
  motionData[93] = xx[45];
  motionData[94] = xx[46];
  motionData[95] = xx[13];
  motionData[96] = xx[14];
  motionData[97] = xx[24];
  motionData[98] = xx[63];
  motionData[99] = xx[64];
  motionData[100] = xx[65];
  motionData[101] = xx[66];
  motionData[102] = xx[40];
  motionData[103] = xx[47];
  motionData[104] = xx[25];
  motionData[105] = xx[71];
  motionData[106] = xx[72];
  motionData[107] = xx[73];
  motionData[108] = xx[74];
  motionData[109] = xx[5];
  motionData[110] = xx[26];
  motionData[111] = xx[27];
  motionData[112] = xx[120];
  motionData[113] = xx[121];
  motionData[114] = xx[124];
  motionData[115] = xx[125];
  motionData[116] = xx[126];
  motionData[117] = xx[127];
  motionData[118] = xx[128];
  motionData[119] = xx[51];
  motionData[120] = xx[52];
  motionData[121] = xx[53];
  motionData[122] = xx[54];
  motionData[123] = xx[60];
  motionData[124] = xx[67];
  motionData[125] = xx[80];
  motionData[126] = xx[83];
  motionData[127] = xx[84];
  motionData[128] = xx[85];
  motionData[129] = xx[86];
  motionData[130] = xx[87];
  motionData[131] = xx[91];
  motionData[132] = xx[92];
  motionData[133] = xx[103];
  motionData[134] = xx[104];
  motionData[135] = xx[105];
  motionData[136] = xx[106];
  motionData[137] = xx[93];
  motionData[138] = xx[94];
  motionData[139] = xx[100];
  motionData[140] = xx[139];
  motionData[141] = xx[140];
  motionData[142] = xx[141];
  motionData[143] = xx[142];
  motionData[144] = xx[107];
  motionData[145] = xx[114];
  motionData[146] = xx[123];
  motionData[147] = xx[146];
  motionData[148] = xx[147];
  motionData[149] = xx[148];
  motionData[150] = xx[149];
  motionData[151] = xx[133];
  motionData[152] = xx[134];
  motionData[153] = xx[153];
  motionData[154] = xx[154];
  motionData[155] = xx[155];
  motionData[156] = xx[156];
  motionData[157] = xx[157];
  motionData[158] = xx[164];
  motionData[159] = xx[165];
  motionData[160] = xx[161];
  motionData[161] = xx[166];
  motionData[162] = xx[167];
  motionData[163] = xx[168];
  motionData[164] = xx[169];
  motionData[165] = xx[162];
  motionData[166] = xx[163];
  motionData[167] = xx[173];
  motionData[168] = xx[178];
  motionData[169] = xx[179];
  motionData[170] = xx[180];
  motionData[171] = xx[181];
  motionData[172] = xx[182] + xx[162];
  motionData[173] = xx[183] + xx[163];
  motionData[174] = xx[184] + xx[173];
  motionData[175] = xx[185];
  motionData[176] = xx[186];
  motionData[177] = xx[187];
  motionData[178] = xx[188];
  motionData[179] = xx[189];
  motionData[180] = - xx[190];
  motionData[181] = xx[191];
  motionData[182] = xx[192];
  motionData[183] = xx[193];
  motionData[184] = xx[194];
  motionData[185] = xx[195];
  motionData[186] = xx[196];
  motionData[187] = xx[197];
  motionData[188] = xx[198];
  motionData[189] = xx[199];
  motionData[190] = xx[200];
  motionData[191] = xx[201];
  motionData[192] = xx[202];
  motionData[193] = xx[209];
  motionData[194] = xx[210];
  motionData[195] = xx[206];
  motionData[196] = xx[211];
  motionData[197] = xx[212];
  motionData[198] = xx[213];
  motionData[199] = xx[214];
  motionData[200] = xx[207];
  motionData[201] = xx[208];
  motionData[202] = xx[218];
  motionData[203] = xx[219];
  motionData[204] = xx[220];
  motionData[205] = xx[221];
  motionData[206] = xx[222];
  motionData[207] = xx[229];
  motionData[208] = xx[230];
  motionData[209] = xx[226];
  motionData[210] = xx[231];
  motionData[211] = xx[232];
  motionData[212] = xx[233];
  motionData[213] = xx[234];
  motionData[214] = xx[227];
  motionData[215] = xx[228];
  motionData[216] = xx[238];
  motionData[217] = xx[239];
  motionData[218] = xx[240];
  motionData[219] = xx[241];
  motionData[220] = xx[242];
  motionData[221] = xx[249];
  motionData[222] = xx[250];
  motionData[223] = xx[246];
  motionData[224] = xx[251];
  motionData[225] = xx[252];
  motionData[226] = xx[253];
  motionData[227] = xx[254];
  motionData[228] = xx[255] + xx[249];
  motionData[229] = xx[256] + xx[250];
  motionData[230] = xx[257] + xx[246];
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
  motionData[243] = xx[247];
  motionData[244] = xx[248];
  motionData[245] = xx[258];
  motionData[246] = xx[260];
  motionData[247] = - xx[262];
  motionData[248] = - xx[264];
  motionData[249] = xx[271];
  motionData[250] = xx[272];
  motionData[251] = xx[268];
  motionData[252] = xx[112];
  motionData[253] = xx[269];
  motionData[254] = xx[266];
  motionData[255] = xx[267];
  motionData[256] = xx[270];
  motionData[257] = xx[276];
  motionData[258] = xx[144];
  motionData[259] = xx[277];
  motionData[260] = xx[274];
  motionData[261] = xx[275];
  motionData[262] = xx[284];
  motionData[263] = xx[281];
  motionData[264] = xx[151];
  motionData[265] = xx[282];
  motionData[266] = xx[279];
  motionData[267] = xx[280];
  motionData[268] = xx[283];
  motionData[269] = xx[288];
  motionData[270] = xx[159];
  motionData[271] = xx[289];
  motionData[272] = xx[286];
  motionData[273] = xx[287];
  motionData[274] = xx[296];
  motionData[275] = xx[293];
  motionData[276] = xx[171];
  motionData[277] = xx[294];
  motionData[278] = xx[291];
  motionData[279] = xx[300];
  motionData[280] = xx[301];
  motionData[281] = xx[302];
  motionData[282] = xx[303];
  motionData[283] = xx[304];
  motionData[284] = xx[305];
  motionData[285] = xx[0];
  motionData[286] = xx[0];
  motionData[287] = xx[0];
  motionData[288] = xx[0];
  motionData[289] = xx[0];
  motionData[290] = xx[0];
  motionData[291] = xx[292];
  motionData[292] = xx[7];
  motionData[293] = xx[9];
  motionData[294] = xx[10];
  motionData[295] = - xx[259];
  motionData[296] = - xx[261];
  motionData[297] = xx[263];
  motionData[298] = xx[21];
  motionData[299] = xx[29];
  motionData[300] = xx[30];
  motionData[301] = xx[111];
  motionData[302] = xx[113];
  motionData[303] = xx[265];
  motionData[304] = xx[41];
  motionData[305] = xx[49];
  motionData[306] = xx[50];
  motionData[307] = xx[143];
  motionData[308] = xx[145];
  motionData[309] = xx[273];
  motionData[310] = xx[61];
  motionData[311] = xx[69];
  motionData[312] = xx[70];
  motionData[313] = xx[150];
  motionData[314] = xx[152];
  motionData[315] = xx[223];
  motionData[316] = xx[81];
  motionData[317] = xx[89];
  motionData[318] = xx[90];
  motionData[319] = xx[158];
  motionData[320] = xx[160];
  motionData[321] = xx[224];
  motionData[322] = xx[101];
  motionData[323] = xx[109];
  motionData[324] = xx[110];
  motionData[325] = xx[170];
  motionData[326] = xx[172];
  motionData[327] = xx[215];
  motionData[328] = xx[216];
  motionData[329] = xx[217];
  motionData[330] = xx[235];
  motionData[331] = xx[236];
  motionData[332] = xx[237];
}

size_t FD_552be714_1_computeAssemblyPosError(const void *mech, const
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

size_t FD_552be714_1_computeAssemblyJacobian(const void *mech, const
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

size_t FD_552be714_1_computeFullAssemblyJacobian(const void *mech, const
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

boolean_T FD_552be714_1_isInKinematicSingularity(const void *mech, const
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

void FD_552be714_1_convertStateVector(const void *asmMech, const
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
