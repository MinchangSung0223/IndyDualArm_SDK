/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'FD_nom/DualArm/Solver Configuration'.
 */

#include "pm_std.h"
#include "ne_std.h"
#include "ssc_dae.h"
#include "pm_default_allocator.h"
#include "sm_ssci_NeDaePrivateData.h"
#include "sm_CTarget.h"
#define NULL_INDEX                     ((size_t) -1)

PmfMessageId sm_ssci_recordRunTimeError(
  const char *errorId, const char *errorMsg, NeuDiagnosticManager* mgr);

#define pm_allocator_alloc(_allocator, _m, _n) ((_allocator)->mCallocFcn((_allocator), (_m), (_n)))
#define PM_ALLOCATE_ARRAY(_name, _type, _size, _allocator)\
 _name = (_type *) pm_allocator_alloc(_allocator, sizeof(_type), _size)
#define pm_size_to_int(_size)          ((int32_T) (_size))

extern const NeAssertData FD_nom_552be714_1_assertData[];
extern const NeZCData FD_nom_552be714_1_ZCData[];
void FD_nom_552be714_1_computeRuntimeParameters(
  const double *runtimeRootVariables,
  double *runtimeParameters);
void FD_nom_552be714_1_validateRuntimeParameters(
  const double *runtimeParameters,
  int32_T *assertSatisfactionFlags);
void FD_nom_552be714_1_computeAsmRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
void FD_nom_552be714_1_computeSimRuntimeDerivedValues(
  const double *runtimeParameters,
  RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
void FD_nom_552be714_1_initializeGeometries(
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle);
PmfMessageId FD_nom_552be714_1_compDerivs(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_numJacPerturbLoBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_numJacPerturbHiBounds(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_checkDynamics(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  const int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_compOutputsDyn(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_compOutputsKin(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_compOutputs (
  const RuntimeDerivedValuesBundle *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  const double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_computeAsmModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_computeSimModeVector(
  const double *, const double *, const double *,
  int *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_computeZeroCrossings(
  const RuntimeDerivedValuesBundle *,
  const double *,
  const double *, const double *, const double *,
  const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
PmfMessageId FD_nom_552be714_1_recordLog(
  const RuntimeDerivedValuesBundle *,
  const int *,
  const double *,
  const int *,
  const double *, const double *, const double *,
  double *,
  double *,
  NeuDiagnosticManager *neDiagMgr);
void FD_nom_552be714_1_setTargets(
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  CTarget *targets);
void FD_nom_552be714_1_resetAsmStateVector(const void *mech, double *stateVector);
void FD_nom_552be714_1_resetSimStateVector(const void *mech, double *stateVector);
void FD_nom_552be714_1_initializeTrackedAngleState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *modeVector,
  const double *motionData,
  double *stateVector);
void FD_nom_552be714_1_computeDiscreteState(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *modeVector,
  double *stateVector);
void FD_nom_552be714_1_adjustPosition(
  const void *mech,
  const double *dofDeltas,
  double *stateVector);
void FD_nom_552be714_1_perturbAsmJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void FD_nom_552be714_1_perturbSimJointPrimitiveState(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void FD_nom_552be714_1_perturbFlexibleBodyState(
  const void *mech,
  size_t stageIdx,
  double magnitude,
  boolean_T doPerturbVelocity,
  double *stateVector);
void FD_nom_552be714_1_computePosDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void FD_nom_552be714_1_computeVelDofBlendMatrix(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *stateVector,
  int partialType,
  double *matrix);
void FD_nom_552be714_1_projectPartiallyTargetedPos(
  const void *mech,
  size_t stageIdx,
  size_t primitiveIdx,
  const double *origStateVector,
  int partialType,
  double *stateVector);
void FD_nom_552be714_1_propagateMotion(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  double *motionData);
size_t FD_nom_552be714_1_computeAssemblyPosError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const int *modeVector,
  const double *motionData,
  double *error);
size_t FD_nom_552be714_1_computeAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  boolean_T forVelocitySatisfaction,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
size_t FD_nom_552be714_1_computeFullAssemblyJacobian(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  const double *motionData,
  double *J);
boolean_T FD_nom_552be714_1_isInKinematicSingularity(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  size_t constraintIdx,
  const int *modeVector,
  const double *motionData);
void FD_nom_552be714_1_convertStateVector(
  const void *asmMech,
  const RuntimeDerivedValuesBundle *asmRuntimeDerivedValuesBundle,
  const void *simMech,
  const double *asmStateVector,
  const int *asmModeVector,
  const int *simModeVector,
  double *simStateVector);
void FD_nom_552be714_1_constructStateVector(
  const void *mech,
  const double *solverStateVector,
  const double *u,
  const double *uDot,
  const double *discreteStateVector,
  double *fullStateVector);
void FD_nom_552be714_1_extractSolverStateVector(
  const void *mech,
  const double *fullStateVector,
  double *solverStateVector);
void FD_nom_552be714_1_extractDiscreteStateVector(
  const void *mech,
  const double *fullStateVector,
  double *discreteStateVector);
boolean_T FD_nom_552be714_1_isPositionViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
boolean_T FD_nom_552be714_1_isVelocityViolation(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const double *stateVector,
  const int *modeVector);
PmfMessageId FD_nom_552be714_1_projectStateSim(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const int *modeVector,
  double *stateVector,
  void *neDiagMgr);
void FD_nom_552be714_1_computeConstraintError(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const double *stateVector,
  const int *modeVector,
  double *error);
void FD_nom_552be714_1_resetModeVector(const void *mech, int *modeVector);
boolean_T FD_nom_552be714_1_hasJointUpwardModeChange(
  const void *mech,
  const int *prevModeVector,
  const int *modeVector);
PmfMessageId FD_nom_552be714_1_performJointUpwardModeChange(
  const void *mech,
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle,
  const int *constraintEqnEnableFlags,
  const int *prevModeVector,
  const int *modeVector,
  const double *inputVector,
  double *stateVector,
  void *neDiagMgr);
void FD_nom_552be714_1_onModeChangedCutJoints(
  const void *mech,
  const int *prevModeVector,
  const int *modeVector,
  double *stateVector);
void FD_nom_552be714_1_setVariableModeJointsToLocked(
  const void *mech,
  int *modeVector);
PmfMessageId FD_nom_552be714_1_assemble(const double *u, double *udot, double *x,
  NeuDiagnosticManager *neDiagMgr)
{
  (void) x;
  (void) u;
  (void) udot;
  (void) neDiagMgr;
  return NULL;
}

static
  void dae_cg_setParameters_function(const NeDae *dae,
  const NeParameterBundle *paramBundle)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeRootVariables = paramBundle->mRealParameters.mX;
  if (smData->mRuntimeParameterScalars.mN == 0)
    return;
  FD_nom_552be714_1_computeRuntimeParameters(
    runtimeRootVariables,
    smData->mRuntimeParameterScalars.mX);
  FD_nom_552be714_1_computeAsmRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mAsmRuntimeDerivedValuesBundle);
  FD_nom_552be714_1_computeSimRuntimeDerivedValues(
    smData->mRuntimeParameterScalars.mX,
    &dae->mPrivateData->mSimRuntimeDerivedValuesBundle);
  FD_nom_552be714_1_initializeGeometries(&smData->mSimRuntimeDerivedValuesBundle);
  sm_core_computeRedundantConstraintEquations(
    &dae->mPrivateData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle);

#if 0

  {
    size_t i;
    const size_t n = smData->mSimulationDelegate.mRunTimeEnabledEquations.mSize;
    pmf_printf("\nRuntime Enabled Equations (%lu)\n", n);
    for (i = 0; i < n; ++i)
      pmf_printf("  %2lu:  %d\n", i,
                 smData->mSimulationDelegate.mRunTimeEnabledEquations.mValues[i]);
  }

#endif

}

static
  PmfMessageId dae_cg_pAssert_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  const double *runtimeParams = smData->mRuntimeParameterScalars.mX;
  int32_T *assertSatisfactionFlags = daeMethodOutput->mPASSERT.mX;
  (void) systemInput;
  (void) neDiagMgr;
  FD_nom_552be714_1_validateRuntimeParameters(
    runtimeParams, assertSatisfactionFlags);
  return NULL;
}

static
  PmfMessageId dae_cg_deriv_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  if (smData->mCachedDerivativesAvailable)
    memcpy(daeMethodOutput->mXP0.mX, smData->mCachedDerivatives.mX,
           24 * sizeof(real_T));
  else
    errorId = FD_nom_552be714_1_compDerivs(
      &smData->mSimRuntimeDerivedValuesBundle,
      smData->mSimulationDelegate
      .mRunTimeEnabledEquations.mValues,
      systemInput->mX.mX,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 24,
      systemInput->mV.mX + 24,
      systemInput->mD.mX,
      daeMethodOutput->mXP0.mX,
      &errorResult,
      neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbLoBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = FD_nom_552be714_1_numJacPerturbLoBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mV.mX + 24,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_LO.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_numJacPerturbHiBounds_method(
  const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = FD_nom_552be714_1_numJacPerturbHiBounds(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mV.mX + 24,
    systemInput->mD.mX,
    daeMethodOutput->mNUMJAC_DX_HI.mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_compOutputs_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  PmfMessageId errorId = NULL;
  NeDaePrivateData *smData = dae->mPrivateData;
  errorId = FD_nom_552be714_1_compOutputs(
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mX.mX,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mV.mX + 24,
    systemInput->mD.mX,
    daeMethodOutput->mY.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_mode_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  errorId = FD_nom_552be714_1_computeSimModeVector(
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mV.mX + 24,
    daeMethodOutput->mMODE.mX,
    &errorResult,
    neDiagMgr);
  memcpy(smData->mCachedModeVector.mX, daeMethodOutput->mMODE.mX,
         0 * sizeof(int32_T));
  return errorId;
}

static
  PmfMessageId dae_cg_zeroCrossing_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeDaeMethodOutput *daeMethodOutput,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  double errorResult = 0.0;
  return
    FD_nom_552be714_1_computeZeroCrossings(
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mX.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mV.mX + 24,
    systemInput->mD.mX,
    daeMethodOutput->mZC.mX,
    &errorResult,
    neDiagMgr);
}

static
  void dae_cg_setupLoggerFcn(const NeDae *dae,
  NeLoggerBuilder *neLoggerBuilder)
{
  (void) dae;
  (void) neLoggerBuilder;
}

static
  PmfMessageId dae_cg_recordLog_method(const NeDae *dae,
  const NeSystemInput *systemInput,
  PmRealVector *output,
  NeuDiagnosticManager *neDiagMgr)
{
  const NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  double *fullStateVector = smData->mSimulationFullStateVector.mX;
  FD_nom_552be714_1_constructStateVector(
    NULL,
    systemInput->mX.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mD.mX,
    fullStateVector);
  errorId = FD_nom_552be714_1_recordLog(
    &smData->mSimRuntimeDerivedValuesBundle,
    smData->mSimulationDelegate
    .mRunTimeEnabledEquations.mValues,
    fullStateVector,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mV.mX + 24,
    output->mX,
    &errorResult,
    neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_project_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  return
    sm_core_projectState(
    false,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
}

static
  PmfMessageId dae_cg_check_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  if (smData->mNumConstraintEqns > 0)
    errorId = sm_core_projectState(
      false,
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mM.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 24,
      systemInput->mD.mX,
      systemInput->mX.mX, neDiagMgr);
  if (errorId == NULL) {
    double result = 0.0;
    errorId = FD_nom_552be714_1_checkDynamics(
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mX.mX,
      systemInput->mU.mX,
      systemInput->mU.mX + 24,
      systemInput->mV.mX + 24,
      systemInput->mD.mX,
      systemInput->mM.mX,
      &result, neDiagMgr);
  }

  return errorId;
}

static
  PmfMessageId dae_cg_CIC_MODE_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  PmfMessageId errorId = NULL;
  double errorResult = 0.0;
  const size_t mvSize = smData->mModeVectorSize;
  boolean_T modeChanged = false;
  if (mvSize > 0) {
    errorId = FD_nom_552be714_1_computeSimModeVector(
      systemInput->mU.mX,
      systemInput->mU.mX + 24,
      systemInput->mV.mX + 24,
      systemInput->mM.mX,
      &errorResult,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;

    {
      size_t i;
      for (i = 0; i < mvSize; ++i)
        if (systemInput->mM.mX[i] != smData->mCachedModeVector.mX[i]) {
          modeChanged = true;
          break;
        }
    }
  }

  if (modeChanged) {
    errorId = sm_core_onModeChanged(
      &smData->mSimulationDelegate,
      &smData->mSimRuntimeDerivedValuesBundle,
      systemInput->mU.mX,
      systemInput->mU.mX + 24,
      smData->mCachedModeVector.mX,
      systemInput->mM.mX,
      systemInput->mX.mX,
      systemInput->mD.mX,
      neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  errorId =
    sm_core_projectState(
    true,
    &smData->mSimulationDelegate,
    &smData->mSimRuntimeDerivedValuesBundle,
    systemInput->mM.mX,
    systemInput->mU.mX,
    systemInput->mU.mX + 24,
    systemInput->mD.mX,
    systemInput->mX.mX, neDiagMgr);
  return errorId;
}

static
  PmfMessageId dae_cg_assemble_solve(const NeDae *dae,
  const NeSystemInput *systemInput,
  NeuDiagnosticManager *neDiagMgr)
{
  NeDaePrivateData *smData = dae->mPrivateData;
  const SmMechanismDelegate *delegate = &smData->mAssemblyDelegate;
  const RuntimeDerivedValuesBundle *runtimeDerivedValuesBundle =
    &smData->mAsmRuntimeDerivedValuesBundle;
  PmfMessageId errorId = NULL;
  size_t i;
  double errorResult = 0.0;
  const size_t numTargets = 24;
  unsigned int asmStatus = 0;
  double *assemblyFullStateVector = smData->mAssemblyFullStateVector.mX;
  double *simulationFullStateVector = smData->mSimulationFullStateVector.mX;
  const double *u = systemInput->mU.mX;
  const double *uDot = u + smData->mInputVectorSize;
  const double *uDDot = systemInput->mV.mX +
    smData->mInputVectorSize;
  if (smData->mAssemblyModeVector.mN > 0) {
    errorId = FD_nom_552be714_1_computeAsmModeVector(
      u, uDot, uDDot, smData->mAssemblyModeVector.mX, &errorResult, neDiagMgr);
    if (errorId != NULL)
      return errorId;
  }

  if (smData->mModeVectorSize > 0) {
    errorId = FD_nom_552be714_1_computeSimModeVector(
      u, uDot, uDDot, systemInput->mM.mX, &errorResult, neDiagMgr);
    if (errorId != NULL)
      return errorId;
    memcpy(smData->mCachedModeVector.mX, systemInput->mM.mX,
           0 * sizeof(int32_T));
  }

  (*delegate->mSetTargets)(runtimeDerivedValuesBundle, smData->mTargets);

  {
    CTarget *target = smData->mTargets + smData->mNumInternalTargets;
    for (i = 0; i < smData->mNumMotionInputPrimitives; ++i) {
      const size_t inputOffset = smData->mMotionInputOffsets.mX[i];
      sm_compiler_CTarget_setValue( &u[inputOffset], 1, target++);
      sm_compiler_CTarget_setValue(&uDot[inputOffset], 1, target++);
    }

    for (i = 0; i < smData->mNumMaybeLockedPrimitives; ++i) {
      const boolean_T hasMode = smData->mMaybeLockedPrimHasModes.mX[i];
      const size_t modeOffset = smData->mMaybeLockedPrimModeOffsets.mX[i];
      if (hasMode && systemInput->mM.mX[modeOffset] != 1)
        target->mStrength = 0;
      else
        target->mStrength = 3;
      ++target;
    }
  }

  sm_core_computeStateVector(
    delegate, runtimeDerivedValuesBundle, smData->mAssemblyModeVector.mX,
    numTargets, smData->mTargets, assemblyFullStateVector);
  asmStatus = sm_core_checkAssembly(
    delegate, runtimeDerivedValuesBundle, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX,
    NULL, NULL, NULL);
  if (asmStatus != 1) {
    return sm_ssci_recordRunTimeError(
      "physmod:sm:ssci:core:dae:dae:assemblyFailure",
      asmStatus == 2 ?
      "Model not assembled. The following violation occurred: Position Violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
      :
      (asmStatus == 3 ?
       "Model not assembled. The following violation occurred: Velocity Violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."
       :
       "Model not assembled. The following violation occurred: Singularity Violation. The failure occurred during the attempt to assemble all joints in the system and satisfy any motion inputs. If an Update Diagram operation completes successfully, the failure is likely caused by motion inputs. Consider adjusting the motion inputs to specify a different starting configuration. Also consider adjusting or adding joint targets to better guide the assembly."),
      neDiagMgr);
  }

#if 0

  FD_nom_552be714_1_checkTargets(
    &smData->mSimRuntimeDerivedValuesBundle,
    assemblyFullStateVector);

#endif

  (*delegate->mConvertStateVector)(
    NULL, runtimeDerivedValuesBundle, NULL, assemblyFullStateVector,
    smData->mAssemblyModeVector.mX, systemInput->mM.mX,
    simulationFullStateVector);
  for (i = 0; i < smData->mStateVectorSize; ++i)
    systemInput->mX.mX[i] = simulationFullStateVector[smData->
      mStateVectorMap.mX[i]];
  memcpy(systemInput->mD.mX,
         simulationFullStateVector +
         smData->mFullStateVectorSize - smData->mDiscreteStateSize,
         smData->mDiscreteStateSize * sizeof(double));
  return errorId;
}

typedef struct {
  size_t first;
  size_t second;
} SizePair;

static void checkMemAllocStatus(int_T status)
{
  (void) status;
}

static
  PmCharVector cStringToCharVector(const char *src)
{
  const size_t n = strlen(src);
  PmCharVector charVect;
  const int_T status =
    pm_create_char_vector_fields(&charVect, n + 1, pm_default_allocator());
  checkMemAllocStatus(status);
  strcpy(charVect.mX, src);
  return charVect;
}

static
  void initBasicAttributes(NeDaePrivateData *smData)
{
  size_t i;
  smData->mStateVectorSize = 24;
  smData->mFullStateVectorSize = 24;
  smData->mDiscreteStateSize = 0;
  smData->mModeVectorSize = 0;
  smData->mNumZeroCrossings = 0;
  smData->mInputVectorSize = 24;
  smData->mOutputVectorSize = 48;
  smData->mNumConstraintEqns = 0;
  smData->mFundamentalSampleTime = +1.000000000000000021e-03;
  for (i = 0; i < 4; ++i)
    smData->mChecksum[i] = 0;
}

static
  void initStateVector(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  static const int32_T stateVectorMap[24] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23
  };

  static real_T targetVals[24] = {
    +3.307339653589793205e+00, +0.000000000000000000e+00,
    +8.192739999999999467e-01, +0.000000000000000000e+00,
    +1.263689999999999980e+00, +0.000000000000000000e+00,
    +6.041480000000000183e-01, +0.000000000000000000e+00,
    +1.051500000000000101e+00, +0.000000000000000000e+00,
    -1.392170000000000074e-01, +0.000000000000000000e+00,
    -1.657470000000000054e-01, +0.000000000000000000e+00,
    -8.192739999999999467e-01, +0.000000000000000000e+00,
    -1.263689999999999980e+00, +0.000000000000000000e+00,
    -6.041480000000000183e-01, +0.000000000000000000e+00,
    -1.051500000000000101e+00, +0.000000000000000000e+00,
    +1.392170000000000074e-01, +0.000000000000000000e+00
  };

  static const CTarget targets[24] = {
    { 0, 145, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[0] }, { +0.000000000000000000e+00 } },

    { 0, 145, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[1] }, { +0.000000000000000000e+00 } },

    { 0, 146, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[2] }, { +0.000000000000000000e+00 } },

    { 0, 146, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[3] }, { +0.000000000000000000e+00 } },

    { 0, 147, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[4] }, { +0.000000000000000000e+00 } },

    { 0, 147, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[5] }, { +0.000000000000000000e+00 } },

    { 0, 148, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[6] }, { +0.000000000000000000e+00 } },

    { 0, 148, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[7] }, { +0.000000000000000000e+00 } },

    { 0, 149, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[8] }, { +0.000000000000000000e+00 } },

    { 0, 149, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[9] }, { +0.000000000000000000e+00 } },

    { 0, 150, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[10] }, { +0.000000000000000000e+00 } },

    { 0, 150, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[11] }, { +0.000000000000000000e+00 } },

    { 0, 297, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[12] }, { +0.000000000000000000e+00 } },

    { 0, 297, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[13] }, { +0.000000000000000000e+00 } },

    { 0, 298, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[14] }, { +0.000000000000000000e+00 } },

    { 0, 298, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[15] }, { +0.000000000000000000e+00 } },

    { 0, 299, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[16] }, { +0.000000000000000000e+00 } },

    { 0, 299, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[17] }, { +0.000000000000000000e+00 } },

    { 0, 300, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[18] }, { +0.000000000000000000e+00 } },

    { 0, 300, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[19] }, { +0.000000000000000000e+00 } },

    { 0, 301, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[20] }, { +0.000000000000000000e+00 } },

    { 0, 301, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[21] }, { +0.000000000000000000e+00 } },

    { 0, 302, false, 0, 2, "rad", 0, false, true, +1.000000000000000000e+00,
      false, { 1, 1, &targetVals[22] }, { +0.000000000000000000e+00 } },

    { 0, 302, false, 0, 0, "1", 0, true, true, +1.000000000000000000e+00, false,
      { 1, 1, &targetVals[23] }, { +0.000000000000000000e+00 } }
  };

  int_T status;
  size_t i;
  status = pm_create_real_vector_fields(
    &smData->mAssemblyFullStateVector, 24, alloc);
  checkMemAllocStatus(status);
  status = pm_create_real_vector_fields(
    &smData->mSimulationFullStateVector, 24, alloc);
  checkMemAllocStatus(status);
  status = pm_create_int_vector_fields(
    &smData->mStateVectorMap, smData->mStateVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mStateVectorMap.mX, stateVectorMap,
         smData->mStateVectorSize * sizeof(int32_T));
  smData->mNumInternalTargets = 24;
  smData->mNumMotionInputPrimitives = 0;
  smData->mNumMaybeLockedPrimitives = 0;
  smData->mNumTargets = 24;
  PM_ALLOCATE_ARRAY(smData->mTargets, CTarget, smData->mNumTargets, alloc);
  for (i = 0; i < smData->mNumTargets; ++i)
    sm_compiler_CTarget_copy(targets + i, smData->mTargets + i);
}

static void initAsserts(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mNumParamAsserts = 24;
  smData->mParamAssertObjects = NULL;
  smData->mParamAssertPaths = NULL;
  smData->mParamAssertDescriptors = NULL;
  smData->mParamAssertMessages = NULL;
  smData->mParamAssertMessageIds = NULL;
  status = pm_create_bool_vector_fields(
    &smData->mParamAssertIsWarnings, smData->mNumParamAsserts, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumParamAsserts > 0) {
    const NeAssertData *ad = FD_nom_552be714_1_assertData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mParamAssertObjects,
                      PmCharVector, 24, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertPaths,
                      PmCharVector, 24, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertDescriptors,
                      PmCharVector, 24, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessages,
                      PmCharVector, 24, alloc);
    PM_ALLOCATE_ARRAY(smData->mParamAssertMessageIds,
                      PmCharVector, 24, alloc);
    for (i = 0; i < smData->mNumParamAsserts; ++i, ++ad) {
      smData->mParamAssertObjects [i] = cStringToCharVector(ad->mObject );
      smData->mParamAssertPaths [i] = cStringToCharVector(ad->mPath );
      smData->mParamAssertDescriptors[i] = cStringToCharVector(ad->mDescriptor);
      smData->mParamAssertMessages [i] = cStringToCharVector(ad->mMessage );
      smData->mParamAssertMessageIds [i] = cStringToCharVector(ad->mMessageID );
      smData->mParamAssertIsWarnings.mX[i] = ad->mIsWarn;
    }
  }
}

static
  void initModeVector(NeDaePrivateData *smData)
{
  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mAssemblyModeVector, 0,
      pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mAssemblyModeVector.mN; ++i)
      smData->mAssemblyModeVector.mX[i] = 0;
  }

  {
    size_t i;
    const int_T status = pm_create_int_vector_fields(
      &smData->mCachedModeVector, 0, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mModeVectorSize; ++i)
      smData->mCachedModeVector.mX[i] = 0;
  }
}

static void initZeroCrossings(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  smData->mZeroCrossingObjects = NULL;
  smData->mZeroCrossingPaths = NULL;
  smData->mZeroCrossingDescriptors = NULL;
  status = pm_create_int_vector_fields(
    &smData->mZeroCrossingTypes, 0, alloc);
  checkMemAllocStatus(status);
  if (smData->mNumZeroCrossings > 0) {
    const NeZCData *zcd = FD_nom_552be714_1_ZCData;
    size_t i;
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingObjects,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingPaths,
                      PmCharVector, 0, alloc);
    PM_ALLOCATE_ARRAY(smData->mZeroCrossingDescriptors,
                      PmCharVector, 0, alloc);
    for (i = 0; i < smData->mNumZeroCrossings; ++i, ++zcd) {
      smData->mZeroCrossingObjects [i] = cStringToCharVector(zcd->mObject);
      smData->mZeroCrossingPaths [i] = cStringToCharVector(zcd->mPath );
      smData->mZeroCrossingDescriptors[i] = cStringToCharVector(zcd->mDescriptor);
      smData->mZeroCrossingTypes.mX[i] = zcd->mType;
    }
  }
}

static
  void initVariables(NeDaePrivateData *smData)
{
  static const char *varFullPaths[24] = {
    "DualArm.Left.l_joint_0.Rz.q",
    "DualArm.Left.l_joint_0.Rz.w",
    "DualArm.Left.l_joint_1.Rz.q",
    "DualArm.Left.l_joint_1.Rz.w",
    "DualArm.Left.l_joint_2.Rz.q",
    "DualArm.Left.l_joint_2.Rz.w",
    "DualArm.Left.l_joint_3.Rz.q",
    "DualArm.Left.l_joint_3.Rz.w",
    "DualArm.Left.l_joint_4.Rz.q",
    "DualArm.Left.l_joint_4.Rz.w",
    "DualArm.Left.l_joint_5.Rz.q",
    "DualArm.Left.l_joint_5.Rz.w",
    "DualArm.Right.r_joint_0.Rz.q",
    "DualArm.Right.r_joint_0.Rz.w",
    "DualArm.Right.r_joint_1.Rz.q",
    "DualArm.Right.r_joint_1.Rz.w",
    "DualArm.Right.r_joint_2.Rz.q",
    "DualArm.Right.r_joint_2.Rz.w",
    "DualArm.Right.r_joint_3.Rz.q",
    "DualArm.Right.r_joint_3.Rz.w",
    "DualArm.Right.r_joint_4.Rz.q",
    "DualArm.Right.r_joint_4.Rz.w",
    "DualArm.Right.r_joint_5.Rz.q",
    "DualArm.Right.r_joint_5.Rz.w"
  };

  static const char *varObjects[24] = {
    "FD_nom/DualArm/Left/l_joint_0",
    "FD_nom/DualArm/Left/l_joint_0",
    "FD_nom/DualArm/Left/l_joint_1",
    "FD_nom/DualArm/Left/l_joint_1",
    "FD_nom/DualArm/Left/l_joint_2",
    "FD_nom/DualArm/Left/l_joint_2",
    "FD_nom/DualArm/Left/l_joint_3",
    "FD_nom/DualArm/Left/l_joint_3",
    "FD_nom/DualArm/Left/l_joint_4",
    "FD_nom/DualArm/Left/l_joint_4",
    "FD_nom/DualArm/Left/l_joint_5",
    "FD_nom/DualArm/Left/l_joint_5",
    "FD_nom/DualArm/Right/r_joint_0",
    "FD_nom/DualArm/Right/r_joint_0",
    "FD_nom/DualArm/Right/r_joint_1",
    "FD_nom/DualArm/Right/r_joint_1",
    "FD_nom/DualArm/Right/r_joint_2",
    "FD_nom/DualArm/Right/r_joint_2",
    "FD_nom/DualArm/Right/r_joint_3",
    "FD_nom/DualArm/Right/r_joint_3",
    "FD_nom/DualArm/Right/r_joint_4",
    "FD_nom/DualArm/Right/r_joint_4",
    "FD_nom/DualArm/Right/r_joint_5",
    "FD_nom/DualArm/Right/r_joint_5"
  };

  static const char *varEncodedDims[24] = {
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1",
    "1x1"
  };

  static const size_t varNumels[24] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1
  };

  smData->mNumVarScalars = 24;
  smData->mVarFullPaths = NULL;
  smData->mVarObjects = NULL;
  smData->mVarEncodedDims = NULL;
  if (smData->mNumVarScalars > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    int_T status = 0;
    PM_ALLOCATE_ARRAY(smData->mVarFullPaths, PmCharVector, 24, alloc);
    PM_ALLOCATE_ARRAY(smData->mVarObjects, PmCharVector, 24, alloc);
    PM_ALLOCATE_ARRAY(smData->mVarEncodedDims, PmCharVector, 24, alloc);
    for (s = 0; s < smData->mNumVarScalars; ++s) {
      smData->mVarFullPaths[s] = cStringToCharVector(varFullPaths[s]);
      smData->mVarObjects[s] = cStringToCharVector(varObjects[s]);
      smData->mVarEncodedDims[s] = cStringToCharVector(varEncodedDims[s]);
    }

    status = pm_create_size_vector_fields(
      &smData->mVarNumels, smData->mNumVarScalars, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mVarNumels.mX, varNumels,
           24 * sizeof(size_t));
  }
}

static
  void initRuntimeParameters(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  int_T status = 0;
  size_t i = 0;
  static const int32_T rtpRootVarNumels[12] = {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1
  };

  static const char *rtpFullPaths [12] = {
    "RTP_136D3059_PositionTargetValue",
    "RTP_1FF00632_PositionTargetValue",
    "RTP_2A185AF1_PositionTargetValue",
    "RTP_2D759EE8_PositionTargetValue",
    "RTP_5A72AE7E_PositionTargetValue",
    "RTP_5D1F6A67_PositionTargetValue",
    "RTP_646A00CF_PositionTargetValue",
    "RTP_68F736A4_PositionTargetValue",
    "RTP_86F95788_PositionTargetValue",
    "RTP_B3110B4B_PositionTargetValue",
    "RTP_C4163BDD_PositionTargetValue",
    "RTP_F1FE671E_PositionTargetValue"
  };

  smData->mNumRtpRootVars = 12;
  status = pm_create_int_vector_fields(
    &smData->mRtpRootVarNumels, smData->mNumRtpRootVars, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mRtpRootVarNumels.mX, rtpRootVarNumels,
         smData->mNumRtpRootVars * sizeof(int32_T));
  smData->mRtpFullPaths = NULL;
  if (smData->mNumRtpRootVars > 0) {
    size_t v;
    PM_ALLOCATE_ARRAY(smData->mRtpFullPaths, PmCharVector, 12, alloc);
    for (v = 0; v < smData->mNumRtpRootVars; ++v) {
      smData->mRtpFullPaths[v] = cStringToCharVector(rtpFullPaths[v]);
    }
  }

  smData->mNumRuntimeRootVarScalars = 12;
  status = pm_create_real_vector_fields(
    &smData->mRuntimeParameterScalars, 12,
    alloc);
  checkMemAllocStatus(status);
  for (i = 0; i < smData->mRuntimeParameterScalars.mN; ++i)
    smData->mRuntimeParameterScalars.mX[i] = 0.0;
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mAsmRuntimeDerivedValuesBundle,
    24,
    0);
  sm_core_RuntimeDerivedValuesBundle_create(
    &smData->mSimRuntimeDerivedValuesBundle,
    0,
    0);
}

static
  void initIoInfoHelper(
  size_t n,
  const char *portPathsSource[],
  const char *unitsSource[],
  const SscArraySize dimensions[],
  boolean_T doInputs,
  NeDaePrivateData *smData)
{
  PmCharVector *portPaths = NULL;
  PmCharVector *units = NULL;
  SscIoInfo *infos = NULL;
  if (n > 0) {
    size_t s;
    PmAllocator *alloc = pm_default_allocator();
    PM_ALLOCATE_ARRAY(portPaths, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(units, PmCharVector, n, alloc);
    PM_ALLOCATE_ARRAY(infos, SscIoInfo, n, alloc);
    for (s = 0; s < n; ++s) {
      portPaths[s] = cStringToCharVector(portPathsSource[s]);
      units[s] = cStringToCharVector(unitsSource[s]);

      {
        SscIoInfo *info = infos + s;
        info->name = info->identifier = portPaths[s].mX;
        info->size = dimensions[s];
        info->unit = units[s].mX;
      }
    }
  }

  if (doInputs) {
    smData->mNumInputs = n;
    smData->mInputPortPaths = portPaths;
    smData->mInputUnits = units;
    smData->mInputInfos = infos;
  } else {
    smData->mNumOutputs = n;
    smData->mOutputPortPaths = portPaths;
    smData->mOutputUnits = units;
    smData->mOutputInfos = infos;
  }
}

static
  void initIoInfo(NeDaePrivateData *smData)
{
  static const char *inputPortPaths[16] = {
    "DualArm.Left.l_joint_0.ti",
    "DualArm.Left.l_joint_1.ti",
    "DualArm.Left.l_joint_2.ti",
    "DualArm.Left.l_joint_3.ti",
    "DualArm.Left.l_joint_4.ti",
    "DualArm.Left.l_joint_5.ti",
    "DualArm.Right.r_joint_0.ti",
    "DualArm.Right.r_joint_1.ti",
    "DualArm.Right.r_joint_2.ti",
    "DualArm.Right.r_joint_3.ti",
    "DualArm.Right.r_joint_4.ti",
    "DualArm.Right.r_joint_5.ti",
    "DualArm.Left.External_Force_and_Torque.f",
    "DualArm.Left.External_Force_and_Torque.t",
    "DualArm.Right.External_Force_and_Torque.f",
    "DualArm.Right.External_Force_and_Torque.t"
  };

  static const char *inputUnits[16] = {
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m^2/s^2",
    "kg*m/s^2",
    "kg*m^2/s^2",
    "kg*m/s^2",
    "kg*m^2/s^2"
  };

  static const SscArraySize inputDimensions[16] = {
    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 3, "3x1" }, { 3, "3x1" }, { 3, "3x1" }, { 3, "3x1" }
  };

  static const char *outputPortPaths[28] = {
    "DualArm.Left.l_joint_0.q",
    "DualArm.Left.l_joint_0.w",
    "DualArm.Left.l_joint_1.q",
    "DualArm.Left.l_joint_1.w",
    "DualArm.Left.l_joint_2.q",
    "DualArm.Left.l_joint_2.w",
    "DualArm.Left.l_joint_3.q",
    "DualArm.Left.l_joint_3.w",
    "DualArm.Left.l_joint_4.q",
    "DualArm.Left.l_joint_4.w",
    "DualArm.Left.l_joint_5.q",
    "DualArm.Left.l_joint_5.w",
    "DualArm.Right.r_joint_0.q",
    "DualArm.Right.r_joint_0.w",
    "DualArm.Right.r_joint_1.q",
    "DualArm.Right.r_joint_1.w",
    "DualArm.Right.r_joint_2.q",
    "DualArm.Right.r_joint_2.w",
    "DualArm.Right.r_joint_3.q",
    "DualArm.Right.r_joint_3.w",
    "DualArm.Right.r_joint_4.q",
    "DualArm.Right.r_joint_4.w",
    "DualArm.Right.r_joint_5.q",
    "DualArm.Right.r_joint_5.w",
    "DualArm.Transform_Sensor.R",
    "DualArm.Transform_Sensor.p",
    "DualArm.Transform_Sensor1.R",
    "DualArm.Transform_Sensor1.p"
  };

  static const char *outputUnits[28] = {
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "rad",
    "rad/s",
    "1",
    "m",
    "1",
    "m"
  };

  static const SscArraySize outputDimensions[28] = {
    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" }, { 1, "1x1" },

    { 9, "3x3" }, { 3, "3x1" }, { 9, "3x3" }, { 3, "3x1" }
  };

  initIoInfoHelper(16, inputPortPaths, inputUnits, inputDimensions,
                   true, smData);
  initIoInfoHelper(28, outputPortPaths, outputUnits, outputDimensions,
                   false, smData);
}

static
  void initInputDerivs(NeDaePrivateData *smData)
{
  static const int32_T numInputDerivs[24] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0
  };

  PmAllocator *alloc = pm_default_allocator();
  const int_T status = pm_create_int_vector_fields(
    &smData->mNumInputDerivs, smData->mInputVectorSize, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mNumInputDerivs.mX, numInputDerivs,
         24 * sizeof(int32_T));
  smData->mInputOrder = 1;
}

static
  void initDirectFeedthrough(NeDaePrivateData *smData)
{
  static const boolean_T directFeedthroughVector[24] = {
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false
  };

  static const boolean_T directFeedthroughMatrix[1152] = {
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false, false, false, false, false, false, false, false, false,
    false, false
  };

  PmAllocator *alloc = pm_default_allocator();

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughVector, 24, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughVector.mX, directFeedthroughVector,
           24 * sizeof(boolean_T));
  }

  {
    const int_T status = pm_create_bool_vector_fields(
      &smData->mDirectFeedthroughMatrix, 1152, alloc);
    checkMemAllocStatus(status);
    memcpy(smData->mDirectFeedthroughMatrix.mX, directFeedthroughMatrix,
           1152 * sizeof(boolean_T));
  }
}

static
  void initOutputDerivProc(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  static const int32_T outputFunctionMap[48] = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
  };

  smData->mOutputFunctionMap = pm_create_int_vector(48, alloc);
  memcpy(smData->mOutputFunctionMap->mX, outputFunctionMap,
         48 * sizeof(int32_T));
  smData->mNumOutputClasses = 1;
  smData->mHasKinematicOutputs = true;
  smData->mHasDynamicOutputs = false;
  smData->mIsOutputClass0Dynamic = false;
  smData->mDoComputeDynamicOutputs = false;
  smData->mCachedDerivativesAvailable = false;

  {
    size_t i = 0;
    const int_T status = pm_create_real_vector_fields(
      &smData->mCachedDerivatives, 0, pm_default_allocator());
    checkMemAllocStatus(status);
    for (i = 0; i < smData->mCachedDerivatives.mN; ++i)
      smData->mCachedDerivatives.mX[i] = 0.0;
  }
}

#if 0

static void initializeSizePairVector(const SmSizePair *data,
  SmSizePairVector *vector)
{
  const size_t n = sm_core_SmSizePairVector_size(vector);
  size_t i;
  for (i = 0; i < n; ++i, ++data)
    sm_core_SmSizePairVector_setValue(vector, i, data++);
}

#endif

static
  void initAssemblyDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  static const SmSizePair jointToStageIdx[17] = {
    { 144, 1 }, { 145, 2 }, { 146, 3 }, { 147, 4 }, { 148, 5 }, { 149, 6 },

    { 150, 7 }, { 151, 8 }, { 296, 9 }, { 297, 10 }, { 298, 11 }, { 299, 12 },

    { 300, 13 }, { 301, 14 }, { 302, 15 }, { 303, 16 }, { 345, 0 }
  };

  static const size_t primitiveIndices[17 + 1] = {
    0, 0, 0, 1, 2, 3, 4, 5, 6, 6,
    6, 7, 8, 9, 10, 11, 12, 12
  };

  static const SmSizePair stateOffsets[12] = {
    { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 8, 9 }, { 10, 11 },

    { 12, 13 }, { 14, 15 }, { 16, 17 }, { 18, 19 }, { 20, 21 }, { 22, 23 }
  };

  static const SmSizePair dofOffsets[12] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 }, { 5, 6 },

    { 6, 7 }, { 7, 8 }, { 8, 9 }, { 9, 10 }, { 10, 11 }, { 11, 12 }
  };

  static const SmSizePair *flexBodyToStageIdx = NULL;
  static const SmSizePair *flexStateOffsets = NULL;
  static const size_t *flexibleStages = NULL;
  static const size_t remodIndices[12] = {
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18,
    20, 22
  };

  static const size_t *equationsPerConstraint = NULL;
  static const int32_T *hasAllVelocityDisabledEquations = NULL;
  static const int32_T *runtimeEnabledEquations = NULL;
  static const size_t dofToVelSlot[12] = {
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19,
    21, 23
  };

  static const size_t *constraintDofs = NULL;
  static const size_t constraintDofOffsets[0 + 1] = {
    0
  };

  const size_t Jm = 0;
  const size_t Jn = 12;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000062e-09;
  delegate->mTreeJointDof = 12;
  delegate->mDof = 12;
  delegate->mStateSize = 24;
  delegate->mContinuousStateSize = 24;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 17;
  delegate->mNumConstraints = 0;
  delegate->mNumAllConstraintEquations = 0;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 17, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 17 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 12, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 12 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 12, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 12 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mFlexBodyToStageIdx, 0, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mFlexBodyToStageIdx),
         flexBodyToStageIdx, 0 *sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mFlexStateOffsets, 0, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mFlexStateOffsets),
         flexStateOffsets, 0 *sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 12, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 12 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mHasAllVelocityDisabledEquations, delegate->mNumConstraints, 0);
  memcpy(
         sm_core_SmIntVector_nonConstValues
         (&delegate->mHasAllVelocityDisabledEquations),
         hasAllVelocityDisabledEquations, delegate->mNumConstraints * sizeof
         (int32_T));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 0);
  memcpy(
         sm_core_SmIntVector_nonConstValues(&delegate->mRunTimeEnabledEquations),
         runtimeEnabledEquations, delegate->mNumAllConstraintEquations *
         sizeof(int32_T));
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    0, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 12, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 12, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 12, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 12, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 12, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 12, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 12, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 0, &zeroSizePair);
  sm_core_SmSizePairVector_create(&scratchpad->mCvQuaternionVelOffsets,
    0,
    &zeroSizePair);
  sm_core_SmRealVector_create(&scratchpad->mCvQuaternionAzimuthValues,
    0, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 24, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 169, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 12, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 12, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 333, 0.0);
  delegate->mSetTargets = FD_nom_552be714_1_setTargets;
  delegate->mResetStateVector = FD_nom_552be714_1_resetAsmStateVector;
  delegate->mInitializeTrackedAngleState =
    FD_nom_552be714_1_initializeTrackedAngleState;
  delegate->mComputeDiscreteState = FD_nom_552be714_1_computeDiscreteState;
  delegate->mAdjustPosition = FD_nom_552be714_1_adjustPosition;
  delegate->mPerturbJointPrimitiveState =
    FD_nom_552be714_1_perturbAsmJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState = NULL;
  delegate->mComputePosDofBlendMatrix =
    FD_nom_552be714_1_computePosDofBlendMatrix;
  delegate->mComputeVelDofBlendMatrix =
    FD_nom_552be714_1_computeVelDofBlendMatrix;
  delegate->mProjectPartiallyTargetedPos =
    FD_nom_552be714_1_projectPartiallyTargetedPos;
  delegate->mPropagateMotion = FD_nom_552be714_1_propagateMotion;
  delegate->mComputeAssemblyPosError = FD_nom_552be714_1_computeAssemblyPosError;
  delegate->mComputeAssemblyJacobian = FD_nom_552be714_1_computeAssemblyJacobian;
  delegate->mComputeFullAssemblyJacobian =
    FD_nom_552be714_1_computeFullAssemblyJacobian;
  delegate->mIsInKinematicSingularity =
    FD_nom_552be714_1_isInKinematicSingularity;
  delegate->mConvertStateVector = FD_nom_552be714_1_convertStateVector;
  delegate->mConstructStateVector = NULL;
  delegate->mExtractSolverStateVector = NULL;
  delegate->mExtractDiscreteStateVector = NULL;
  delegate->mIsPositionViolation = NULL;
  delegate->mIsVelocityViolation = NULL;
  delegate->mProjectStateSim = NULL;
  delegate->mComputeConstraintError = NULL;
  delegate->mResetModeVector = NULL;
  delegate->mHasJointUpwardModeChange = NULL;
  delegate->mPerformJointUpwardModeChange = NULL;
  delegate->mOnModeChangedCutJoints = NULL;
  delegate->mSetVariableModeJointsToLocked = NULL;
  delegate->mMech = NULL;
}

static
  void initSimulationDelegate(SmMechanismDelegate *delegate)
{
  SmMechanismDelegateScratchpad *scratchpad = NULL;
  static const SmSizePair jointToStageIdx[17] = {
    { 144, 1 }, { 145, 2 }, { 146, 3 }, { 147, 4 }, { 148, 5 }, { 149, 6 },

    { 150, 7 }, { 151, 8 }, { 296, 9 }, { 297, 10 }, { 298, 11 }, { 299, 12 },

    { 300, 13 }, { 301, 14 }, { 302, 15 }, { 303, 16 }, { 345, 0 }
  };

  static const size_t primitiveIndices[17 + 1] = {
    0, 0, 0, 1, 2, 3, 4, 5, 6, 6,
    6, 7, 8, 9, 10, 11, 12, 12
  };

  static const SmSizePair stateOffsets[12] = {
    { 0, 1 }, { 2, 3 }, { 4, 5 }, { 6, 7 }, { 8, 9 }, { 10, 11 },

    { 12, 13 }, { 14, 15 }, { 16, 17 }, { 18, 19 }, { 20, 21 }, { 22, 23 }
  };

  static const SmSizePair dofOffsets[12] = {
    { 0, 1 }, { 1, 2 }, { 2, 3 }, { 3, 4 }, { 4, 5 }, { 5, 6 },

    { 6, 7 }, { 7, 8 }, { 8, 9 }, { 9, 10 }, { 10, 11 }, { 11, 12 }
  };

  static const SmSizePair *flexBodyToStageIdx = NULL;
  static const SmSizePair *flexStateOffsets = NULL;
  static const size_t *flexibleStages = NULL;
  static const size_t remodIndices[12] = {
    0, 2, 4, 6, 8, 10, 12, 14, 16, 18,
    20, 22
  };

  static const size_t *equationsPerConstraint = NULL;
  static const int32_T *hasAllVelocityDisabledEquations = NULL;
  static const int32_T *runtimeEnabledEquations = NULL;
  static const size_t dofToVelSlot[12] = {
    1, 3, 5, 7, 9, 11, 13, 15, 17, 19,
    21, 23
  };

  static const size_t *constraintDofs = NULL;
  static const size_t constraintDofOffsets[0 + 1] = {
    0
  };

  const size_t Jm = 0;
  const size_t Jn = 12;
  SmSizePair zeroSizePair;
  zeroSizePair.mFirst = zeroSizePair.mSecond = 0;
  sm_core_MechanismDelegate_allocScratchpad(delegate);
  scratchpad = delegate->mScratchpad;
  delegate->mTargetStrengthFree = 0;
  delegate->mTargetStrengthSuggested = 1;
  delegate->mTargetStrengthDesired = 2;
  delegate->mTargetStrengthRequired = 3;
  delegate->mConsistencyTol = +1.000000000000000062e-09;
  delegate->mTreeJointDof = 12;
  delegate->mDof = 12;
  delegate->mStateSize = 24;
  delegate->mContinuousStateSize = 24;
  delegate->mModeVectorSize = 0;
  delegate->mNumStages = 17;
  delegate->mNumConstraints = 0;
  delegate->mNumAllConstraintEquations = 0;
  sm_core_SmSizePairVector_create(
    &delegate->mJointToStageIdx, 17, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mJointToStageIdx),
         jointToStageIdx, 17 * sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mPrimitiveIndices, delegate->mNumStages + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mPrimitiveIndices),
         primitiveIndices, (delegate->mNumStages + 1) * sizeof(size_t));
  sm_core_SmSizePairVector_create(
    &delegate->mStateOffsets, 12, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mStateOffsets),
         stateOffsets, 12 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mDofOffsets, 12, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mDofOffsets),
         dofOffsets, 12 * sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mFlexBodyToStageIdx, 0, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mFlexBodyToStageIdx),
         flexBodyToStageIdx, 0 *sizeof(SmSizePair));
  sm_core_SmSizePairVector_create(
    &delegate->mFlexStateOffsets, 0, &zeroSizePair);
  memcpy(sm_core_SmSizePairVector_nonConstValues(&delegate->mFlexStateOffsets),
         flexStateOffsets, 0 *sizeof(SmSizePair));
  sm_core_SmSizeTVector_create(
    &delegate->mFlexibleStages, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mFlexibleStages),
         flexibleStages, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mRemodIndices, 12, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mRemodIndices),
         remodIndices, 12 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mEquationsPerConstraint, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mEquationsPerConstraint),
         equationsPerConstraint, delegate->mNumConstraints * sizeof(size_t));
  sm_core_SmIntVector_create(
    &delegate->mHasAllVelocityDisabledEquations, delegate->mNumConstraints, 0);
  memcpy(sm_core_SmIntVector_nonConstValues
         (&delegate->mHasAllVelocityDisabledEquations),
         hasAllVelocityDisabledEquations, delegate->mNumConstraints * sizeof
         (int32_T));
  sm_core_SmIntVector_create(
    &delegate->mRunTimeEnabledEquations,
    delegate->mNumAllConstraintEquations, 0);
  memcpy(
         sm_core_SmIntVector_nonConstValues(&delegate->mRunTimeEnabledEquations),
         runtimeEnabledEquations, delegate->mNumAllConstraintEquations *
         sizeof(int32_T));
  sm_core_SmSizeTVector_create(
    &delegate->mDofToVelSlot, delegate->mDof, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mDofToVelSlot),
         dofToVelSlot, delegate->mDof * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofs, 0, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofs),
         constraintDofs, 0 * sizeof(size_t));
  sm_core_SmSizeTVector_create(
    &delegate->mConstraintDofOffsets, delegate->mNumConstraints + 1, 0);
  memcpy(sm_core_SmSizeTVector_nonConstValues(&delegate->mConstraintDofOffsets),
         constraintDofOffsets, (delegate->mNumConstraints + 1) * sizeof(size_t));
  sm_core_SmBoundedSet_create(&scratchpad->mPosRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosDesired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggested, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosNonRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mPosSuggAndFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelDesired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggested, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelNonRequired, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mVelSuggAndFree, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mConstraintFilter, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mActiveDofs0, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mNewConstraints, 0);
  sm_core_SmBoundedSet_create(&scratchpad->mNewDofs, 12);
  sm_core_SmBoundedSet_create(&scratchpad->mUnsatisfiedConstraints, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveConstraintsVect,
    0, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mActiveDofsVect, 12, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mFullDofToActiveDof, 12, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyPosTargetedPrims, 12, &zeroSizePair);
  sm_core_SmSizePairVector_create(
    &scratchpad->mPartiallyVelTargetedPrims, 12, &zeroSizePair);
  sm_core_SmSizeTVector_create(&scratchpad->mPosPartialTypes, 12, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mVelPartialTypes, 12, 0);
  sm_core_SmSizeTVector_create(&scratchpad->mPartiallyActivePrims, 12, 0);
  sm_core_SmSizePairVector_create(
    &scratchpad->mBaseFrameVelOffsets, 0, &zeroSizePair);
  sm_core_SmSizePairVector_create(&scratchpad->mCvQuaternionVelOffsets,
    0,
    &zeroSizePair);
  sm_core_SmRealVector_create(&scratchpad->mCvQuaternionAzimuthValues,
    0, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mInitialState, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mStartState, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mTestState, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mFullStateVector, 24, 0.0);
  sm_core_SmIntVector_create(&scratchpad->mModeVector, 0, 0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianRowMaj, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobian, Jm * Jn, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mJacobianPrimSubmatrix, Jm * 6, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintNonhomoTerms, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mBestConstraintError, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mDeltas,
    Jn * (Jm <= Jn ? Jm : Jn), 0.0);
  sm_core_SmRealVector_create(&scratchpad->mSvdWork, 169, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchScaledDeltaVect, 12, 0.0);
  sm_core_SmRealVector_create(
    &scratchpad->mLineSearchTestStateVect, 24, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mLineSearchErrorVect, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mActiveDofVelsVect, 12, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mVelSystemRhs, Jm, 0.0);
  sm_core_SmRealVector_create(&scratchpad->mMotionData, 333, 0.0);
  delegate->mSetTargets = NULL;
  delegate->mResetStateVector = FD_nom_552be714_1_resetSimStateVector;
  delegate->mInitializeTrackedAngleState = NULL;
  delegate->mComputeDiscreteState = NULL;
  delegate->mAdjustPosition = NULL;
  delegate->mPerturbJointPrimitiveState =
    FD_nom_552be714_1_perturbSimJointPrimitiveState;
  delegate->mPerturbFlexibleBodyState =
    FD_nom_552be714_1_perturbFlexibleBodyState;
  delegate->mComputePosDofBlendMatrix = NULL;
  delegate->mComputeVelDofBlendMatrix = NULL;
  delegate->mProjectPartiallyTargetedPos = NULL;
  delegate->mPropagateMotion = NULL;
  delegate->mComputeAssemblyPosError = NULL;
  delegate->mComputeAssemblyJacobian = NULL;
  delegate->mComputeFullAssemblyJacobian = NULL;
  delegate->mIsInKinematicSingularity = NULL;
  delegate->mConvertStateVector = NULL;
  delegate->mConstructStateVector = FD_nom_552be714_1_constructStateVector;
  delegate->mExtractSolverStateVector =
    FD_nom_552be714_1_extractSolverStateVector;
  delegate->mExtractDiscreteStateVector =
    FD_nom_552be714_1_extractDiscreteStateVector;
  delegate->mIsPositionViolation = FD_nom_552be714_1_isPositionViolation;
  delegate->mIsVelocityViolation = FD_nom_552be714_1_isVelocityViolation;
  delegate->mProjectStateSim = FD_nom_552be714_1_projectStateSim;
  delegate->mComputeConstraintError = FD_nom_552be714_1_computeConstraintError;
  delegate->mResetModeVector = FD_nom_552be714_1_resetModeVector;
  delegate->mHasJointUpwardModeChange =
    FD_nom_552be714_1_hasJointUpwardModeChange;
  delegate->mPerformJointUpwardModeChange =
    FD_nom_552be714_1_performJointUpwardModeChange;
  delegate->mOnModeChangedCutJoints = FD_nom_552be714_1_onModeChangedCutJoints;
  delegate->mSetVariableModeJointsToLocked =
    FD_nom_552be714_1_setVariableModeJointsToLocked;
  delegate->mMech = NULL;
}

static
  void initMechanismDelegates(NeDaePrivateData *smData)
{
  PmAllocator *alloc = pm_default_allocator();
  static const size_t *motionInputOffsets = NULL;
  static const boolean_T *maybeLockedPrimHasModes = NULL;
  static const size_t *maybeLockedPrimModeOffsets = NULL;
  int_T status = 0;
  initAssemblyDelegate(&smData->mAssemblyDelegate);
  initSimulationDelegate(&smData->mSimulationDelegate);
  status = pm_create_size_vector_fields(
    &smData->mMotionInputOffsets, smData->mNumMotionInputPrimitives, alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mMotionInputOffsets.mX, motionInputOffsets,
         0 * sizeof(size_t));
  status = pm_create_bool_vector_fields(
    &smData->mMaybeLockedPrimHasModes, smData->mNumMaybeLockedPrimitives,
    alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mMaybeLockedPrimHasModes.mX, maybeLockedPrimHasModes,
         0 * sizeof(boolean_T));
  status = pm_create_size_vector_fields(
    &smData->mMaybeLockedPrimModeOffsets, smData->mNumMaybeLockedPrimitives,
    alloc);
  checkMemAllocStatus(status);
  memcpy(smData->mMaybeLockedPrimModeOffsets.mX, maybeLockedPrimModeOffsets,
         0 * sizeof(size_t));
}

static
  void initComputationFcnPtrs(NeDaePrivateData *smData)
{
  smData->mSetParametersFcn = dae_cg_setParameters_function;
  smData->mPAssertFcn = dae_cg_pAssert_method;
  smData->mDerivativeFcn = dae_cg_deriv_method;
  smData->mNumJacPerturbLoBoundsFcn = dae_cg_numJacPerturbLoBounds_method;
  smData->mNumJacPerturbHiBoundsFcn = dae_cg_numJacPerturbHiBounds_method;
  smData->mOutputFcn = dae_cg_compOutputs_method;
  smData->mModeFcn = dae_cg_mode_method;
  smData->mZeroCrossingFcn = dae_cg_zeroCrossing_method;
  smData->mProjectionFcn = dae_cg_project_solve;
  smData->mCIC_MODE_Fcn = dae_cg_CIC_MODE_solve;
  smData->mCheckFcn =
    (smData->mStateVectorSize == 0) ? dae_cg_check_solve : NULL;
  smData->mAssemblyFcn = dae_cg_assemble_solve;
  smData->mSetupLoggerFcn = dae_cg_setupLoggerFcn;
  smData->mLogFcn = dae_cg_recordLog_method;
  smData->mResidualsFcn = NULL;
  smData->mLinearizeFcn = NULL;
  smData->mGenerateFcn = NULL;
}

static
  void initLiveLinkToSm(NeDaePrivateData *smData)
{
  smData->mLiveSmLink = NULL;
  smData->mLiveSmLink_destroy = NULL;
  smData->mLiveSmLink_copy = NULL;
}

void FD_nom_552be714_1_NeDaePrivateData_create(NeDaePrivateData *smData)
{
  initBasicAttributes (smData);
  initStateVector (smData);
  initAsserts (smData);
  initModeVector (smData);
  initZeroCrossings (smData);
  initVariables (smData);
  initRuntimeParameters (smData);
  initIoInfo (smData);
  initInputDerivs (smData);
  initDirectFeedthrough (smData);
  initOutputDerivProc (smData);
  initMechanismDelegates (smData);
  initComputationFcnPtrs (smData);
  initLiveLinkToSm (smData);
}
