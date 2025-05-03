/* Simscape target specific file.
 * This file is generated for the Simscape network associated with the solver block 'FD_nom/DualArm/Solver Configuration'.
 */

#ifdef MATLAB_MEX_FILE
#include "tmwtypes.h"
#else
#include "rtwtypes.h"
#endif

#include "nesl_rtw.h"
#include "FD_nom_552be714_1.h"
#include "FD_nom_552be714_1_gateway.h"

void* FD_nom_552be714_1_gateway(void)
{
  NeModelParameters modelparams = { (enum NeSolverTypeTag)1, 0.001, 0.001, 0.001,
    FALSE, FALSE, (enum NeModifyAbsTolTag)0, 0.001, 0.0, FALSE, FALSE, FALSE, (
    enum SscLoggingSettingTag)0, 668167703.0, TRUE, FALSE, FALSE };

  NeSolverParameters solverparams = { FALSE, FALSE, TRUE, FALSE, FALSE, 0.001,
    0.001, 1e-09, FALSE, FALSE, 100U, FALSE, 1U, (enum NeConsistencySolverTag)2,
    (enum NeIndexReductionMethodTag)1, FALSE, 1e-09, (enum NeToleranceSourceTag)
    1, 0.001, 0.001, 0.001, FALSE, (enum NeLocalSolverChoiceTag)0, TRUE, 1U,
    0.001, FALSE, 3U, 2U, FALSE, 2U, (enum NeLinearAlgebraChoiceTag)0, 0U, (enum
    NeEquationFormulationChoiceTag)0, 1024U, TRUE, 0.001, (enum
    NePartitionStorageMethodTag)0, 1024U, (enum NePartitionMethodTag)0, TRUE, (
    enum NeMultibodyLocalSolverChoiceTag)0, 0.001 };

  NeDae* dae;
  FD_nom_552be714_1_dae(&dae,
                        &modelparams,
                        &solverparams);
  return dae;
}
