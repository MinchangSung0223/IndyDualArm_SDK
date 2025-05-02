#define S_FUNCTION_LEVEL               2
#define S_FUNCTION_NAME                indy7_dualArm_cgxe
#include "simstruc.h"
#include "indy7_dualArm_cgxe.h"
#define MDL_START

static void mdlStart(SimStruct* S)
{
  unsigned int success;
  success = cgxe_indy7_dualArm_method_dispatcher(S, SS_CALL_MDL_START, NULL);
  if (!success) {
    /* error */
    mexPrintf("ERROR: Failed to dispatch s-function method!\n");
  }
}

#define MDL_INITIALIZE_CONDITIONS

static void mdlInitializeConditions(SimStruct *S)
{
  mexPrintf("ERROR: Calling model mdlInitializeConditions method directly.\n");
}

#define MDL_UPDATE

static void mdlUpdate(SimStruct *S, int_T tid)
{
  mexPrintf("ERROR: Calling model mdlUpdate method directly.\n");
}

static void mdlOutputs(SimStruct* S, int_T tid)
{
  mexPrintf("ERROR: Calling model mdlOutputs method directly.\n");
}

static void mdlTerminate(SimStruct *S)
{
  mexPrintf("ERROR: Calling model mdlTerminate method directly.\n");
}

static void mdlInitializeSizes(SimStruct *S)
{
}

static void mdlInitializeSampleTimes(SimStruct *S)
{
}

static mxArray* cgxe_get_supported_modules(void)
{
  mxArray* mxModules = mxCreateCellMatrix(8, 1);
  mxArray* mxChksum = NULL;
  uint32_T* checksumData = NULL;
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 178817427;
  checksumData[1] = 4072434611;
  checksumData[2] = 4068674943;
  checksumData[3] = 1436499906;
  mxSetCell(mxModules, 0, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 1114460459;
  checksumData[1] = 1701144773;
  checksumData[2] = 1766980214;
  checksumData[3] = 2101927784;
  mxSetCell(mxModules, 1, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 2122025888;
  checksumData[1] = 3622931021;
  checksumData[2] = 2709846304;
  checksumData[3] = 3490596982;
  mxSetCell(mxModules, 2, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 2414102413;
  checksumData[1] = 814913094;
  checksumData[2] = 2581825787;
  checksumData[3] = 1628411881;
  mxSetCell(mxModules, 3, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 3148610900;
  checksumData[1] = 2301114800;
  checksumData[2] = 3139047242;
  checksumData[3] = 212001703;
  mxSetCell(mxModules, 4, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 3521690577;
  checksumData[1] = 1375613089;
  checksumData[2] = 3985637424;
  checksumData[3] = 4155627802;
  mxSetCell(mxModules, 5, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 3530623186;
  checksumData[1] = 3648337709;
  checksumData[2] = 1100901509;
  checksumData[3] = 2499504597;
  mxSetCell(mxModules, 6, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 3547873016;
  checksumData[1] = 3695362885;
  checksumData[2] = 502855996;
  checksumData[3] = 1209443866;
  mxSetCell(mxModules, 7, mxChksum);
  return mxModules;
}

static int cgxe_process_get_checksums(int nlhs, mxArray* plhs[], int nrhs, const
  mxArray* prhs[])
{
  const char* checksumFields[] = { "modules", "model", "makefile", "target",
    "overall" };

  mxArray* mxChecksum = mxCreateStructMatrix(1, 1, 5, checksumFields);
  mxSetField(mxChecksum, 0, "modules", cgxe_get_supported_modules());

  {
    mxArray* mxModelChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxModelChksum);
    checksumData[0] = 1371703865;
    checksumData[1] = 3300861469;
    checksumData[2] = 1287070248;
    checksumData[3] = 204515961;
    mxSetField(mxChecksum, 0, "model", mxModelChksum);
  }

  {
    mxArray* mxMakefileChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxMakefileChksum);
    checksumData[0] = 187007998;
    checksumData[1] = 2000170808;
    checksumData[2] = 2705413794;
    checksumData[3] = 918737752;
    mxSetField(mxChecksum, 0, "makefile", mxMakefileChksum);
  }

  {
    mxArray* mxTargetChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxTargetChksum);
    checksumData[0] = 2911221907;
    checksumData[1] = 2308967934;
    checksumData[2] = 2419390157;
    checksumData[3] = 1906300239;
    mxSetField(mxChecksum, 0, "target", mxTargetChksum);
  }

  {
    mxArray* mxOverallChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxOverallChksum);
    checksumData[0] = 1062354480;
    checksumData[1] = 3040430711;
    checksumData[2] = 617346622;
    checksumData[3] = 759740090;
    mxSetField(mxChecksum, 0, "overall", mxOverallChksum);
  }

  plhs[0] = mxChecksum;
  return 1;
}

static int cgxe_mex_unlock_call(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  while (mexIsLocked()) {
    mexUnlock();
  }

  return 1;
}

static SimStruct* cgxe_unpack_simstruct(const mxArray *mxS)
{
  uint32_T *uintPtr = (uint32_T*)malloc(sizeof(SimStruct*));
  int nEl = sizeof(SimStruct*)/sizeof(uint32_T);
  uint32_T *uintDataPtr = (uint32_T *)mxGetData(mxS);
  int el;
  SimStruct *S;
  for (el=0; el < nEl; el++) {
    uintPtr[el] = uintDataPtr[el];
  }

  memcpy(&S,uintPtr,sizeof(SimStruct*));
  free(uintPtr);
  return S;
}

static int cgxe_get_sim_state(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  unsigned int success;
  SimStruct *S = cgxe_unpack_simstruct(prhs[1]);
  success = cgxe_indy7_dualArm_method_dispatcher(S, SS_CALL_MDL_GET_SIM_STATE,
    (void *) (plhs));
  if (!success) {
    /* error */
    mexPrintf("ERROR: Failed to dispatch s-function method!\n");
  }

  return 1;
}

static int cgxe_set_sim_state(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  unsigned int success;
  SimStruct *S = cgxe_unpack_simstruct(prhs[1]);
  success = cgxe_indy7_dualArm_method_dispatcher(S, SS_CALL_MDL_SET_SIM_STATE,
    (void *) prhs[2]);
  if (!success) {
    /* error */
    mexPrintf("ERROR: Failed to dispatch s-function method!\n");
  }

  return 1;
}

static int cgxe_get_BuildInfoUpdate(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(tpChksum, "qq0HSn4MnMIEEMjNzXu2qD") == 0) {
    extern mxArray *cgxe_qq0HSn4MnMIEEMjNzXu2qD_BuildInfoUpdate(void);
    plhs[0] = cgxe_qq0HSn4MnMIEEMjNzXu2qD_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "T6MwjqiGx6loI7k5o1vVhG") == 0) {
    extern mxArray *cgxe_T6MwjqiGx6loI7k5o1vVhG_BuildInfoUpdate(void);
    plhs[0] = cgxe_T6MwjqiGx6loI7k5o1vVhG_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "TCGSFIrmgTrdrXjldiS8GC") == 0) {
    extern mxArray *cgxe_TCGSFIrmgTrdrXjldiS8GC_BuildInfoUpdate(void);
    plhs[0] = cgxe_TCGSFIrmgTrdrXjldiS8GC_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "87mWOnq5ANQkjTWghyXDKD") == 0) {
    extern mxArray *cgxe_87mWOnq5ANQkjTWghyXDKD_BuildInfoUpdate(void);
    plhs[0] = cgxe_87mWOnq5ANQkjTWghyXDKD_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "ixuv55C8EtZmU1WzAsjQDD") == 0) {
    extern mxArray *cgxe_ixuv55C8EtZmU1WzAsjQDD_BuildInfoUpdate(void);
    plhs[0] = cgxe_ixuv55C8EtZmU1WzAsjQDD_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "imQiRskwoVIlJDtKF8Ie9C") == 0) {
    extern mxArray *cgxe_imQiRskwoVIlJDtKF8Ie9C_BuildInfoUpdate(void);
    plhs[0] = cgxe_imQiRskwoVIlJDtKF8Ie9C_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "s5IVGQHtlmgQqKlp1mGyWG") == 0) {
    extern mxArray *cgxe_s5IVGQHtlmgQqKlp1mGyWG_BuildInfoUpdate(void);
    plhs[0] = cgxe_s5IVGQHtlmgQqKlp1mGyWG_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "cGalwQDtj5O4ZBPIMXFzfD") == 0) {
    extern mxArray *cgxe_cGalwQDtj5O4ZBPIMXFzfD_BuildInfoUpdate(void);
    plhs[0] = cgxe_cGalwQDtj5O4ZBPIMXFzfD_BuildInfoUpdate();
    return 1;
  }

  return 0;
}

static int cgxe_get_fallback_info(int nlhs, mxArray * plhs[], int nrhs, const
  mxArray * prhs[])
{
  char tpChksum[64];
  mxGetString(prhs[1], tpChksum,sizeof(tpChksum)/sizeof(char));
  tpChksum[(sizeof(tpChksum)/sizeof(char)-1)] = '\0';
  if (strcmp(tpChksum, "qq0HSn4MnMIEEMjNzXu2qD") == 0) {
    extern mxArray *cgxe_qq0HSn4MnMIEEMjNzXu2qD_fallback_info(void);
    plhs[0] = cgxe_qq0HSn4MnMIEEMjNzXu2qD_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "T6MwjqiGx6loI7k5o1vVhG") == 0) {
    extern mxArray *cgxe_T6MwjqiGx6loI7k5o1vVhG_fallback_info(void);
    plhs[0] = cgxe_T6MwjqiGx6loI7k5o1vVhG_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "TCGSFIrmgTrdrXjldiS8GC") == 0) {
    extern mxArray *cgxe_TCGSFIrmgTrdrXjldiS8GC_fallback_info(void);
    plhs[0] = cgxe_TCGSFIrmgTrdrXjldiS8GC_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "87mWOnq5ANQkjTWghyXDKD") == 0) {
    extern mxArray *cgxe_87mWOnq5ANQkjTWghyXDKD_fallback_info(void);
    plhs[0] = cgxe_87mWOnq5ANQkjTWghyXDKD_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "ixuv55C8EtZmU1WzAsjQDD") == 0) {
    extern mxArray *cgxe_ixuv55C8EtZmU1WzAsjQDD_fallback_info(void);
    plhs[0] = cgxe_ixuv55C8EtZmU1WzAsjQDD_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "imQiRskwoVIlJDtKF8Ie9C") == 0) {
    extern mxArray *cgxe_imQiRskwoVIlJDtKF8Ie9C_fallback_info(void);
    plhs[0] = cgxe_imQiRskwoVIlJDtKF8Ie9C_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "s5IVGQHtlmgQqKlp1mGyWG") == 0) {
    extern mxArray *cgxe_s5IVGQHtlmgQqKlp1mGyWG_fallback_info(void);
    plhs[0] = cgxe_s5IVGQHtlmgQqKlp1mGyWG_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "cGalwQDtj5O4ZBPIMXFzfD") == 0) {
    extern mxArray *cgxe_cGalwQDtj5O4ZBPIMXFzfD_fallback_info(void);
    plhs[0] = cgxe_cGalwQDtj5O4ZBPIMXFzfD_fallback_info();
    return 1;
  }

  return 0;
}

#define PROCESS_MEX_SFUNCTION_CMD_LINE_CALL

static int ProcessMexSfunctionCmdLineCall(int nlhs, mxArray* plhs[], int nrhs,
  const mxArray* prhs[])
{
  char commandName[64];
  if (nrhs < 1 || !mxIsChar(prhs[0]))
    return 0;
  mxGetString(prhs[0], commandName, sizeof(commandName)/sizeof(char));
  commandName[(sizeof(commandName)/sizeof(char)-1)] = '\0';
  if (strcmp(commandName, "get_checksums") == 0) {
    return cgxe_process_get_checksums(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "mex_unlock") == 0) {
    return cgxe_mex_unlock_call(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "get_sim_state") == 0) {
    return cgxe_get_sim_state(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "set_sim_state") == 0) {
    return cgxe_set_sim_state(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "get_BuildInfoUpdate") == 0) {
    return cgxe_get_BuildInfoUpdate(nlhs, plhs, nrhs, prhs);
  }

  if (strcmp(commandName, "get_fallback_info") == 0) {
    return cgxe_get_fallback_info(nlhs, plhs, nrhs, prhs);
  }

  return 0;
}

#include "simulink.c"
