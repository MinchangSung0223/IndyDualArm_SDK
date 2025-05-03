#define S_FUNCTION_LEVEL               2
#define S_FUNCTION_NAME                FK_cgxe
#include "simstruc.h"
#include "FK_cgxe.h"
#define MDL_START

static void mdlStart(SimStruct* S)
{
  unsigned int success;
  success = cgxe_FK_method_dispatcher(S, SS_CALL_MDL_START, NULL);
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
  mxArray* mxModules = mxCreateCellMatrix(5, 1);
  mxArray* mxChksum = NULL;
  uint32_T* checksumData = NULL;
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 173167807;
  checksumData[1] = 605056185;
  checksumData[2] = 2322253468;
  checksumData[3] = 2221260243;
  mxSetCell(mxModules, 0, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 2035886445;
  checksumData[1] = 177704183;
  checksumData[2] = 3437916696;
  checksumData[3] = 1220199086;
  mxSetCell(mxModules, 1, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 2705541145;
  checksumData[1] = 3741194739;
  checksumData[2] = 1351966242;
  checksumData[3] = 1728866603;
  mxSetCell(mxModules, 2, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 3384544997;
  checksumData[1] = 1857972302;
  checksumData[2] = 473354016;
  checksumData[3] = 416662537;
  mxSetCell(mxModules, 3, mxChksum);
  mxChksum = mxCreateNumericMatrix(1, 4, mxUINT32_CLASS, mxREAL);
  checksumData = (uint32_T*) mxGetData(mxChksum);
  checksumData[0] = 3973810701;
  checksumData[1] = 1371350547;
  checksumData[2] = 290273119;
  checksumData[3] = 1585343220;
  mxSetCell(mxModules, 4, mxChksum);
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
    checksumData[0] = 1983621850;
    checksumData[1] = 700076532;
    checksumData[2] = 2931111172;
    checksumData[3] = 2475764535;
    mxSetField(mxChecksum, 0, "model", mxModelChksum);
  }

  {
    mxArray* mxMakefileChksum = mxCreateDoubleMatrix(1, 4, mxREAL);
    double* checksumData = (double*) mxGetData(mxMakefileChksum);
    checksumData[0] = 4121657494;
    checksumData[1] = 321740908;
    checksumData[2] = 273670433;
    checksumData[3] = 4287263120;
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
    checksumData[0] = 1949325540;
    checksumData[1] = 1107459229;
    checksumData[2] = 4240957314;
    checksumData[3] = 2927124089;
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
  success = cgxe_FK_method_dispatcher(S, SS_CALL_MDL_GET_SIM_STATE, (void *)
    (plhs));
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
  success = cgxe_FK_method_dispatcher(S, SS_CALL_MDL_SET_SIM_STATE, (void *)
    prhs[2]);
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
  if (strcmp(tpChksum, "LJRV8xaUaAupvDb6yXgyhD") == 0) {
    extern mxArray *cgxe_LJRV8xaUaAupvDb6yXgyhD_BuildInfoUpdate(void);
    plhs[0] = cgxe_LJRV8xaUaAupvDb6yXgyhD_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "4dIWKdXw9f42uXOYIbe9fE") == 0) {
    extern mxArray *cgxe_4dIWKdXw9f42uXOYIbe9fE_BuildInfoUpdate(void);
    plhs[0] = cgxe_4dIWKdXw9f42uXOYIbe9fE_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "lxQ3kncuZwyT2eLDpVVMcD") == 0) {
    extern mxArray *cgxe_lxQ3kncuZwyT2eLDpVVMcD_BuildInfoUpdate(void);
    plhs[0] = cgxe_lxQ3kncuZwyT2eLDpVVMcD_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "VuWzy0sCJJluCsE5Tx8UrF") == 0) {
    extern mxArray *cgxe_VuWzy0sCJJluCsE5Tx8UrF_BuildInfoUpdate(void);
    plhs[0] = cgxe_VuWzy0sCJJluCsE5Tx8UrF_BuildInfoUpdate();
    return 1;
  }

  if (strcmp(tpChksum, "IBRnOsQNtYL7X0HdVTMLIH") == 0) {
    extern mxArray *cgxe_IBRnOsQNtYL7X0HdVTMLIH_BuildInfoUpdate(void);
    plhs[0] = cgxe_IBRnOsQNtYL7X0HdVTMLIH_BuildInfoUpdate();
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
  if (strcmp(tpChksum, "LJRV8xaUaAupvDb6yXgyhD") == 0) {
    extern mxArray *cgxe_LJRV8xaUaAupvDb6yXgyhD_fallback_info(void);
    plhs[0] = cgxe_LJRV8xaUaAupvDb6yXgyhD_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "4dIWKdXw9f42uXOYIbe9fE") == 0) {
    extern mxArray *cgxe_4dIWKdXw9f42uXOYIbe9fE_fallback_info(void);
    plhs[0] = cgxe_4dIWKdXw9f42uXOYIbe9fE_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "lxQ3kncuZwyT2eLDpVVMcD") == 0) {
    extern mxArray *cgxe_lxQ3kncuZwyT2eLDpVVMcD_fallback_info(void);
    plhs[0] = cgxe_lxQ3kncuZwyT2eLDpVVMcD_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "VuWzy0sCJJluCsE5Tx8UrF") == 0) {
    extern mxArray *cgxe_VuWzy0sCJJluCsE5Tx8UrF_fallback_info(void);
    plhs[0] = cgxe_VuWzy0sCJJluCsE5Tx8UrF_fallback_info();
    return 1;
  }

  if (strcmp(tpChksum, "IBRnOsQNtYL7X0HdVTMLIH") == 0) {
    extern mxArray *cgxe_IBRnOsQNtYL7X0HdVTMLIH_fallback_info(void);
    plhs[0] = cgxe_IBRnOsQNtYL7X0HdVTMLIH_fallback_info();
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
