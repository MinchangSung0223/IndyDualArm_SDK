//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_log6_api.cpp
//
// Code generation for function 'log6'
//

// Include files
#include "_coder_log6_api.h"
#include "_coder_log6_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131659U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "log6",                                               // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16];

static void emlrtExitTimeCleanupDtorFcn(const void *r);

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[16];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[16];

static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const creal_T u[6]);

// Function Definitions
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16]
{
  static const int32_T dims[2]{4, 4};
  real_T(*ret)[16];
  int32_T iv[2];
  boolean_T bv[2]{false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[16])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void emlrtExitTimeCleanupDtorFcn(const void *r)
{
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[16]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[16];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[16]
{
  real_T(*y)[16];
  y = b_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const emlrtStack &sp,
                                        const creal_T u[6])
{
  static const int32_T i{6};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxCOMPLEX);
  emlrtExportNumericArrayR2013b((emlrtConstCTX)&sp, m, (const void *)&u[0], 8);
  emlrtAssign(&y, m);
  return y;
}

void log6_api(const mxArray *prhs, const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  creal_T lambda[6];
  real_T(*T)[16];
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  T = emlrt_marshallIn(st, emlrtAlias(prhs), "T");
  // Invoke the target function
  log6(*T, lambda);
  // Marshall function outputs
  *plhs = emlrt_marshallOut(st, lambda);
}

void log6_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtPushHeapReferenceStackR2021a(&st, false, nullptr,
                                    (void *)&emlrtExitTimeCleanupDtorFcn,
                                    nullptr, nullptr, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  log6_xil_terminate();
  log6_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void log6_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void log6_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (_coder_log6_api.cpp)
