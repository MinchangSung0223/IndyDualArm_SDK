/* This file was automatically generated by CasADi 3.6.7.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) MatrixExp3_ ## ID
#endif

#include <math.h>
#include <string.h>
#ifdef MATLAB_MEX_FILE
#include <mex.h>
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_c0 CASADI_PREFIX(c0)
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_dot CASADI_PREFIX(dot)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_f4 CASADI_PREFIX(f4)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_from_mex CASADI_PREFIX(from_mex)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_to_mex CASADI_PREFIX(to_mex)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

casadi_real casadi_dot(casadi_int n, const casadi_real* x, const casadi_real* y) {
  casadi_int i;
  casadi_real r = 0;
  for (i=0; i<n; ++i) r += *x++ * *y++;
  return r;
}

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

#ifdef MATLAB_MEX_FILE
casadi_real* casadi_from_mex(const mxArray* p, casadi_real* y, const casadi_int* sp, casadi_real* w) {
  casadi_int nrow, ncol, is_sparse, c, k, p_nrow, p_ncol;
  const casadi_int *colind, *row;
  mwIndex *Jc, *Ir;
  const double* p_data;
  if (!mxIsDouble(p) || mxGetNumberOfDimensions(p)!=2)
    mexErrMsgIdAndTxt("Casadi:RuntimeError",
      "\"from_mex\" failed: Not a two-dimensional matrix of double precision.");
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
  p_nrow = mxGetM(p);
  p_ncol = mxGetN(p);
  is_sparse = mxIsSparse(p);
  Jc = 0;
  Ir = 0;
  if (is_sparse) {
    Jc = mxGetJc(p);
    Ir = mxGetIr(p);
  }
  p_data = (const double*)mxGetData(p);
  if (p_nrow==1 && p_ncol==1) {
    casadi_int nnz;
    double v = is_sparse && Jc[1]==0 ? 0 : *p_data;
    nnz = sp[ncol];
    casadi_fill(y, nnz, v);
  } else {
    casadi_int tr = 0;
    if (nrow!=p_nrow || ncol!=p_ncol) {
      tr = nrow==p_ncol && ncol==p_nrow && (nrow==1 || ncol==1);
      if (!tr) mexErrMsgIdAndTxt("Casadi:RuntimeError",
                                 "\"from_mex\" failed: Dimension mismatch. "
                                 "Expected %d-by-%d, got %d-by-%d instead.",
                                 nrow, ncol, p_nrow, p_ncol);
    }
    if (is_sparse) {
      if (tr) {
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]+c*nrow]=0;
        for (c=0; c<p_ncol; ++c)
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[c+Ir[k]*p_ncol] = p_data[k];
        for (c=0; c<ncol; ++c)
          for (k=colind[c]; k<colind[c+1]; ++k) y[k] = w[row[k]+c*nrow];
      } else {
        for (c=0; c<ncol; ++c) {
          for (k=colind[c]; k<colind[c+1]; ++k) w[row[k]]=0;
          for (k=Jc[c]; k<(casadi_int) Jc[c+1]; ++k) w[Ir[k]]=p_data[k];
          for (k=colind[c]; k<colind[c+1]; ++k) y[k]=w[row[k]];
        }
      }
    } else {
      for (c=0; c<ncol; ++c) {
        for (k=colind[c]; k<colind[c+1]; ++k) {
          y[k] = p_data[row[k]+c*nrow];
        }
      }
    }
  }
  return y;
}

#endif

#define casadi_to_double(x) ((double) x)

#ifdef MATLAB_MEX_FILE
mxArray* casadi_to_mex(const casadi_int* sp, const casadi_real* x) {
  casadi_int nrow, ncol, c, k;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int nnz;
#endif
  const casadi_int *colind, *row;
  mxArray *p;
  double *d;
#ifndef CASADI_MEX_NO_SPARSE
  casadi_int i;
  mwIndex *j;
#endif /* CASADI_MEX_NO_SPARSE */
  nrow = *sp++;
  ncol = *sp++;
  colind = sp;
  row = sp+ncol+1;
#ifndef CASADI_MEX_NO_SPARSE
  nnz = sp[ncol];
  if (nnz!=nrow*ncol) {
    p = mxCreateSparse(nrow, ncol, nnz, mxREAL);
    for (i=0, j=mxGetJc(p); i<=ncol; ++i) *j++ = *colind++;
    for (i=0, j=mxGetIr(p); i<nnz; ++i) *j++ = *row++;
    if (x) {
      d = (double*)mxGetData(p);
      for (i=0; i<nnz; ++i) *d++ = casadi_to_double(*x++);
    }
    return p;
  }
#endif /* CASADI_MEX_NO_SPARSE */
  p = mxCreateDoubleMatrix(nrow, ncol, mxREAL);
  if (x) {
    d = (double*)mxGetData(p);
    for (c=0; c<ncol; ++c) {
      for (k=colind[c]; k<colind[c+1]; ++k) {
        d[row[k]+c*nrow] = casadi_to_double(*x++);
      }
    }
  }
  return p;
}

#endif

static const casadi_int casadi_s0[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};

static const casadi_real casadi_c0[9] = {1., 0., 0., 0., 1., 0., 0., 0., 1.};

/* so3ToVec:(i0[3x3])->(o0[3]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  casadi_real *w0=w+0, w1;
  /* #0: @0 = input[0][0] */
  casadi_copy(arg[0], 9, w0);
  /* #1: @1 = @0[5] */
  for (rr=(&w1), ss=w0+5; ss!=w0+6; ss+=1) *rr++ = *ss;
  /* #2: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  /* #3: @1 = @0[6] */
  for (rr=(&w1), ss=w0+6; ss!=w0+7; ss+=1) *rr++ = *ss;
  /* #4: output[0][1] = @1 */
  if (res[0]) res[0][1] = w1;
  /* #5: @1 = @0[1] */
  for (rr=(&w1), ss=w0+1; ss!=w0+2; ss+=1) *rr++ = *ss;
  /* #6: output[0][2] = @1 */
  if (res[0]) res[0][2] = w1;
  return 0;
}

/* NearZero:(i0)->(o0) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = ||@0||_F */
  w1 = sqrt(casadi_dot(1, (&w0), (&w0)));
  /* #2: @0 = 2.22045e-16 */
  w0 = 2.2204460492503131e-16;
  /* #3: @1 = (@1<@0) */
  w1  = (w1<w0);
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* AxisAng3:(i0[3])->(o0[3x3],o1) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real **res1=res+2, *rr;
  const casadi_real **arg1=arg+1, *cs;
  casadi_real *w0=w+2, w1, w2, *w3=w+7;
  /* #0: @0 = input[0][0] */
  casadi_copy(arg[0], 3, w0);
  /* #1: @1 = ||@0||_F */
  w1 = sqrt(casadi_dot(3, w0, w0));
  /* #2: @2 = NearZero(@1) */
  arg1[0]=(&w1);
  res1[0]=(&w2);
  if (casadi_f3(arg1, res1, iw, w, 0)) return 1;
  /* #3: @2 = (!@2) */
  w2 = (! w2 );
  /* #4: @0 = (@0/@1) */
  for (i=0, rr=w0; i<3; ++i) (*rr++) /= w1;
  /* #5: @0 = (@2?@0:0) */
  for (i=0, rr=w0, cs=w0; i<3; ++i) (*rr++)  = (w2?cs[i]:0);
  /* #6: @3 = repmat(@0, 3) */
  for (i=0;i<3;++i) {
    casadi_copy(w0, 3, w3+ i*3);
  }
  /* #7: output[0][0] = @3 */
  casadi_copy(w3, 9, res[0]);
  /* #8: output[1][0] = @1 */
  if (res[1]) res[1][0] = w1;
  return 0;
}

/* NearZero:(i0)->(o0) */
static int casadi_f4(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1;
  /* #0: @0 = input[0][0] */
  w0 = arg[0] ? arg[0][0] : 0;
  /* #1: @1 = ||@0||_F */
  w1 = sqrt(casadi_dot(1, (&w0), (&w0)));
  /* #2: @0 = 2.22045e-16 */
  w0 = 2.2204460492503131e-16;
  /* #3: @1 = (@1<@0) */
  w1  = (w1<w0);
  /* #4: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  return 0;
}

/* MatrixExp3:(i0[3x3])->(o0[3x3]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real **res1=res+1, *rr, *ss, *tt;
  const casadi_real **arg1=arg+1, *cs;
  casadi_real *w0=w+16, *w1=w+25, w2, w3, *w4=w+30, *w5=w+39, w6, *w7=w+49, w8, *w9=w+59;
  /* #0: @0 = input[0][0] */
  casadi_copy(arg[0], 9, w0);
  /* #1: @1 = so3ToVec(@0) */
  arg1[0]=w0;
  res1[0]=w1;
  if (casadi_f1(arg1, res1, iw, w, 0)) return 1;
  /* #2: {NULL, @2} = AxisAng3(@1) */
  arg1[0]=w1;
  res1[0]=0;
  res1[1]=(&w2);
  if (casadi_f2(arg1, res1, iw, w, 0)) return 1;
  /* #3: @3 = NearZero(@2) */
  arg1[0]=(&w2);
  res1[0]=(&w3);
  if (casadi_f4(arg1, res1, iw, w, 0)) return 1;
  /* #4: @4 = 
  [[1, 0, 0], 
   [0, 1, 0], 
   [0, 0, 1]] */
  casadi_copy(casadi_c0, 9, w4);
  /* #5: @4 = (@3?@4:0) */
  for (i=0, rr=w4, cs=w4; i<9; ++i) (*rr++)  = (w3?cs[i]:0);
  /* #6: @3 = (!@3) */
  w3 = (! w3 );
  /* #7: @5 = 
  [[1, 0, 0], 
   [0, 1, 0], 
   [0, 0, 1]] */
  casadi_copy(casadi_c0, 9, w5);
  /* #8: @6 = sin(@2) */
  w6 = sin( w2 );
  /* #9: @6 = (@6/@2) */
  w6 /= w2;
  /* #10: @7 = (@6*@0) */
  for (i=0, rr=w7, cs=w0; i<9; ++i) (*rr++)  = (w6*(*cs++));
  /* #11: @5 = (@5+@7) */
  for (i=0, rr=w5, cs=w7; i<9; ++i) (*rr++) += (*cs++);
  /* #12: @7 = zeros(3x3) */
  casadi_clear(w7, 9);
  /* #13: @6 = 1 */
  w6 = 1.;
  /* #14: @8 = cos(@2) */
  w8 = cos( w2 );
  /* #15: @6 = (@6-@8) */
  w6 -= w8;
  /* #16: @6 = (@6/@2) */
  w6 /= w2;
  /* #17: @6 = (@6/@2) */
  w6 /= w2;
  /* #18: @9 = (@6*@0) */
  for (i=0, rr=w9, cs=w0; i<9; ++i) (*rr++)  = (w6*(*cs++));
  /* #19: @7 = mac(@9,@0,@7) */
  for (i=0, rr=w7; i<3; ++i) for (j=0; j<3; ++j, ++rr) for (k=0, ss=w9+j, tt=w0+i*3; k<3; ++k) *rr += ss[k*3]**tt++;
  /* #20: @5 = (@5+@7) */
  for (i=0, rr=w5, cs=w7; i<9; ++i) (*rr++) += (*cs++);
  /* #21: @5 = (@3?@5:0) */
  for (i=0, rr=w5, cs=w5; i<9; ++i) (*rr++)  = (w3?cs[i]:0);
  /* #22: @4 = (@4+@5) */
  for (i=0, rr=w4, cs=w5; i<9; ++i) (*rr++) += (*cs++);
  /* #23: output[0][0] = @4 */
  casadi_copy(w4, 9, res[0]);
  return 0;
}

CASADI_SYMBOL_EXPORT int MatrixExp3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int MatrixExp3_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int MatrixExp3_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void MatrixExp3_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int MatrixExp3_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void MatrixExp3_release(int mem) {
}

CASADI_SYMBOL_EXPORT void MatrixExp3_incref(void) {
}

CASADI_SYMBOL_EXPORT void MatrixExp3_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int MatrixExp3_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int MatrixExp3_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real MatrixExp3_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* MatrixExp3_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* MatrixExp3_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* MatrixExp3_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* MatrixExp3_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int MatrixExp3_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 68;
  return 0;
}

CASADI_SYMBOL_EXPORT int MatrixExp3_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 5*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 68*sizeof(casadi_real);
  return 0;
}

#ifdef MATLAB_MEX_FILE
void mex_MatrixExp3(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  casadi_int i;
  int mem;
  casadi_real w[86];
  casadi_int *iw = 0;
  const casadi_real* arg[5] = {0};
  casadi_real* res[5] = {0};
  if (argc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"MatrixExp3\" failed. Too many input arguments (%d, max 1)", argc);
  if (resc>1) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"MatrixExp3\" failed. Too many output arguments (%d, max 1)", resc);
  if (--argc>=0) arg[0] = casadi_from_mex(argv[0], w, casadi_s0, w+18);
  --resc;
  res[0] = w+9;
  MatrixExp3_incref();
  mem = MatrixExp3_checkout();
  i = MatrixExp3(arg, res, iw, w+18, mem);
  if (i) mexErrMsgIdAndTxt("Casadi:RuntimeError","Evaluation of \"MatrixExp3\" failed.");
  MatrixExp3_release(mem);
  MatrixExp3_decref();
  if (res[0]) resv[0] = casadi_to_mex(casadi_s0, res[0]);
}
#endif


#ifdef MATLAB_MEX_FILE
void mexFunction(int resc, mxArray *resv[], int argc, const mxArray *argv[]) {
  char buf[11];
  int buf_ok = argc > 0 && !mxGetString(*argv, buf, sizeof(buf));
  if (!buf_ok) {
    mex_MatrixExp3(resc, resv, argc, argv);
    return;
  } else if (strcmp(buf, "MatrixExp3")==0) {
    mex_MatrixExp3(resc, resv, argc-1, argv+1);
    return;
  }
  mexErrMsgTxt("First input should be a command string. Possible values: 'MatrixExp3'");
}
#endif
#ifdef __cplusplus
} /* extern "C" */
#endif
