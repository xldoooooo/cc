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
  #define CASADI_PREFIX(ID) NMPC_model_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[28] = {24, 1, 0, 24, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
static const casadi_int casadi_s1[12] = {8, 1, 0, 8, 0, 1, 2, 3, 4, 5, 6, 7};
static const casadi_int casadi_s2[3] = {0, 0, 0};

/* NMPC_model_expl_ode_fun:(i0[24],i1[8],i2[])->(o0[24]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][12] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0]? arg[0][13] : 0;
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[0]? arg[0][14] : 0;
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[0]? arg[0][16] : 0;
  a1=arg[0]? arg[0][5] : 0;
  a2=(a0*a1);
  a3=arg[0]? arg[0][17] : 0;
  a4=arg[0]? arg[0][4] : 0;
  a5=(a3*a4);
  a2=(a2-a5);
  if (res[0]!=0) res[0][3]=a2;
  a2=arg[0]? arg[0][3] : 0;
  a3=(a3*a2);
  a5=arg[0]? arg[0][15] : 0;
  a6=(a5*a1);
  a3=(a3-a6);
  if (res[0]!=0) res[0][4]=a3;
  a5=(a5*a4);
  a0=(a0*a2);
  a5=(a5-a0);
  if (res[0]!=0) res[0][5]=a5;
  a5=arg[0]? arg[0][19] : 0;
  a0=arg[0]? arg[0][8] : 0;
  a3=(a5*a0);
  a6=arg[0]? arg[0][20] : 0;
  a7=arg[0]? arg[0][7] : 0;
  a8=(a6*a7);
  a3=(a3-a8);
  if (res[0]!=0) res[0][6]=a3;
  a3=arg[0]? arg[0][6] : 0;
  a6=(a6*a3);
  a8=arg[0]? arg[0][18] : 0;
  a9=(a8*a0);
  a6=(a6-a9);
  if (res[0]!=0) res[0][7]=a6;
  a8=(a8*a7);
  a5=(a5*a3);
  a8=(a8-a5);
  if (res[0]!=0) res[0][8]=a8;
  a8=arg[0]? arg[0][22] : 0;
  a5=arg[0]? arg[0][11] : 0;
  a6=(a8*a5);
  a9=arg[0]? arg[0][23] : 0;
  a10=arg[0]? arg[0][10] : 0;
  a11=(a9*a10);
  a6=(a6-a11);
  if (res[0]!=0) res[0][9]=a6;
  a6=arg[0]? arg[0][9] : 0;
  a9=(a9*a6);
  a11=arg[0]? arg[0][21] : 0;
  a12=(a11*a5);
  a9=(a9-a12);
  if (res[0]!=0) res[0][10]=a9;
  a11=(a11*a10);
  a8=(a8*a6);
  a11=(a11-a8);
  if (res[0]!=0) res[0][11]=a11;
  a11=arg[1]? arg[1][6] : 0;
  a8=4.4500000000000001e-01;
  a9=(a11/a8);
  a12=(a9*a3);
  a13=arg[1]? arg[1][7] : 0;
  a8=(a13/a8);
  a14=(a8*a6);
  a12=(a12+a14);
  a12=(-a12);
  if (res[0]!=0) res[0][12]=a12;
  a12=(a9*a7);
  a14=(a8*a10);
  a12=(a12+a14);
  a12=(-a12);
  if (res[0]!=0) res[0][13]=a12;
  a12=9.8100000000000005e+00;
  a9=(a9*a0);
  a8=(a8*a5);
  a9=(a9+a8);
  a12=(a12-a9);
  if (res[0]!=0) res[0][14]=a12;
  a12=1.4833333333333332e-01;
  a11=(a11/a12);
  a9=(a11*a1);
  a8=(a9*a7);
  a14=(a11*a4);
  a15=(a14*a0);
  a8=(a8-a15);
  a13=(a13/a12);
  a4=(a13*a4);
  a12=(a4*a5);
  a1=(a13*a1);
  a15=(a1*a10);
  a12=(a12-a15);
  a8=(a8+a12);
  if (res[0]!=0) res[0][15]=a8;
  a11=(a11*a2);
  a0=(a11*a0);
  a9=(a9*a3);
  a0=(a0-a9);
  a1=(a1*a6);
  a13=(a13*a2);
  a5=(a13*a5);
  a1=(a1-a5);
  a0=(a0+a1);
  if (res[0]!=0) res[0][16]=a0;
  a14=(a14*a3);
  a11=(a11*a7);
  a14=(a14-a11);
  a13=(a13*a10);
  a4=(a4*a6);
  a13=(a13-a4);
  a14=(a14+a13);
  if (res[0]!=0) res[0][17]=a14;
  a14=arg[1]? arg[1][0] : 0;
  if (res[0]!=0) res[0][18]=a14;
  a14=arg[1]? arg[1][1] : 0;
  if (res[0]!=0) res[0][19]=a14;
  a14=arg[1]? arg[1][2] : 0;
  if (res[0]!=0) res[0][20]=a14;
  a14=arg[1]? arg[1][3] : 0;
  if (res[0]!=0) res[0][21]=a14;
  a14=arg[1]? arg[1][4] : 0;
  if (res[0]!=0) res[0][22]=a14;
  a14=arg[1]? arg[1][5] : 0;
  if (res[0]!=0) res[0][23]=a14;
  return 0;
}

CASADI_SYMBOL_EXPORT int NMPC_model_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int NMPC_model_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int NMPC_model_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void NMPC_model_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int NMPC_model_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void NMPC_model_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void NMPC_model_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void NMPC_model_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int NMPC_model_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int NMPC_model_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real NMPC_model_expl_ode_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* NMPC_model_expl_ode_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* NMPC_model_expl_ode_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* NMPC_model_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* NMPC_model_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int NMPC_model_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int NMPC_model_expl_ode_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif