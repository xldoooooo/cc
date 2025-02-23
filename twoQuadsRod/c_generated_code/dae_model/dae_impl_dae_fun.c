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
  #define CASADI_PREFIX(ID) dae_impl_dae_fun_ ## ID
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
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[28] = {24, 1, 0, 24, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s3[3] = {0, 0, 0};
static const casadi_int casadi_s4[30] = {26, 1, 0, 26, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25};

/* dae_impl_dae_fun:(i0[24],i1[24],i2[6],i3[2],i4[],i5[])->(o0[26]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a31, a32, a33, a34, a35, a36, a37, a38, a39, a4, a40, a41, a42, a43, a44, a45, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][0] : 0;
  a1=arg[0]? arg[0][12] : 0;
  a0=(a0-a1);
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[1]? arg[1][1] : 0;
  a2=arg[0]? arg[0][13] : 0;
  a0=(a0-a2);
  if (res[0]!=0) res[0][1]=a0;
  a0=arg[1]? arg[1][2] : 0;
  a3=arg[0]? arg[0][14] : 0;
  a0=(a0-a3);
  if (res[0]!=0) res[0][2]=a0;
  a0=arg[1]? arg[1][3] : 0;
  a4=arg[0]? arg[0][16] : 0;
  a5=arg[0]? arg[0][5] : 0;
  a6=(a4*a5);
  a7=arg[0]? arg[0][17] : 0;
  a8=arg[0]? arg[0][4] : 0;
  a9=(a7*a8);
  a6=(a6-a9);
  a0=(a0-a6);
  if (res[0]!=0) res[0][3]=a0;
  a0=arg[1]? arg[1][4] : 0;
  a9=arg[0]? arg[0][3] : 0;
  a10=(a7*a9);
  a11=arg[0]? arg[0][15] : 0;
  a12=(a11*a5);
  a10=(a10-a12);
  a0=(a0-a10);
  if (res[0]!=0) res[0][4]=a0;
  a0=arg[1]? arg[1][5] : 0;
  a12=(a11*a8);
  a13=(a4*a9);
  a12=(a12-a13);
  a0=(a0-a12);
  if (res[0]!=0) res[0][5]=a0;
  a0=arg[1]? arg[1][6] : 0;
  a13=arg[0]? arg[0][18] : 0;
  a0=(a0-a13);
  if (res[0]!=0) res[0][6]=a0;
  a0=arg[1]? arg[1][7] : 0;
  a14=arg[0]? arg[0][19] : 0;
  a0=(a0-a14);
  if (res[0]!=0) res[0][7]=a0;
  a0=arg[1]? arg[1][8] : 0;
  a15=arg[0]? arg[0][20] : 0;
  a0=(a0-a15);
  if (res[0]!=0) res[0][8]=a0;
  a0=arg[1]? arg[1][9] : 0;
  a16=arg[0]? arg[0][21] : 0;
  a0=(a0-a16);
  if (res[0]!=0) res[0][9]=a0;
  a0=arg[1]? arg[1][10] : 0;
  a17=arg[0]? arg[0][22] : 0;
  a0=(a0-a17);
  if (res[0]!=0) res[0][10]=a0;
  a0=arg[1]? arg[1][11] : 0;
  a18=arg[0]? arg[0][23] : 0;
  a0=(a0-a18);
  if (res[0]!=0) res[0][11]=a0;
  a0=4.4500000000000001e-01;
  a19=arg[1]? arg[1][12] : 0;
  a20=(a0*a19);
  a21=arg[0]? arg[0][0] : 0;
  a22=(a21+a9);
  a23=arg[0]? arg[0][6] : 0;
  a22=(a22-a23);
  a23=arg[3]? arg[3][0] : 0;
  a24=(a22*a23);
  a20=(a20+a24);
  a21=(a21-a9);
  a25=arg[0]? arg[0][9] : 0;
  a21=(a21-a25);
  a25=arg[3]? arg[3][1] : 0;
  a26=(a21*a25);
  a20=(a20+a26);
  if (res[0]!=0) res[0][12]=a20;
  a20=arg[1]? arg[1][13] : 0;
  a27=(a0*a20);
  a28=arg[0]? arg[0][1] : 0;
  a29=(a28+a8);
  a30=arg[0]? arg[0][7] : 0;
  a29=(a29-a30);
  a30=(a29*a23);
  a27=(a27+a30);
  a28=(a28-a8);
  a31=arg[0]? arg[0][10] : 0;
  a28=(a28-a31);
  a31=(a28*a25);
  a27=(a27+a31);
  if (res[0]!=0) res[0][13]=a27;
  a27=arg[1]? arg[1][14] : 0;
  a0=(a0*a27);
  a32=arg[0]? arg[0][2] : 0;
  a33=(a32+a5);
  a34=arg[0]? arg[0][8] : 0;
  a33=(a33-a34);
  a34=(a33*a23);
  a0=(a0+a34);
  a32=(a32-a5);
  a35=arg[0]? arg[0][11] : 0;
  a32=(a32-a35);
  a35=(a32*a25);
  a0=(a0+a35);
  a36=4.3654500000000001e+00;
  a0=(a0-a36);
  if (res[0]!=0) res[0][14]=a0;
  a0=1.4999999999999999e-01;
  a36=arg[1]? arg[1][15] : 0;
  a37=(a0*a36);
  a38=(a8*a33);
  a39=(a5*a29);
  a38=(a38-a39);
  a39=(a38*a23);
  a37=(a37+a39);
  a39=(a5*a28);
  a40=(a8*a32);
  a39=(a39-a40);
  a40=(a39*a25);
  a37=(a37+a40);
  if (res[0]!=0) res[0][15]=a37;
  a37=arg[1]? arg[1][16] : 0;
  a40=(a0*a37);
  a41=(a5*a22);
  a42=(a9*a33);
  a41=(a41-a42);
  a42=(a41*a23);
  a40=(a40+a42);
  a42=(a9*a32);
  a43=(a5*a21);
  a42=(a42-a43);
  a43=(a42*a25);
  a40=(a40+a43);
  if (res[0]!=0) res[0][16]=a40;
  a40=arg[1]? arg[1][17] : 0;
  a0=(a0*a40);
  a43=(a9*a29);
  a44=(a8*a22);
  a43=(a43-a44);
  a23=(a43*a23);
  a0=(a0+a23);
  a23=(a8*a21);
  a44=(a9*a28);
  a23=(a23-a44);
  a25=(a23*a25);
  a0=(a0+a25);
  if (res[0]!=0) res[0][17]=a0;
  a0=1.2000000000000000e+00;
  a25=arg[1]? arg[1][18] : 0;
  a44=(a0*a25);
  a44=(a44-a24);
  a24=arg[2]? arg[2][0] : 0;
  a44=(a44-a24);
  if (res[0]!=0) res[0][18]=a44;
  a44=arg[1]? arg[1][19] : 0;
  a24=(a0*a44);
  a24=(a24-a30);
  a30=arg[2]? arg[2][1] : 0;
  a24=(a24-a30);
  if (res[0]!=0) res[0][19]=a24;
  a24=arg[1]? arg[1][20] : 0;
  a30=(a0*a24);
  a30=(a30-a34);
  a34=1.1772000000000000e+01;
  a45=arg[2]? arg[2][2] : 0;
  a45=(a34+a45);
  a30=(a30-a45);
  if (res[0]!=0) res[0][20]=a30;
  a30=arg[1]? arg[1][21] : 0;
  a45=(a0*a30);
  a45=(a45-a26);
  a26=arg[2]? arg[2][3] : 0;
  a45=(a45-a26);
  if (res[0]!=0) res[0][21]=a45;
  a45=arg[1]? arg[1][22] : 0;
  a26=(a0*a45);
  a26=(a26-a31);
  a31=arg[2]? arg[2][4] : 0;
  a26=(a26-a31);
  if (res[0]!=0) res[0][22]=a26;
  a26=arg[1]? arg[1][23] : 0;
  a0=(a0*a26);
  a0=(a0-a35);
  a35=arg[2]? arg[2][5] : 0;
  a34=(a34+a35);
  a0=(a0-a34);
  if (res[0]!=0) res[0][23]=a0;
  a0=(a22*a19);
  a34=(a29*a20);
  a0=(a0+a34);
  a34=(a33*a27);
  a0=(a0+a34);
  a38=(a38*a36);
  a0=(a0+a38);
  a41=(a41*a37);
  a0=(a0+a41);
  a43=(a43*a40);
  a0=(a0+a43);
  a25=(a22*a25);
  a0=(a0-a25);
  a44=(a29*a44);
  a0=(a0-a44);
  a24=(a33*a24);
  a0=(a0-a24);
  a11=casadi_sq(a11);
  a4=casadi_sq(a4);
  a11=(a11+a4);
  a7=casadi_sq(a7);
  a11=(a11+a7);
  a7=(a22*a9);
  a4=(a29*a8);
  a7=(a7+a4);
  a4=(a33*a5);
  a7=(a7+a4);
  a7=(a11*a7);
  a4=(a1+a6);
  a4=(a4-a13);
  a24=(a4*a1);
  a44=(a2+a10);
  a44=(a44-a14);
  a25=(a44*a2);
  a24=(a24+a25);
  a25=(a3+a12);
  a25=(a25-a15);
  a43=(a25*a3);
  a24=(a24+a43);
  a7=(a7-a24);
  a24=(a4*a6);
  a43=(a44*a10);
  a24=(a24+a43);
  a43=(a25*a12);
  a24=(a24+a43);
  a7=(a7-a24);
  a4=(a4*a13);
  a44=(a44*a14);
  a4=(a4+a44);
  a25=(a25*a15);
  a4=(a4+a25);
  a7=(a7+a4);
  a4=25.;
  a25=5.0000000000000000e-01;
  a44=casadi_sq(a22);
  a24=casadi_sq(a29);
  a44=(a44+a24);
  a24=casadi_sq(a33);
  a44=(a44+a24);
  a24=1.;
  a44=(a44-a24);
  a44=(a25*a44);
  a44=(a4*a44);
  a7=(a7-a44);
  a44=10.;
  a43=(a22*a1);
  a41=(a29*a2);
  a43=(a43+a41);
  a41=(a33*a3);
  a43=(a43+a41);
  a41=(a22*a6);
  a38=(a29*a10);
  a41=(a41+a38);
  a38=(a33*a12);
  a41=(a41+a38);
  a43=(a43+a41);
  a22=(a22*a13);
  a29=(a29*a14);
  a22=(a22+a29);
  a33=(a33*a15);
  a22=(a22+a33);
  a43=(a43-a22);
  a43=(a44*a43);
  a7=(a7-a43);
  a0=(a0-a7);
  if (res[0]!=0) res[0][24]=a0;
  a19=(a21*a19);
  a20=(a28*a20);
  a19=(a19+a20);
  a27=(a32*a27);
  a19=(a19+a27);
  a39=(a39*a36);
  a19=(a19+a39);
  a42=(a42*a37);
  a19=(a19+a42);
  a23=(a23*a40);
  a19=(a19+a23);
  a30=(a21*a30);
  a19=(a19-a30);
  a45=(a28*a45);
  a19=(a19-a45);
  a26=(a32*a26);
  a19=(a19-a26);
  a26=(a1-a6);
  a26=(a26-a16);
  a45=(a26*a6);
  a30=(a2-a10);
  a30=(a30-a17);
  a23=(a30*a10);
  a45=(a45+a23);
  a23=(a3-a12);
  a23=(a23-a18);
  a40=(a23*a12);
  a45=(a45+a40);
  a40=(a26*a1);
  a42=(a30*a2);
  a40=(a40+a42);
  a42=(a23*a3);
  a40=(a40+a42);
  a9=(a21*a9);
  a8=(a28*a8);
  a9=(a9+a8);
  a5=(a32*a5);
  a9=(a9+a5);
  a11=(a11*a9);
  a40=(a40+a11);
  a45=(a45-a40);
  a26=(a26*a16);
  a30=(a30*a17);
  a26=(a26+a30);
  a23=(a23*a18);
  a26=(a26+a23);
  a45=(a45+a26);
  a26=casadi_sq(a21);
  a23=casadi_sq(a28);
  a26=(a26+a23);
  a23=casadi_sq(a32);
  a26=(a26+a23);
  a26=(a26-a24);
  a25=(a25*a26);
  a4=(a4*a25);
  a45=(a45-a4);
  a1=(a21*a1);
  a2=(a28*a2);
  a1=(a1+a2);
  a3=(a32*a3);
  a1=(a1+a3);
  a6=(a21*a6);
  a10=(a28*a10);
  a6=(a6+a10);
  a12=(a32*a12);
  a6=(a6+a12);
  a1=(a1-a6);
  a21=(a21*a16);
  a28=(a28*a17);
  a21=(a21+a28);
  a32=(a32*a18);
  a21=(a21+a32);
  a1=(a1-a21);
  a44=(a44*a1);
  a45=(a45-a44);
  a19=(a19-a45);
  if (res[0]!=0) res[0][25]=a19;
  return 0;
}

CASADI_SYMBOL_EXPORT int dae_impl_dae_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int dae_impl_dae_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int dae_impl_dae_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dae_impl_dae_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int dae_impl_dae_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void dae_impl_dae_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void dae_impl_dae_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void dae_impl_dae_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int dae_impl_dae_fun_n_in(void) { return 6;}

CASADI_SYMBOL_EXPORT casadi_int dae_impl_dae_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real dae_impl_dae_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* dae_impl_dae_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    case 5: return "i5";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* dae_impl_dae_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* dae_impl_dae_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    case 4: return casadi_s3;
    case 5: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* dae_impl_dae_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int dae_impl_dae_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

CASADI_SYMBOL_EXPORT int dae_impl_dae_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 1*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 0*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
