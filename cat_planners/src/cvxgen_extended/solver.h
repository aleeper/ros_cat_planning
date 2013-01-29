/* Produced by CVXGEN, 2013-01-29 18:49:38 -0500.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double x_d[3];
  double J_v[21];
  double weight_x[3];
  double w_d[3];
  double J_w[21];
  double weight_w[3];
  double weight_q[7];
  double normal_0[3];
  double J_c_0[21];
  double retreat_0[1];
  double normal_1[3];
  double J_c_1[21];
  double retreat_1[1];
  double normal_2[3];
  double J_c_2[21];
  double retreat_2[1];
  double normal_3[3];
  double J_c_3[21];
  double retreat_3[1];
  double normal_4[3];
  double J_c_4[21];
  double retreat_4[1];
  double normal_5[3];
  double J_c_5[21];
  double retreat_5[1];
  double normal_6[3];
  double J_c_6[21];
  double retreat_6[1];
  double normal_7[3];
  double J_c_7[21];
  double retreat_7[1];
  double normal_8[3];
  double J_c_8[21];
  double retreat_8[1];
  double normal_9[3];
  double J_c_9[21];
  double retreat_9[1];
  double normal_10[3];
  double J_c_10[21];
  double retreat_10[1];
  double normal_11[3];
  double J_c_11[21];
  double retreat_11[1];
  double normal_12[3];
  double J_c_12[21];
  double retreat_12[1];
  double normal_13[3];
  double J_c_13[21];
  double retreat_13[1];
  double normal_14[3];
  double J_c_14[21];
  double retreat_14[1];
  double normal_15[3];
  double J_c_15[21];
  double retreat_15[1];
  double q_min[7];
  double has_limits[7];
  double q[7];
  double q_max[7];
  double *normal[16];
  double *J_c[16];
  double *retreat[16];
} Params;
typedef struct Vars_t {
  double *t_01; /* 3 rows. */
  double *t_02; /* 3 rows. */
  double *q_d; /* 7 rows. */
} Vars;
typedef struct Workspace_t {
  double h[30];
  double s_inv[30];
  double s_inv_z[30];
  double b[6];
  double q[13];
  double rhs[79];
  double x[79];
  double *s;
  double *z;
  double *y;
  double lhs_aff[79];
  double lhs_cc[79];
  double buffer[79];
  double buffer2[79];
  double KKT[277];
  double L[225];
  double d[79];
  double v[79];
  double d_inv[79];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long*idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

#endif
