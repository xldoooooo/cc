/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_geo_H_
#define ACADOS_SOLVER_geo_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define GEO_NX     24
#define GEO_NZ     0
#define GEO_NU     6
#define GEO_NP     0
#define GEO_NP_GLOBAL     0
#define GEO_NBX    0
#define GEO_NBX0   0
#define GEO_NBU    0
#define GEO_NSBX   0
#define GEO_NSBU   0
#define GEO_NSH    0
#define GEO_NSH0   0
#define GEO_NSG    0
#define GEO_NSPHI  0
#define GEO_NSHN   0
#define GEO_NSGN   0
#define GEO_NSPHIN 0
#define GEO_NSPHI0 0
#define GEO_NSBXN  0
#define GEO_NS     0
#define GEO_NS0    0
#define GEO_NSN    0
#define GEO_NG     0
#define GEO_NBXN   0
#define GEO_NGN    0
#define GEO_NY0    18
#define GEO_NY     18
#define GEO_NYN    0
#define GEO_N      20
#define GEO_NH     0
#define GEO_NHN    0
#define GEO_NH0    0
#define GEO_NPHI0  0
#define GEO_NPHI   0
#define GEO_NPHIN  0
#define GEO_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct geo_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */

    // dynamics

    external_function_external_param_casadi *expl_vde_forw;
    external_function_external_param_casadi *expl_ode_fun;
    external_function_external_param_casadi *expl_vde_adj;




    // cost

    external_function_external_param_casadi *cost_y_fun;
    external_function_external_param_casadi *cost_y_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_0_fun;
    external_function_external_param_casadi cost_y_0_fun_jac_ut_xt;




    // constraints







} geo_solver_capsule;

ACADOS_SYMBOL_EXPORT geo_solver_capsule * geo_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int geo_acados_free_capsule(geo_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int geo_acados_create(geo_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int geo_acados_reset(geo_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of geo_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int geo_acados_create_with_discretization(geo_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int geo_acados_update_time_steps(geo_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int geo_acados_update_qp_solver_cond_N(geo_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int geo_acados_update_params(geo_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int geo_acados_update_params_sparse(geo_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int geo_acados_set_p_global_and_precompute_dependencies(geo_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int geo_acados_solve(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void geo_acados_batch_solve(geo_solver_capsule ** capsules, int N_batch);
ACADOS_SYMBOL_EXPORT int geo_acados_free(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void geo_acados_print_stats(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int geo_acados_custom_update(geo_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *geo_acados_get_nlp_in(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *geo_acados_get_nlp_out(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *geo_acados_get_sens_out(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *geo_acados_get_nlp_solver(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *geo_acados_get_nlp_config(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *geo_acados_get_nlp_opts(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *geo_acados_get_nlp_dims(geo_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *geo_acados_get_nlp_plan(geo_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_geo_H_
