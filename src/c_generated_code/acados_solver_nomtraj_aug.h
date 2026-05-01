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

#ifndef ACADOS_SOLVER_nomtraj_aug_H_
#define ACADOS_SOLVER_nomtraj_aug_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define NOMTRAJ_AUG_NX     17
#define NOMTRAJ_AUG_NZ     0
#define NOMTRAJ_AUG_NU     4
#define NOMTRAJ_AUG_NP     16
#define NOMTRAJ_AUG_NP_GLOBAL     0
#define NOMTRAJ_AUG_NBX    0
#define NOMTRAJ_AUG_NBX0   17
#define NOMTRAJ_AUG_NBU    4
#define NOMTRAJ_AUG_NSBX   0
#define NOMTRAJ_AUG_NSBU   0
#define NOMTRAJ_AUG_NSH    0
#define NOMTRAJ_AUG_NSH0   0
#define NOMTRAJ_AUG_NSG    0
#define NOMTRAJ_AUG_NSPHI  0
#define NOMTRAJ_AUG_NSHN   0
#define NOMTRAJ_AUG_NSGN   0
#define NOMTRAJ_AUG_NSPHIN 0
#define NOMTRAJ_AUG_NSPHI0 0
#define NOMTRAJ_AUG_NSBXN  0
#define NOMTRAJ_AUG_NS     0
#define NOMTRAJ_AUG_NS0    0
#define NOMTRAJ_AUG_NSN    0
#define NOMTRAJ_AUG_NG     0
#define NOMTRAJ_AUG_NBXN   0
#define NOMTRAJ_AUG_NGN    0
#define NOMTRAJ_AUG_NY0    25
#define NOMTRAJ_AUG_NY     25
#define NOMTRAJ_AUG_NYN    17
#define NOMTRAJ_AUG_N      2000
#define NOMTRAJ_AUG_NH     0
#define NOMTRAJ_AUG_NHN    0
#define NOMTRAJ_AUG_NH0    0
#define NOMTRAJ_AUG_NPHI0  0
#define NOMTRAJ_AUG_NPHI   0
#define NOMTRAJ_AUG_NPHIN  0
#define NOMTRAJ_AUG_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct nomtraj_aug_solver_capsule
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

    external_function_external_param_casadi *discr_dyn_phi_fun;
    external_function_external_param_casadi *discr_dyn_phi_fun_jac_ut_xt;




    // cost

    external_function_external_param_casadi *cost_y_fun;
    external_function_external_param_casadi *cost_y_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_0_fun;
    external_function_external_param_casadi cost_y_0_fun_jac_ut_xt;



    external_function_external_param_casadi cost_y_e_fun;
    external_function_external_param_casadi cost_y_e_fun_jac_ut_xt;


    // constraints







} nomtraj_aug_solver_capsule;

ACADOS_SYMBOL_EXPORT nomtraj_aug_solver_capsule * nomtraj_aug_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_free_capsule(nomtraj_aug_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_create(nomtraj_aug_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_reset(nomtraj_aug_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of nomtraj_aug_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_create_with_discretization(nomtraj_aug_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_update_time_steps(nomtraj_aug_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_update_qp_solver_cond_N(nomtraj_aug_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_update_params(nomtraj_aug_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_update_params_sparse(nomtraj_aug_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_set_p_global_and_precompute_dependencies(nomtraj_aug_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_solve(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_setup_qp_matrices_and_factorize(nomtraj_aug_solver_capsule* capsule);



ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_free(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void nomtraj_aug_acados_print_stats(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int nomtraj_aug_acados_custom_update(nomtraj_aug_solver_capsule* capsule, double* data, int data_len);

ACADOS_SYMBOL_EXPORT ocp_nlp_in *nomtraj_aug_acados_get_nlp_in(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *nomtraj_aug_acados_get_nlp_out(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *nomtraj_aug_acados_get_sens_out(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *nomtraj_aug_acados_get_nlp_solver(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *nomtraj_aug_acados_get_nlp_config(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *nomtraj_aug_acados_get_nlp_opts(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *nomtraj_aug_acados_get_nlp_dims(nomtraj_aug_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *nomtraj_aug_acados_get_nlp_plan(nomtraj_aug_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_nomtraj_aug_H_
