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

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific

#include "nomtraj_aug_model/nomtraj_aug_model.h"


#include "nomtraj_aug_cost/nomtraj_aug_cost.h"



#include "acados_solver_nomtraj_aug.h"

#define NX     NOMTRAJ_AUG_NX
#define NZ     NOMTRAJ_AUG_NZ
#define NU     NOMTRAJ_AUG_NU
#define NP     NOMTRAJ_AUG_NP
#define NP_GLOBAL     NOMTRAJ_AUG_NP_GLOBAL
#define NY0    NOMTRAJ_AUG_NY0
#define NY     NOMTRAJ_AUG_NY
#define NYN    NOMTRAJ_AUG_NYN

#define NBX    NOMTRAJ_AUG_NBX
#define NBX0   NOMTRAJ_AUG_NBX0
#define NBU    NOMTRAJ_AUG_NBU
#define NG     NOMTRAJ_AUG_NG
#define NBXN   NOMTRAJ_AUG_NBXN
#define NGN    NOMTRAJ_AUG_NGN

#define NH     NOMTRAJ_AUG_NH
#define NHN    NOMTRAJ_AUG_NHN
#define NH0    NOMTRAJ_AUG_NH0
#define NPHI   NOMTRAJ_AUG_NPHI
#define NPHIN  NOMTRAJ_AUG_NPHIN
#define NPHI0  NOMTRAJ_AUG_NPHI0
#define NR     NOMTRAJ_AUG_NR

#define NS     NOMTRAJ_AUG_NS
#define NS0    NOMTRAJ_AUG_NS0
#define NSN    NOMTRAJ_AUG_NSN

#define NSBX   NOMTRAJ_AUG_NSBX
#define NSBU   NOMTRAJ_AUG_NSBU
#define NSH0   NOMTRAJ_AUG_NSH0
#define NSH    NOMTRAJ_AUG_NSH
#define NSHN   NOMTRAJ_AUG_NSHN
#define NSG    NOMTRAJ_AUG_NSG
#define NSPHI0 NOMTRAJ_AUG_NSPHI0
#define NSPHI  NOMTRAJ_AUG_NSPHI
#define NSPHIN NOMTRAJ_AUG_NSPHIN
#define NSGN   NOMTRAJ_AUG_NSGN
#define NSBXN  NOMTRAJ_AUG_NSBXN



// ** solver data **

nomtraj_aug_solver_capsule * nomtraj_aug_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(nomtraj_aug_solver_capsule));
    nomtraj_aug_solver_capsule *capsule = (nomtraj_aug_solver_capsule *) capsule_mem;

    return capsule;
}


int nomtraj_aug_acados_free_capsule(nomtraj_aug_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int nomtraj_aug_acados_create(nomtraj_aug_solver_capsule* capsule)
{
    int N_shooting_intervals = NOMTRAJ_AUG_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return nomtraj_aug_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int nomtraj_aug_acados_update_time_steps(nomtraj_aug_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "nomtraj_aug_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;

}

/**
 * Internal function for nomtraj_aug_acados_create: step 1
 */
void nomtraj_aug_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = NONLINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = NONLINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = NONLINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;

    nlp_solver_plan->globalization = FIXED_STEP;
}


static ocp_nlp_dims* nomtraj_aug_acados_create_setup_dimensions(nomtraj_aug_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    
    nbxe[0] = 0;
    
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);
    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for nomtraj_aug_acados_create: step 3
 */
void nomtraj_aug_acados_create_setup_functions(nomtraj_aug_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    if (N > 0)
    {
        // nonlinear least squares function
        MAP_CASADI_FNC(cost_y_0_fun, nomtraj_aug_cost_y_0_fun);
        MAP_CASADI_FNC(cost_y_0_fun_jac_ut_xt, nomtraj_aug_cost_y_0_fun_jac_ut_xt);



    
        // discrete dynamics
        capsule->discr_dyn_phi_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun[i], nomtraj_aug_dyn_disc_phi_fun);
        }

        capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt[i], nomtraj_aug_dyn_disc_phi_fun_jac);
        }

    

    
        // nonlinear least squares cost
        capsule->cost_y_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun[i], nomtraj_aug_cost_y_fun);
        }

        capsule->cost_y_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun_jac_ut_xt[i], nomtraj_aug_cost_y_fun_jac_ut_xt);
        }
    } // N > 0
    // nonlinear least square function
    MAP_CASADI_FNC(cost_y_e_fun, nomtraj_aug_cost_y_e_fun);
    MAP_CASADI_FNC(cost_y_e_fun_jac_ut_xt, nomtraj_aug_cost_y_e_fun_jac_ut_xt);

#undef MAP_CASADI_FNC
}


/**
 * Internal function for nomtraj_aug_acados_create: step 5
 */
void nomtraj_aug_acados_create_set_default_parameters(nomtraj_aug_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        nomtraj_aug_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for nomtraj_aug_acados_create: step 5
 */
void nomtraj_aug_acados_setup_nlp_in(nomtraj_aug_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        nomtraj_aug_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    
        double time_step = 0.01;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.01;
        cost_scaling[1] = 0.01;
        cost_scaling[2] = 0.01;
        cost_scaling[3] = 0.01;
        cost_scaling[4] = 0.01;
        cost_scaling[5] = 0.01;
        cost_scaling[6] = 0.01;
        cost_scaling[7] = 0.01;
        cost_scaling[8] = 0.01;
        cost_scaling[9] = 0.01;
        cost_scaling[10] = 0.01;
        cost_scaling[11] = 0.01;
        cost_scaling[12] = 0.01;
        cost_scaling[13] = 0.01;
        cost_scaling[14] = 0.01;
        cost_scaling[15] = 0.01;
        cost_scaling[16] = 0.01;
        cost_scaling[17] = 0.01;
        cost_scaling[18] = 0.01;
        cost_scaling[19] = 0.01;
        cost_scaling[20] = 0.01;
        cost_scaling[21] = 0.01;
        cost_scaling[22] = 0.01;
        cost_scaling[23] = 0.01;
        cost_scaling[24] = 0.01;
        cost_scaling[25] = 0.01;
        cost_scaling[26] = 0.01;
        cost_scaling[27] = 0.01;
        cost_scaling[28] = 0.01;
        cost_scaling[29] = 0.01;
        cost_scaling[30] = 0.01;
        cost_scaling[31] = 0.01;
        cost_scaling[32] = 0.01;
        cost_scaling[33] = 0.01;
        cost_scaling[34] = 0.01;
        cost_scaling[35] = 0.01;
        cost_scaling[36] = 0.01;
        cost_scaling[37] = 0.01;
        cost_scaling[38] = 0.01;
        cost_scaling[39] = 0.01;
        cost_scaling[40] = 0.01;
        cost_scaling[41] = 0.01;
        cost_scaling[42] = 0.01;
        cost_scaling[43] = 0.01;
        cost_scaling[44] = 0.01;
        cost_scaling[45] = 0.01;
        cost_scaling[46] = 0.01;
        cost_scaling[47] = 0.01;
        cost_scaling[48] = 0.01;
        cost_scaling[49] = 0.01;
        cost_scaling[50] = 0.01;
        cost_scaling[51] = 0.01;
        cost_scaling[52] = 0.01;
        cost_scaling[53] = 0.01;
        cost_scaling[54] = 0.01;
        cost_scaling[55] = 0.01;
        cost_scaling[56] = 0.01;
        cost_scaling[57] = 0.01;
        cost_scaling[58] = 0.01;
        cost_scaling[59] = 0.01;
        cost_scaling[60] = 0.01;
        cost_scaling[61] = 0.01;
        cost_scaling[62] = 0.01;
        cost_scaling[63] = 0.01;
        cost_scaling[64] = 0.01;
        cost_scaling[65] = 0.01;
        cost_scaling[66] = 0.01;
        cost_scaling[67] = 0.01;
        cost_scaling[68] = 0.01;
        cost_scaling[69] = 0.01;
        cost_scaling[70] = 0.01;
        cost_scaling[71] = 0.01;
        cost_scaling[72] = 0.01;
        cost_scaling[73] = 0.01;
        cost_scaling[74] = 0.01;
        cost_scaling[75] = 0.01;
        cost_scaling[76] = 0.01;
        cost_scaling[77] = 0.01;
        cost_scaling[78] = 0.01;
        cost_scaling[79] = 0.01;
        cost_scaling[80] = 0.01;
        cost_scaling[81] = 0.01;
        cost_scaling[82] = 0.01;
        cost_scaling[83] = 0.01;
        cost_scaling[84] = 0.01;
        cost_scaling[85] = 0.01;
        cost_scaling[86] = 0.01;
        cost_scaling[87] = 0.01;
        cost_scaling[88] = 0.01;
        cost_scaling[89] = 0.01;
        cost_scaling[90] = 0.01;
        cost_scaling[91] = 0.01;
        cost_scaling[92] = 0.01;
        cost_scaling[93] = 0.01;
        cost_scaling[94] = 0.01;
        cost_scaling[95] = 0.01;
        cost_scaling[96] = 0.01;
        cost_scaling[97] = 0.01;
        cost_scaling[98] = 0.01;
        cost_scaling[99] = 0.01;
        cost_scaling[100] = 0.01;
        cost_scaling[101] = 0.01;
        cost_scaling[102] = 0.01;
        cost_scaling[103] = 0.01;
        cost_scaling[104] = 0.01;
        cost_scaling[105] = 0.01;
        cost_scaling[106] = 0.01;
        cost_scaling[107] = 0.01;
        cost_scaling[108] = 0.01;
        cost_scaling[109] = 0.01;
        cost_scaling[110] = 0.01;
        cost_scaling[111] = 0.01;
        cost_scaling[112] = 0.01;
        cost_scaling[113] = 0.01;
        cost_scaling[114] = 0.01;
        cost_scaling[115] = 0.01;
        cost_scaling[116] = 0.01;
        cost_scaling[117] = 0.01;
        cost_scaling[118] = 0.01;
        cost_scaling[119] = 0.01;
        cost_scaling[120] = 0.01;
        cost_scaling[121] = 0.01;
        cost_scaling[122] = 0.01;
        cost_scaling[123] = 0.01;
        cost_scaling[124] = 0.01;
        cost_scaling[125] = 0.01;
        cost_scaling[126] = 0.01;
        cost_scaling[127] = 0.01;
        cost_scaling[128] = 0.01;
        cost_scaling[129] = 0.01;
        cost_scaling[130] = 0.01;
        cost_scaling[131] = 0.01;
        cost_scaling[132] = 0.01;
        cost_scaling[133] = 0.01;
        cost_scaling[134] = 0.01;
        cost_scaling[135] = 0.01;
        cost_scaling[136] = 0.01;
        cost_scaling[137] = 0.01;
        cost_scaling[138] = 0.01;
        cost_scaling[139] = 0.01;
        cost_scaling[140] = 0.01;
        cost_scaling[141] = 0.01;
        cost_scaling[142] = 0.01;
        cost_scaling[143] = 0.01;
        cost_scaling[144] = 0.01;
        cost_scaling[145] = 0.01;
        cost_scaling[146] = 0.01;
        cost_scaling[147] = 0.01;
        cost_scaling[148] = 0.01;
        cost_scaling[149] = 0.01;
        cost_scaling[150] = 0.01;
        cost_scaling[151] = 0.01;
        cost_scaling[152] = 0.01;
        cost_scaling[153] = 0.01;
        cost_scaling[154] = 0.01;
        cost_scaling[155] = 0.01;
        cost_scaling[156] = 0.01;
        cost_scaling[157] = 0.01;
        cost_scaling[158] = 0.01;
        cost_scaling[159] = 0.01;
        cost_scaling[160] = 0.01;
        cost_scaling[161] = 0.01;
        cost_scaling[162] = 0.01;
        cost_scaling[163] = 0.01;
        cost_scaling[164] = 0.01;
        cost_scaling[165] = 0.01;
        cost_scaling[166] = 0.01;
        cost_scaling[167] = 0.01;
        cost_scaling[168] = 0.01;
        cost_scaling[169] = 0.01;
        cost_scaling[170] = 0.01;
        cost_scaling[171] = 0.01;
        cost_scaling[172] = 0.01;
        cost_scaling[173] = 0.01;
        cost_scaling[174] = 0.01;
        cost_scaling[175] = 0.01;
        cost_scaling[176] = 0.01;
        cost_scaling[177] = 0.01;
        cost_scaling[178] = 0.01;
        cost_scaling[179] = 0.01;
        cost_scaling[180] = 0.01;
        cost_scaling[181] = 0.01;
        cost_scaling[182] = 0.01;
        cost_scaling[183] = 0.01;
        cost_scaling[184] = 0.01;
        cost_scaling[185] = 0.01;
        cost_scaling[186] = 0.01;
        cost_scaling[187] = 0.01;
        cost_scaling[188] = 0.01;
        cost_scaling[189] = 0.01;
        cost_scaling[190] = 0.01;
        cost_scaling[191] = 0.01;
        cost_scaling[192] = 0.01;
        cost_scaling[193] = 0.01;
        cost_scaling[194] = 0.01;
        cost_scaling[195] = 0.01;
        cost_scaling[196] = 0.01;
        cost_scaling[197] = 0.01;
        cost_scaling[198] = 0.01;
        cost_scaling[199] = 0.01;
        cost_scaling[200] = 0.01;
        cost_scaling[201] = 0.01;
        cost_scaling[202] = 0.01;
        cost_scaling[203] = 0.01;
        cost_scaling[204] = 0.01;
        cost_scaling[205] = 0.01;
        cost_scaling[206] = 0.01;
        cost_scaling[207] = 0.01;
        cost_scaling[208] = 0.01;
        cost_scaling[209] = 0.01;
        cost_scaling[210] = 0.01;
        cost_scaling[211] = 0.01;
        cost_scaling[212] = 0.01;
        cost_scaling[213] = 0.01;
        cost_scaling[214] = 0.01;
        cost_scaling[215] = 0.01;
        cost_scaling[216] = 0.01;
        cost_scaling[217] = 0.01;
        cost_scaling[218] = 0.01;
        cost_scaling[219] = 0.01;
        cost_scaling[220] = 0.01;
        cost_scaling[221] = 0.01;
        cost_scaling[222] = 0.01;
        cost_scaling[223] = 0.01;
        cost_scaling[224] = 0.01;
        cost_scaling[225] = 0.01;
        cost_scaling[226] = 0.01;
        cost_scaling[227] = 0.01;
        cost_scaling[228] = 0.01;
        cost_scaling[229] = 0.01;
        cost_scaling[230] = 0.01;
        cost_scaling[231] = 0.01;
        cost_scaling[232] = 0.01;
        cost_scaling[233] = 0.01;
        cost_scaling[234] = 0.01;
        cost_scaling[235] = 0.01;
        cost_scaling[236] = 0.01;
        cost_scaling[237] = 0.01;
        cost_scaling[238] = 0.01;
        cost_scaling[239] = 0.01;
        cost_scaling[240] = 0.01;
        cost_scaling[241] = 0.01;
        cost_scaling[242] = 0.01;
        cost_scaling[243] = 0.01;
        cost_scaling[244] = 0.01;
        cost_scaling[245] = 0.01;
        cost_scaling[246] = 0.01;
        cost_scaling[247] = 0.01;
        cost_scaling[248] = 0.01;
        cost_scaling[249] = 0.01;
        cost_scaling[250] = 0.01;
        cost_scaling[251] = 0.01;
        cost_scaling[252] = 0.01;
        cost_scaling[253] = 0.01;
        cost_scaling[254] = 0.01;
        cost_scaling[255] = 0.01;
        cost_scaling[256] = 0.01;
        cost_scaling[257] = 0.01;
        cost_scaling[258] = 0.01;
        cost_scaling[259] = 0.01;
        cost_scaling[260] = 0.01;
        cost_scaling[261] = 0.01;
        cost_scaling[262] = 0.01;
        cost_scaling[263] = 0.01;
        cost_scaling[264] = 0.01;
        cost_scaling[265] = 0.01;
        cost_scaling[266] = 0.01;
        cost_scaling[267] = 0.01;
        cost_scaling[268] = 0.01;
        cost_scaling[269] = 0.01;
        cost_scaling[270] = 0.01;
        cost_scaling[271] = 0.01;
        cost_scaling[272] = 0.01;
        cost_scaling[273] = 0.01;
        cost_scaling[274] = 0.01;
        cost_scaling[275] = 0.01;
        cost_scaling[276] = 0.01;
        cost_scaling[277] = 0.01;
        cost_scaling[278] = 0.01;
        cost_scaling[279] = 0.01;
        cost_scaling[280] = 0.01;
        cost_scaling[281] = 0.01;
        cost_scaling[282] = 0.01;
        cost_scaling[283] = 0.01;
        cost_scaling[284] = 0.01;
        cost_scaling[285] = 0.01;
        cost_scaling[286] = 0.01;
        cost_scaling[287] = 0.01;
        cost_scaling[288] = 0.01;
        cost_scaling[289] = 0.01;
        cost_scaling[290] = 0.01;
        cost_scaling[291] = 0.01;
        cost_scaling[292] = 0.01;
        cost_scaling[293] = 0.01;
        cost_scaling[294] = 0.01;
        cost_scaling[295] = 0.01;
        cost_scaling[296] = 0.01;
        cost_scaling[297] = 0.01;
        cost_scaling[298] = 0.01;
        cost_scaling[299] = 0.01;
        cost_scaling[300] = 0.01;
        cost_scaling[301] = 0.01;
        cost_scaling[302] = 0.01;
        cost_scaling[303] = 0.01;
        cost_scaling[304] = 0.01;
        cost_scaling[305] = 0.01;
        cost_scaling[306] = 0.01;
        cost_scaling[307] = 0.01;
        cost_scaling[308] = 0.01;
        cost_scaling[309] = 0.01;
        cost_scaling[310] = 0.01;
        cost_scaling[311] = 0.01;
        cost_scaling[312] = 0.01;
        cost_scaling[313] = 0.01;
        cost_scaling[314] = 0.01;
        cost_scaling[315] = 0.01;
        cost_scaling[316] = 0.01;
        cost_scaling[317] = 0.01;
        cost_scaling[318] = 0.01;
        cost_scaling[319] = 0.01;
        cost_scaling[320] = 0.01;
        cost_scaling[321] = 0.01;
        cost_scaling[322] = 0.01;
        cost_scaling[323] = 0.01;
        cost_scaling[324] = 0.01;
        cost_scaling[325] = 0.01;
        cost_scaling[326] = 0.01;
        cost_scaling[327] = 0.01;
        cost_scaling[328] = 0.01;
        cost_scaling[329] = 0.01;
        cost_scaling[330] = 0.01;
        cost_scaling[331] = 0.01;
        cost_scaling[332] = 0.01;
        cost_scaling[333] = 0.01;
        cost_scaling[334] = 0.01;
        cost_scaling[335] = 0.01;
        cost_scaling[336] = 0.01;
        cost_scaling[337] = 0.01;
        cost_scaling[338] = 0.01;
        cost_scaling[339] = 0.01;
        cost_scaling[340] = 0.01;
        cost_scaling[341] = 0.01;
        cost_scaling[342] = 0.01;
        cost_scaling[343] = 0.01;
        cost_scaling[344] = 0.01;
        cost_scaling[345] = 0.01;
        cost_scaling[346] = 0.01;
        cost_scaling[347] = 0.01;
        cost_scaling[348] = 0.01;
        cost_scaling[349] = 0.01;
        cost_scaling[350] = 0.01;
        cost_scaling[351] = 0.01;
        cost_scaling[352] = 0.01;
        cost_scaling[353] = 0.01;
        cost_scaling[354] = 0.01;
        cost_scaling[355] = 0.01;
        cost_scaling[356] = 0.01;
        cost_scaling[357] = 0.01;
        cost_scaling[358] = 0.01;
        cost_scaling[359] = 0.01;
        cost_scaling[360] = 0.01;
        cost_scaling[361] = 0.01;
        cost_scaling[362] = 0.01;
        cost_scaling[363] = 0.01;
        cost_scaling[364] = 0.01;
        cost_scaling[365] = 0.01;
        cost_scaling[366] = 0.01;
        cost_scaling[367] = 0.01;
        cost_scaling[368] = 0.01;
        cost_scaling[369] = 0.01;
        cost_scaling[370] = 0.01;
        cost_scaling[371] = 0.01;
        cost_scaling[372] = 0.01;
        cost_scaling[373] = 0.01;
        cost_scaling[374] = 0.01;
        cost_scaling[375] = 0.01;
        cost_scaling[376] = 0.01;
        cost_scaling[377] = 0.01;
        cost_scaling[378] = 0.01;
        cost_scaling[379] = 0.01;
        cost_scaling[380] = 0.01;
        cost_scaling[381] = 0.01;
        cost_scaling[382] = 0.01;
        cost_scaling[383] = 0.01;
        cost_scaling[384] = 0.01;
        cost_scaling[385] = 0.01;
        cost_scaling[386] = 0.01;
        cost_scaling[387] = 0.01;
        cost_scaling[388] = 0.01;
        cost_scaling[389] = 0.01;
        cost_scaling[390] = 0.01;
        cost_scaling[391] = 0.01;
        cost_scaling[392] = 0.01;
        cost_scaling[393] = 0.01;
        cost_scaling[394] = 0.01;
        cost_scaling[395] = 0.01;
        cost_scaling[396] = 0.01;
        cost_scaling[397] = 0.01;
        cost_scaling[398] = 0.01;
        cost_scaling[399] = 0.01;
        cost_scaling[400] = 0.01;
        cost_scaling[401] = 0.01;
        cost_scaling[402] = 0.01;
        cost_scaling[403] = 0.01;
        cost_scaling[404] = 0.01;
        cost_scaling[405] = 0.01;
        cost_scaling[406] = 0.01;
        cost_scaling[407] = 0.01;
        cost_scaling[408] = 0.01;
        cost_scaling[409] = 0.01;
        cost_scaling[410] = 0.01;
        cost_scaling[411] = 0.01;
        cost_scaling[412] = 0.01;
        cost_scaling[413] = 0.01;
        cost_scaling[414] = 0.01;
        cost_scaling[415] = 0.01;
        cost_scaling[416] = 0.01;
        cost_scaling[417] = 0.01;
        cost_scaling[418] = 0.01;
        cost_scaling[419] = 0.01;
        cost_scaling[420] = 0.01;
        cost_scaling[421] = 0.01;
        cost_scaling[422] = 0.01;
        cost_scaling[423] = 0.01;
        cost_scaling[424] = 0.01;
        cost_scaling[425] = 0.01;
        cost_scaling[426] = 0.01;
        cost_scaling[427] = 0.01;
        cost_scaling[428] = 0.01;
        cost_scaling[429] = 0.01;
        cost_scaling[430] = 0.01;
        cost_scaling[431] = 0.01;
        cost_scaling[432] = 0.01;
        cost_scaling[433] = 0.01;
        cost_scaling[434] = 0.01;
        cost_scaling[435] = 0.01;
        cost_scaling[436] = 0.01;
        cost_scaling[437] = 0.01;
        cost_scaling[438] = 0.01;
        cost_scaling[439] = 0.01;
        cost_scaling[440] = 0.01;
        cost_scaling[441] = 0.01;
        cost_scaling[442] = 0.01;
        cost_scaling[443] = 0.01;
        cost_scaling[444] = 0.01;
        cost_scaling[445] = 0.01;
        cost_scaling[446] = 0.01;
        cost_scaling[447] = 0.01;
        cost_scaling[448] = 0.01;
        cost_scaling[449] = 0.01;
        cost_scaling[450] = 0.01;
        cost_scaling[451] = 0.01;
        cost_scaling[452] = 0.01;
        cost_scaling[453] = 0.01;
        cost_scaling[454] = 0.01;
        cost_scaling[455] = 0.01;
        cost_scaling[456] = 0.01;
        cost_scaling[457] = 0.01;
        cost_scaling[458] = 0.01;
        cost_scaling[459] = 0.01;
        cost_scaling[460] = 0.01;
        cost_scaling[461] = 0.01;
        cost_scaling[462] = 0.01;
        cost_scaling[463] = 0.01;
        cost_scaling[464] = 0.01;
        cost_scaling[465] = 0.01;
        cost_scaling[466] = 0.01;
        cost_scaling[467] = 0.01;
        cost_scaling[468] = 0.01;
        cost_scaling[469] = 0.01;
        cost_scaling[470] = 0.01;
        cost_scaling[471] = 0.01;
        cost_scaling[472] = 0.01;
        cost_scaling[473] = 0.01;
        cost_scaling[474] = 0.01;
        cost_scaling[475] = 0.01;
        cost_scaling[476] = 0.01;
        cost_scaling[477] = 0.01;
        cost_scaling[478] = 0.01;
        cost_scaling[479] = 0.01;
        cost_scaling[480] = 0.01;
        cost_scaling[481] = 0.01;
        cost_scaling[482] = 0.01;
        cost_scaling[483] = 0.01;
        cost_scaling[484] = 0.01;
        cost_scaling[485] = 0.01;
        cost_scaling[486] = 0.01;
        cost_scaling[487] = 0.01;
        cost_scaling[488] = 0.01;
        cost_scaling[489] = 0.01;
        cost_scaling[490] = 0.01;
        cost_scaling[491] = 0.01;
        cost_scaling[492] = 0.01;
        cost_scaling[493] = 0.01;
        cost_scaling[494] = 0.01;
        cost_scaling[495] = 0.01;
        cost_scaling[496] = 0.01;
        cost_scaling[497] = 0.01;
        cost_scaling[498] = 0.01;
        cost_scaling[499] = 0.01;
        cost_scaling[500] = 0.01;
        cost_scaling[501] = 0.01;
        cost_scaling[502] = 0.01;
        cost_scaling[503] = 0.01;
        cost_scaling[504] = 0.01;
        cost_scaling[505] = 0.01;
        cost_scaling[506] = 0.01;
        cost_scaling[507] = 0.01;
        cost_scaling[508] = 0.01;
        cost_scaling[509] = 0.01;
        cost_scaling[510] = 0.01;
        cost_scaling[511] = 0.01;
        cost_scaling[512] = 0.01;
        cost_scaling[513] = 0.01;
        cost_scaling[514] = 0.01;
        cost_scaling[515] = 0.01;
        cost_scaling[516] = 0.01;
        cost_scaling[517] = 0.01;
        cost_scaling[518] = 0.01;
        cost_scaling[519] = 0.01;
        cost_scaling[520] = 0.01;
        cost_scaling[521] = 0.01;
        cost_scaling[522] = 0.01;
        cost_scaling[523] = 0.01;
        cost_scaling[524] = 0.01;
        cost_scaling[525] = 0.01;
        cost_scaling[526] = 0.01;
        cost_scaling[527] = 0.01;
        cost_scaling[528] = 0.01;
        cost_scaling[529] = 0.01;
        cost_scaling[530] = 0.01;
        cost_scaling[531] = 0.01;
        cost_scaling[532] = 0.01;
        cost_scaling[533] = 0.01;
        cost_scaling[534] = 0.01;
        cost_scaling[535] = 0.01;
        cost_scaling[536] = 0.01;
        cost_scaling[537] = 0.01;
        cost_scaling[538] = 0.01;
        cost_scaling[539] = 0.01;
        cost_scaling[540] = 0.01;
        cost_scaling[541] = 0.01;
        cost_scaling[542] = 0.01;
        cost_scaling[543] = 0.01;
        cost_scaling[544] = 0.01;
        cost_scaling[545] = 0.01;
        cost_scaling[546] = 0.01;
        cost_scaling[547] = 0.01;
        cost_scaling[548] = 0.01;
        cost_scaling[549] = 0.01;
        cost_scaling[550] = 0.01;
        cost_scaling[551] = 0.01;
        cost_scaling[552] = 0.01;
        cost_scaling[553] = 0.01;
        cost_scaling[554] = 0.01;
        cost_scaling[555] = 0.01;
        cost_scaling[556] = 0.01;
        cost_scaling[557] = 0.01;
        cost_scaling[558] = 0.01;
        cost_scaling[559] = 0.01;
        cost_scaling[560] = 0.01;
        cost_scaling[561] = 0.01;
        cost_scaling[562] = 0.01;
        cost_scaling[563] = 0.01;
        cost_scaling[564] = 0.01;
        cost_scaling[565] = 0.01;
        cost_scaling[566] = 0.01;
        cost_scaling[567] = 0.01;
        cost_scaling[568] = 0.01;
        cost_scaling[569] = 0.01;
        cost_scaling[570] = 0.01;
        cost_scaling[571] = 0.01;
        cost_scaling[572] = 0.01;
        cost_scaling[573] = 0.01;
        cost_scaling[574] = 0.01;
        cost_scaling[575] = 0.01;
        cost_scaling[576] = 0.01;
        cost_scaling[577] = 0.01;
        cost_scaling[578] = 0.01;
        cost_scaling[579] = 0.01;
        cost_scaling[580] = 0.01;
        cost_scaling[581] = 0.01;
        cost_scaling[582] = 0.01;
        cost_scaling[583] = 0.01;
        cost_scaling[584] = 0.01;
        cost_scaling[585] = 0.01;
        cost_scaling[586] = 0.01;
        cost_scaling[587] = 0.01;
        cost_scaling[588] = 0.01;
        cost_scaling[589] = 0.01;
        cost_scaling[590] = 0.01;
        cost_scaling[591] = 0.01;
        cost_scaling[592] = 0.01;
        cost_scaling[593] = 0.01;
        cost_scaling[594] = 0.01;
        cost_scaling[595] = 0.01;
        cost_scaling[596] = 0.01;
        cost_scaling[597] = 0.01;
        cost_scaling[598] = 0.01;
        cost_scaling[599] = 0.01;
        cost_scaling[600] = 0.01;
        cost_scaling[601] = 0.01;
        cost_scaling[602] = 0.01;
        cost_scaling[603] = 0.01;
        cost_scaling[604] = 0.01;
        cost_scaling[605] = 0.01;
        cost_scaling[606] = 0.01;
        cost_scaling[607] = 0.01;
        cost_scaling[608] = 0.01;
        cost_scaling[609] = 0.01;
        cost_scaling[610] = 0.01;
        cost_scaling[611] = 0.01;
        cost_scaling[612] = 0.01;
        cost_scaling[613] = 0.01;
        cost_scaling[614] = 0.01;
        cost_scaling[615] = 0.01;
        cost_scaling[616] = 0.01;
        cost_scaling[617] = 0.01;
        cost_scaling[618] = 0.01;
        cost_scaling[619] = 0.01;
        cost_scaling[620] = 0.01;
        cost_scaling[621] = 0.01;
        cost_scaling[622] = 0.01;
        cost_scaling[623] = 0.01;
        cost_scaling[624] = 0.01;
        cost_scaling[625] = 0.01;
        cost_scaling[626] = 0.01;
        cost_scaling[627] = 0.01;
        cost_scaling[628] = 0.01;
        cost_scaling[629] = 0.01;
        cost_scaling[630] = 0.01;
        cost_scaling[631] = 0.01;
        cost_scaling[632] = 0.01;
        cost_scaling[633] = 0.01;
        cost_scaling[634] = 0.01;
        cost_scaling[635] = 0.01;
        cost_scaling[636] = 0.01;
        cost_scaling[637] = 0.01;
        cost_scaling[638] = 0.01;
        cost_scaling[639] = 0.01;
        cost_scaling[640] = 0.01;
        cost_scaling[641] = 0.01;
        cost_scaling[642] = 0.01;
        cost_scaling[643] = 0.01;
        cost_scaling[644] = 0.01;
        cost_scaling[645] = 0.01;
        cost_scaling[646] = 0.01;
        cost_scaling[647] = 0.01;
        cost_scaling[648] = 0.01;
        cost_scaling[649] = 0.01;
        cost_scaling[650] = 0.01;
        cost_scaling[651] = 0.01;
        cost_scaling[652] = 0.01;
        cost_scaling[653] = 0.01;
        cost_scaling[654] = 0.01;
        cost_scaling[655] = 0.01;
        cost_scaling[656] = 0.01;
        cost_scaling[657] = 0.01;
        cost_scaling[658] = 0.01;
        cost_scaling[659] = 0.01;
        cost_scaling[660] = 0.01;
        cost_scaling[661] = 0.01;
        cost_scaling[662] = 0.01;
        cost_scaling[663] = 0.01;
        cost_scaling[664] = 0.01;
        cost_scaling[665] = 0.01;
        cost_scaling[666] = 0.01;
        cost_scaling[667] = 0.01;
        cost_scaling[668] = 0.01;
        cost_scaling[669] = 0.01;
        cost_scaling[670] = 0.01;
        cost_scaling[671] = 0.01;
        cost_scaling[672] = 0.01;
        cost_scaling[673] = 0.01;
        cost_scaling[674] = 0.01;
        cost_scaling[675] = 0.01;
        cost_scaling[676] = 0.01;
        cost_scaling[677] = 0.01;
        cost_scaling[678] = 0.01;
        cost_scaling[679] = 0.01;
        cost_scaling[680] = 0.01;
        cost_scaling[681] = 0.01;
        cost_scaling[682] = 0.01;
        cost_scaling[683] = 0.01;
        cost_scaling[684] = 0.01;
        cost_scaling[685] = 0.01;
        cost_scaling[686] = 0.01;
        cost_scaling[687] = 0.01;
        cost_scaling[688] = 0.01;
        cost_scaling[689] = 0.01;
        cost_scaling[690] = 0.01;
        cost_scaling[691] = 0.01;
        cost_scaling[692] = 0.01;
        cost_scaling[693] = 0.01;
        cost_scaling[694] = 0.01;
        cost_scaling[695] = 0.01;
        cost_scaling[696] = 0.01;
        cost_scaling[697] = 0.01;
        cost_scaling[698] = 0.01;
        cost_scaling[699] = 0.01;
        cost_scaling[700] = 0.01;
        cost_scaling[701] = 0.01;
        cost_scaling[702] = 0.01;
        cost_scaling[703] = 0.01;
        cost_scaling[704] = 0.01;
        cost_scaling[705] = 0.01;
        cost_scaling[706] = 0.01;
        cost_scaling[707] = 0.01;
        cost_scaling[708] = 0.01;
        cost_scaling[709] = 0.01;
        cost_scaling[710] = 0.01;
        cost_scaling[711] = 0.01;
        cost_scaling[712] = 0.01;
        cost_scaling[713] = 0.01;
        cost_scaling[714] = 0.01;
        cost_scaling[715] = 0.01;
        cost_scaling[716] = 0.01;
        cost_scaling[717] = 0.01;
        cost_scaling[718] = 0.01;
        cost_scaling[719] = 0.01;
        cost_scaling[720] = 0.01;
        cost_scaling[721] = 0.01;
        cost_scaling[722] = 0.01;
        cost_scaling[723] = 0.01;
        cost_scaling[724] = 0.01;
        cost_scaling[725] = 0.01;
        cost_scaling[726] = 0.01;
        cost_scaling[727] = 0.01;
        cost_scaling[728] = 0.01;
        cost_scaling[729] = 0.01;
        cost_scaling[730] = 0.01;
        cost_scaling[731] = 0.01;
        cost_scaling[732] = 0.01;
        cost_scaling[733] = 0.01;
        cost_scaling[734] = 0.01;
        cost_scaling[735] = 0.01;
        cost_scaling[736] = 0.01;
        cost_scaling[737] = 0.01;
        cost_scaling[738] = 0.01;
        cost_scaling[739] = 0.01;
        cost_scaling[740] = 0.01;
        cost_scaling[741] = 0.01;
        cost_scaling[742] = 0.01;
        cost_scaling[743] = 0.01;
        cost_scaling[744] = 0.01;
        cost_scaling[745] = 0.01;
        cost_scaling[746] = 0.01;
        cost_scaling[747] = 0.01;
        cost_scaling[748] = 0.01;
        cost_scaling[749] = 0.01;
        cost_scaling[750] = 0.01;
        cost_scaling[751] = 0.01;
        cost_scaling[752] = 0.01;
        cost_scaling[753] = 0.01;
        cost_scaling[754] = 0.01;
        cost_scaling[755] = 0.01;
        cost_scaling[756] = 0.01;
        cost_scaling[757] = 0.01;
        cost_scaling[758] = 0.01;
        cost_scaling[759] = 0.01;
        cost_scaling[760] = 0.01;
        cost_scaling[761] = 0.01;
        cost_scaling[762] = 0.01;
        cost_scaling[763] = 0.01;
        cost_scaling[764] = 0.01;
        cost_scaling[765] = 0.01;
        cost_scaling[766] = 0.01;
        cost_scaling[767] = 0.01;
        cost_scaling[768] = 0.01;
        cost_scaling[769] = 0.01;
        cost_scaling[770] = 0.01;
        cost_scaling[771] = 0.01;
        cost_scaling[772] = 0.01;
        cost_scaling[773] = 0.01;
        cost_scaling[774] = 0.01;
        cost_scaling[775] = 0.01;
        cost_scaling[776] = 0.01;
        cost_scaling[777] = 0.01;
        cost_scaling[778] = 0.01;
        cost_scaling[779] = 0.01;
        cost_scaling[780] = 0.01;
        cost_scaling[781] = 0.01;
        cost_scaling[782] = 0.01;
        cost_scaling[783] = 0.01;
        cost_scaling[784] = 0.01;
        cost_scaling[785] = 0.01;
        cost_scaling[786] = 0.01;
        cost_scaling[787] = 0.01;
        cost_scaling[788] = 0.01;
        cost_scaling[789] = 0.01;
        cost_scaling[790] = 0.01;
        cost_scaling[791] = 0.01;
        cost_scaling[792] = 0.01;
        cost_scaling[793] = 0.01;
        cost_scaling[794] = 0.01;
        cost_scaling[795] = 0.01;
        cost_scaling[796] = 0.01;
        cost_scaling[797] = 0.01;
        cost_scaling[798] = 0.01;
        cost_scaling[799] = 0.01;
        cost_scaling[800] = 0.01;
        cost_scaling[801] = 0.01;
        cost_scaling[802] = 0.01;
        cost_scaling[803] = 0.01;
        cost_scaling[804] = 0.01;
        cost_scaling[805] = 0.01;
        cost_scaling[806] = 0.01;
        cost_scaling[807] = 0.01;
        cost_scaling[808] = 0.01;
        cost_scaling[809] = 0.01;
        cost_scaling[810] = 0.01;
        cost_scaling[811] = 0.01;
        cost_scaling[812] = 0.01;
        cost_scaling[813] = 0.01;
        cost_scaling[814] = 0.01;
        cost_scaling[815] = 0.01;
        cost_scaling[816] = 0.01;
        cost_scaling[817] = 0.01;
        cost_scaling[818] = 0.01;
        cost_scaling[819] = 0.01;
        cost_scaling[820] = 0.01;
        cost_scaling[821] = 0.01;
        cost_scaling[822] = 0.01;
        cost_scaling[823] = 0.01;
        cost_scaling[824] = 0.01;
        cost_scaling[825] = 0.01;
        cost_scaling[826] = 0.01;
        cost_scaling[827] = 0.01;
        cost_scaling[828] = 0.01;
        cost_scaling[829] = 0.01;
        cost_scaling[830] = 0.01;
        cost_scaling[831] = 0.01;
        cost_scaling[832] = 0.01;
        cost_scaling[833] = 0.01;
        cost_scaling[834] = 0.01;
        cost_scaling[835] = 0.01;
        cost_scaling[836] = 0.01;
        cost_scaling[837] = 0.01;
        cost_scaling[838] = 0.01;
        cost_scaling[839] = 0.01;
        cost_scaling[840] = 0.01;
        cost_scaling[841] = 0.01;
        cost_scaling[842] = 0.01;
        cost_scaling[843] = 0.01;
        cost_scaling[844] = 0.01;
        cost_scaling[845] = 0.01;
        cost_scaling[846] = 0.01;
        cost_scaling[847] = 0.01;
        cost_scaling[848] = 0.01;
        cost_scaling[849] = 0.01;
        cost_scaling[850] = 0.01;
        cost_scaling[851] = 0.01;
        cost_scaling[852] = 0.01;
        cost_scaling[853] = 0.01;
        cost_scaling[854] = 0.01;
        cost_scaling[855] = 0.01;
        cost_scaling[856] = 0.01;
        cost_scaling[857] = 0.01;
        cost_scaling[858] = 0.01;
        cost_scaling[859] = 0.01;
        cost_scaling[860] = 0.01;
        cost_scaling[861] = 0.01;
        cost_scaling[862] = 0.01;
        cost_scaling[863] = 0.01;
        cost_scaling[864] = 0.01;
        cost_scaling[865] = 0.01;
        cost_scaling[866] = 0.01;
        cost_scaling[867] = 0.01;
        cost_scaling[868] = 0.01;
        cost_scaling[869] = 0.01;
        cost_scaling[870] = 0.01;
        cost_scaling[871] = 0.01;
        cost_scaling[872] = 0.01;
        cost_scaling[873] = 0.01;
        cost_scaling[874] = 0.01;
        cost_scaling[875] = 0.01;
        cost_scaling[876] = 0.01;
        cost_scaling[877] = 0.01;
        cost_scaling[878] = 0.01;
        cost_scaling[879] = 0.01;
        cost_scaling[880] = 0.01;
        cost_scaling[881] = 0.01;
        cost_scaling[882] = 0.01;
        cost_scaling[883] = 0.01;
        cost_scaling[884] = 0.01;
        cost_scaling[885] = 0.01;
        cost_scaling[886] = 0.01;
        cost_scaling[887] = 0.01;
        cost_scaling[888] = 0.01;
        cost_scaling[889] = 0.01;
        cost_scaling[890] = 0.01;
        cost_scaling[891] = 0.01;
        cost_scaling[892] = 0.01;
        cost_scaling[893] = 0.01;
        cost_scaling[894] = 0.01;
        cost_scaling[895] = 0.01;
        cost_scaling[896] = 0.01;
        cost_scaling[897] = 0.01;
        cost_scaling[898] = 0.01;
        cost_scaling[899] = 0.01;
        cost_scaling[900] = 0.01;
        cost_scaling[901] = 0.01;
        cost_scaling[902] = 0.01;
        cost_scaling[903] = 0.01;
        cost_scaling[904] = 0.01;
        cost_scaling[905] = 0.01;
        cost_scaling[906] = 0.01;
        cost_scaling[907] = 0.01;
        cost_scaling[908] = 0.01;
        cost_scaling[909] = 0.01;
        cost_scaling[910] = 0.01;
        cost_scaling[911] = 0.01;
        cost_scaling[912] = 0.01;
        cost_scaling[913] = 0.01;
        cost_scaling[914] = 0.01;
        cost_scaling[915] = 0.01;
        cost_scaling[916] = 0.01;
        cost_scaling[917] = 0.01;
        cost_scaling[918] = 0.01;
        cost_scaling[919] = 0.01;
        cost_scaling[920] = 0.01;
        cost_scaling[921] = 0.01;
        cost_scaling[922] = 0.01;
        cost_scaling[923] = 0.01;
        cost_scaling[924] = 0.01;
        cost_scaling[925] = 0.01;
        cost_scaling[926] = 0.01;
        cost_scaling[927] = 0.01;
        cost_scaling[928] = 0.01;
        cost_scaling[929] = 0.01;
        cost_scaling[930] = 0.01;
        cost_scaling[931] = 0.01;
        cost_scaling[932] = 0.01;
        cost_scaling[933] = 0.01;
        cost_scaling[934] = 0.01;
        cost_scaling[935] = 0.01;
        cost_scaling[936] = 0.01;
        cost_scaling[937] = 0.01;
        cost_scaling[938] = 0.01;
        cost_scaling[939] = 0.01;
        cost_scaling[940] = 0.01;
        cost_scaling[941] = 0.01;
        cost_scaling[942] = 0.01;
        cost_scaling[943] = 0.01;
        cost_scaling[944] = 0.01;
        cost_scaling[945] = 0.01;
        cost_scaling[946] = 0.01;
        cost_scaling[947] = 0.01;
        cost_scaling[948] = 0.01;
        cost_scaling[949] = 0.01;
        cost_scaling[950] = 0.01;
        cost_scaling[951] = 0.01;
        cost_scaling[952] = 0.01;
        cost_scaling[953] = 0.01;
        cost_scaling[954] = 0.01;
        cost_scaling[955] = 0.01;
        cost_scaling[956] = 0.01;
        cost_scaling[957] = 0.01;
        cost_scaling[958] = 0.01;
        cost_scaling[959] = 0.01;
        cost_scaling[960] = 0.01;
        cost_scaling[961] = 0.01;
        cost_scaling[962] = 0.01;
        cost_scaling[963] = 0.01;
        cost_scaling[964] = 0.01;
        cost_scaling[965] = 0.01;
        cost_scaling[966] = 0.01;
        cost_scaling[967] = 0.01;
        cost_scaling[968] = 0.01;
        cost_scaling[969] = 0.01;
        cost_scaling[970] = 0.01;
        cost_scaling[971] = 0.01;
        cost_scaling[972] = 0.01;
        cost_scaling[973] = 0.01;
        cost_scaling[974] = 0.01;
        cost_scaling[975] = 0.01;
        cost_scaling[976] = 0.01;
        cost_scaling[977] = 0.01;
        cost_scaling[978] = 0.01;
        cost_scaling[979] = 0.01;
        cost_scaling[980] = 0.01;
        cost_scaling[981] = 0.01;
        cost_scaling[982] = 0.01;
        cost_scaling[983] = 0.01;
        cost_scaling[984] = 0.01;
        cost_scaling[985] = 0.01;
        cost_scaling[986] = 0.01;
        cost_scaling[987] = 0.01;
        cost_scaling[988] = 0.01;
        cost_scaling[989] = 0.01;
        cost_scaling[990] = 0.01;
        cost_scaling[991] = 0.01;
        cost_scaling[992] = 0.01;
        cost_scaling[993] = 0.01;
        cost_scaling[994] = 0.01;
        cost_scaling[995] = 0.01;
        cost_scaling[996] = 0.01;
        cost_scaling[997] = 0.01;
        cost_scaling[998] = 0.01;
        cost_scaling[999] = 0.01;
        cost_scaling[1000] = 0.01;
        cost_scaling[1001] = 0.01;
        cost_scaling[1002] = 0.01;
        cost_scaling[1003] = 0.01;
        cost_scaling[1004] = 0.01;
        cost_scaling[1005] = 0.01;
        cost_scaling[1006] = 0.01;
        cost_scaling[1007] = 0.01;
        cost_scaling[1008] = 0.01;
        cost_scaling[1009] = 0.01;
        cost_scaling[1010] = 0.01;
        cost_scaling[1011] = 0.01;
        cost_scaling[1012] = 0.01;
        cost_scaling[1013] = 0.01;
        cost_scaling[1014] = 0.01;
        cost_scaling[1015] = 0.01;
        cost_scaling[1016] = 0.01;
        cost_scaling[1017] = 0.01;
        cost_scaling[1018] = 0.01;
        cost_scaling[1019] = 0.01;
        cost_scaling[1020] = 0.01;
        cost_scaling[1021] = 0.01;
        cost_scaling[1022] = 0.01;
        cost_scaling[1023] = 0.01;
        cost_scaling[1024] = 0.01;
        cost_scaling[1025] = 0.01;
        cost_scaling[1026] = 0.01;
        cost_scaling[1027] = 0.01;
        cost_scaling[1028] = 0.01;
        cost_scaling[1029] = 0.01;
        cost_scaling[1030] = 0.01;
        cost_scaling[1031] = 0.01;
        cost_scaling[1032] = 0.01;
        cost_scaling[1033] = 0.01;
        cost_scaling[1034] = 0.01;
        cost_scaling[1035] = 0.01;
        cost_scaling[1036] = 0.01;
        cost_scaling[1037] = 0.01;
        cost_scaling[1038] = 0.01;
        cost_scaling[1039] = 0.01;
        cost_scaling[1040] = 0.01;
        cost_scaling[1041] = 0.01;
        cost_scaling[1042] = 0.01;
        cost_scaling[1043] = 0.01;
        cost_scaling[1044] = 0.01;
        cost_scaling[1045] = 0.01;
        cost_scaling[1046] = 0.01;
        cost_scaling[1047] = 0.01;
        cost_scaling[1048] = 0.01;
        cost_scaling[1049] = 0.01;
        cost_scaling[1050] = 0.01;
        cost_scaling[1051] = 0.01;
        cost_scaling[1052] = 0.01;
        cost_scaling[1053] = 0.01;
        cost_scaling[1054] = 0.01;
        cost_scaling[1055] = 0.01;
        cost_scaling[1056] = 0.01;
        cost_scaling[1057] = 0.01;
        cost_scaling[1058] = 0.01;
        cost_scaling[1059] = 0.01;
        cost_scaling[1060] = 0.01;
        cost_scaling[1061] = 0.01;
        cost_scaling[1062] = 0.01;
        cost_scaling[1063] = 0.01;
        cost_scaling[1064] = 0.01;
        cost_scaling[1065] = 0.01;
        cost_scaling[1066] = 0.01;
        cost_scaling[1067] = 0.01;
        cost_scaling[1068] = 0.01;
        cost_scaling[1069] = 0.01;
        cost_scaling[1070] = 0.01;
        cost_scaling[1071] = 0.01;
        cost_scaling[1072] = 0.01;
        cost_scaling[1073] = 0.01;
        cost_scaling[1074] = 0.01;
        cost_scaling[1075] = 0.01;
        cost_scaling[1076] = 0.01;
        cost_scaling[1077] = 0.01;
        cost_scaling[1078] = 0.01;
        cost_scaling[1079] = 0.01;
        cost_scaling[1080] = 0.01;
        cost_scaling[1081] = 0.01;
        cost_scaling[1082] = 0.01;
        cost_scaling[1083] = 0.01;
        cost_scaling[1084] = 0.01;
        cost_scaling[1085] = 0.01;
        cost_scaling[1086] = 0.01;
        cost_scaling[1087] = 0.01;
        cost_scaling[1088] = 0.01;
        cost_scaling[1089] = 0.01;
        cost_scaling[1090] = 0.01;
        cost_scaling[1091] = 0.01;
        cost_scaling[1092] = 0.01;
        cost_scaling[1093] = 0.01;
        cost_scaling[1094] = 0.01;
        cost_scaling[1095] = 0.01;
        cost_scaling[1096] = 0.01;
        cost_scaling[1097] = 0.01;
        cost_scaling[1098] = 0.01;
        cost_scaling[1099] = 0.01;
        cost_scaling[1100] = 0.01;
        cost_scaling[1101] = 0.01;
        cost_scaling[1102] = 0.01;
        cost_scaling[1103] = 0.01;
        cost_scaling[1104] = 0.01;
        cost_scaling[1105] = 0.01;
        cost_scaling[1106] = 0.01;
        cost_scaling[1107] = 0.01;
        cost_scaling[1108] = 0.01;
        cost_scaling[1109] = 0.01;
        cost_scaling[1110] = 0.01;
        cost_scaling[1111] = 0.01;
        cost_scaling[1112] = 0.01;
        cost_scaling[1113] = 0.01;
        cost_scaling[1114] = 0.01;
        cost_scaling[1115] = 0.01;
        cost_scaling[1116] = 0.01;
        cost_scaling[1117] = 0.01;
        cost_scaling[1118] = 0.01;
        cost_scaling[1119] = 0.01;
        cost_scaling[1120] = 0.01;
        cost_scaling[1121] = 0.01;
        cost_scaling[1122] = 0.01;
        cost_scaling[1123] = 0.01;
        cost_scaling[1124] = 0.01;
        cost_scaling[1125] = 0.01;
        cost_scaling[1126] = 0.01;
        cost_scaling[1127] = 0.01;
        cost_scaling[1128] = 0.01;
        cost_scaling[1129] = 0.01;
        cost_scaling[1130] = 0.01;
        cost_scaling[1131] = 0.01;
        cost_scaling[1132] = 0.01;
        cost_scaling[1133] = 0.01;
        cost_scaling[1134] = 0.01;
        cost_scaling[1135] = 0.01;
        cost_scaling[1136] = 0.01;
        cost_scaling[1137] = 0.01;
        cost_scaling[1138] = 0.01;
        cost_scaling[1139] = 0.01;
        cost_scaling[1140] = 0.01;
        cost_scaling[1141] = 0.01;
        cost_scaling[1142] = 0.01;
        cost_scaling[1143] = 0.01;
        cost_scaling[1144] = 0.01;
        cost_scaling[1145] = 0.01;
        cost_scaling[1146] = 0.01;
        cost_scaling[1147] = 0.01;
        cost_scaling[1148] = 0.01;
        cost_scaling[1149] = 0.01;
        cost_scaling[1150] = 0.01;
        cost_scaling[1151] = 0.01;
        cost_scaling[1152] = 0.01;
        cost_scaling[1153] = 0.01;
        cost_scaling[1154] = 0.01;
        cost_scaling[1155] = 0.01;
        cost_scaling[1156] = 0.01;
        cost_scaling[1157] = 0.01;
        cost_scaling[1158] = 0.01;
        cost_scaling[1159] = 0.01;
        cost_scaling[1160] = 0.01;
        cost_scaling[1161] = 0.01;
        cost_scaling[1162] = 0.01;
        cost_scaling[1163] = 0.01;
        cost_scaling[1164] = 0.01;
        cost_scaling[1165] = 0.01;
        cost_scaling[1166] = 0.01;
        cost_scaling[1167] = 0.01;
        cost_scaling[1168] = 0.01;
        cost_scaling[1169] = 0.01;
        cost_scaling[1170] = 0.01;
        cost_scaling[1171] = 0.01;
        cost_scaling[1172] = 0.01;
        cost_scaling[1173] = 0.01;
        cost_scaling[1174] = 0.01;
        cost_scaling[1175] = 0.01;
        cost_scaling[1176] = 0.01;
        cost_scaling[1177] = 0.01;
        cost_scaling[1178] = 0.01;
        cost_scaling[1179] = 0.01;
        cost_scaling[1180] = 0.01;
        cost_scaling[1181] = 0.01;
        cost_scaling[1182] = 0.01;
        cost_scaling[1183] = 0.01;
        cost_scaling[1184] = 0.01;
        cost_scaling[1185] = 0.01;
        cost_scaling[1186] = 0.01;
        cost_scaling[1187] = 0.01;
        cost_scaling[1188] = 0.01;
        cost_scaling[1189] = 0.01;
        cost_scaling[1190] = 0.01;
        cost_scaling[1191] = 0.01;
        cost_scaling[1192] = 0.01;
        cost_scaling[1193] = 0.01;
        cost_scaling[1194] = 0.01;
        cost_scaling[1195] = 0.01;
        cost_scaling[1196] = 0.01;
        cost_scaling[1197] = 0.01;
        cost_scaling[1198] = 0.01;
        cost_scaling[1199] = 0.01;
        cost_scaling[1200] = 0.01;
        cost_scaling[1201] = 0.01;
        cost_scaling[1202] = 0.01;
        cost_scaling[1203] = 0.01;
        cost_scaling[1204] = 0.01;
        cost_scaling[1205] = 0.01;
        cost_scaling[1206] = 0.01;
        cost_scaling[1207] = 0.01;
        cost_scaling[1208] = 0.01;
        cost_scaling[1209] = 0.01;
        cost_scaling[1210] = 0.01;
        cost_scaling[1211] = 0.01;
        cost_scaling[1212] = 0.01;
        cost_scaling[1213] = 0.01;
        cost_scaling[1214] = 0.01;
        cost_scaling[1215] = 0.01;
        cost_scaling[1216] = 0.01;
        cost_scaling[1217] = 0.01;
        cost_scaling[1218] = 0.01;
        cost_scaling[1219] = 0.01;
        cost_scaling[1220] = 0.01;
        cost_scaling[1221] = 0.01;
        cost_scaling[1222] = 0.01;
        cost_scaling[1223] = 0.01;
        cost_scaling[1224] = 0.01;
        cost_scaling[1225] = 0.01;
        cost_scaling[1226] = 0.01;
        cost_scaling[1227] = 0.01;
        cost_scaling[1228] = 0.01;
        cost_scaling[1229] = 0.01;
        cost_scaling[1230] = 0.01;
        cost_scaling[1231] = 0.01;
        cost_scaling[1232] = 0.01;
        cost_scaling[1233] = 0.01;
        cost_scaling[1234] = 0.01;
        cost_scaling[1235] = 0.01;
        cost_scaling[1236] = 0.01;
        cost_scaling[1237] = 0.01;
        cost_scaling[1238] = 0.01;
        cost_scaling[1239] = 0.01;
        cost_scaling[1240] = 0.01;
        cost_scaling[1241] = 0.01;
        cost_scaling[1242] = 0.01;
        cost_scaling[1243] = 0.01;
        cost_scaling[1244] = 0.01;
        cost_scaling[1245] = 0.01;
        cost_scaling[1246] = 0.01;
        cost_scaling[1247] = 0.01;
        cost_scaling[1248] = 0.01;
        cost_scaling[1249] = 0.01;
        cost_scaling[1250] = 0.01;
        cost_scaling[1251] = 0.01;
        cost_scaling[1252] = 0.01;
        cost_scaling[1253] = 0.01;
        cost_scaling[1254] = 0.01;
        cost_scaling[1255] = 0.01;
        cost_scaling[1256] = 0.01;
        cost_scaling[1257] = 0.01;
        cost_scaling[1258] = 0.01;
        cost_scaling[1259] = 0.01;
        cost_scaling[1260] = 0.01;
        cost_scaling[1261] = 0.01;
        cost_scaling[1262] = 0.01;
        cost_scaling[1263] = 0.01;
        cost_scaling[1264] = 0.01;
        cost_scaling[1265] = 0.01;
        cost_scaling[1266] = 0.01;
        cost_scaling[1267] = 0.01;
        cost_scaling[1268] = 0.01;
        cost_scaling[1269] = 0.01;
        cost_scaling[1270] = 0.01;
        cost_scaling[1271] = 0.01;
        cost_scaling[1272] = 0.01;
        cost_scaling[1273] = 0.01;
        cost_scaling[1274] = 0.01;
        cost_scaling[1275] = 0.01;
        cost_scaling[1276] = 0.01;
        cost_scaling[1277] = 0.01;
        cost_scaling[1278] = 0.01;
        cost_scaling[1279] = 0.01;
        cost_scaling[1280] = 0.01;
        cost_scaling[1281] = 0.01;
        cost_scaling[1282] = 0.01;
        cost_scaling[1283] = 0.01;
        cost_scaling[1284] = 0.01;
        cost_scaling[1285] = 0.01;
        cost_scaling[1286] = 0.01;
        cost_scaling[1287] = 0.01;
        cost_scaling[1288] = 0.01;
        cost_scaling[1289] = 0.01;
        cost_scaling[1290] = 0.01;
        cost_scaling[1291] = 0.01;
        cost_scaling[1292] = 0.01;
        cost_scaling[1293] = 0.01;
        cost_scaling[1294] = 0.01;
        cost_scaling[1295] = 0.01;
        cost_scaling[1296] = 0.01;
        cost_scaling[1297] = 0.01;
        cost_scaling[1298] = 0.01;
        cost_scaling[1299] = 0.01;
        cost_scaling[1300] = 0.01;
        cost_scaling[1301] = 0.01;
        cost_scaling[1302] = 0.01;
        cost_scaling[1303] = 0.01;
        cost_scaling[1304] = 0.01;
        cost_scaling[1305] = 0.01;
        cost_scaling[1306] = 0.01;
        cost_scaling[1307] = 0.01;
        cost_scaling[1308] = 0.01;
        cost_scaling[1309] = 0.01;
        cost_scaling[1310] = 0.01;
        cost_scaling[1311] = 0.01;
        cost_scaling[1312] = 0.01;
        cost_scaling[1313] = 0.01;
        cost_scaling[1314] = 0.01;
        cost_scaling[1315] = 0.01;
        cost_scaling[1316] = 0.01;
        cost_scaling[1317] = 0.01;
        cost_scaling[1318] = 0.01;
        cost_scaling[1319] = 0.01;
        cost_scaling[1320] = 0.01;
        cost_scaling[1321] = 0.01;
        cost_scaling[1322] = 0.01;
        cost_scaling[1323] = 0.01;
        cost_scaling[1324] = 0.01;
        cost_scaling[1325] = 0.01;
        cost_scaling[1326] = 0.01;
        cost_scaling[1327] = 0.01;
        cost_scaling[1328] = 0.01;
        cost_scaling[1329] = 0.01;
        cost_scaling[1330] = 0.01;
        cost_scaling[1331] = 0.01;
        cost_scaling[1332] = 0.01;
        cost_scaling[1333] = 0.01;
        cost_scaling[1334] = 0.01;
        cost_scaling[1335] = 0.01;
        cost_scaling[1336] = 0.01;
        cost_scaling[1337] = 0.01;
        cost_scaling[1338] = 0.01;
        cost_scaling[1339] = 0.01;
        cost_scaling[1340] = 0.01;
        cost_scaling[1341] = 0.01;
        cost_scaling[1342] = 0.01;
        cost_scaling[1343] = 0.01;
        cost_scaling[1344] = 0.01;
        cost_scaling[1345] = 0.01;
        cost_scaling[1346] = 0.01;
        cost_scaling[1347] = 0.01;
        cost_scaling[1348] = 0.01;
        cost_scaling[1349] = 0.01;
        cost_scaling[1350] = 0.01;
        cost_scaling[1351] = 0.01;
        cost_scaling[1352] = 0.01;
        cost_scaling[1353] = 0.01;
        cost_scaling[1354] = 0.01;
        cost_scaling[1355] = 0.01;
        cost_scaling[1356] = 0.01;
        cost_scaling[1357] = 0.01;
        cost_scaling[1358] = 0.01;
        cost_scaling[1359] = 0.01;
        cost_scaling[1360] = 0.01;
        cost_scaling[1361] = 0.01;
        cost_scaling[1362] = 0.01;
        cost_scaling[1363] = 0.01;
        cost_scaling[1364] = 0.01;
        cost_scaling[1365] = 0.01;
        cost_scaling[1366] = 0.01;
        cost_scaling[1367] = 0.01;
        cost_scaling[1368] = 0.01;
        cost_scaling[1369] = 0.01;
        cost_scaling[1370] = 0.01;
        cost_scaling[1371] = 0.01;
        cost_scaling[1372] = 0.01;
        cost_scaling[1373] = 0.01;
        cost_scaling[1374] = 0.01;
        cost_scaling[1375] = 0.01;
        cost_scaling[1376] = 0.01;
        cost_scaling[1377] = 0.01;
        cost_scaling[1378] = 0.01;
        cost_scaling[1379] = 0.01;
        cost_scaling[1380] = 0.01;
        cost_scaling[1381] = 0.01;
        cost_scaling[1382] = 0.01;
        cost_scaling[1383] = 0.01;
        cost_scaling[1384] = 0.01;
        cost_scaling[1385] = 0.01;
        cost_scaling[1386] = 0.01;
        cost_scaling[1387] = 0.01;
        cost_scaling[1388] = 0.01;
        cost_scaling[1389] = 0.01;
        cost_scaling[1390] = 0.01;
        cost_scaling[1391] = 0.01;
        cost_scaling[1392] = 0.01;
        cost_scaling[1393] = 0.01;
        cost_scaling[1394] = 0.01;
        cost_scaling[1395] = 0.01;
        cost_scaling[1396] = 0.01;
        cost_scaling[1397] = 0.01;
        cost_scaling[1398] = 0.01;
        cost_scaling[1399] = 0.01;
        cost_scaling[1400] = 0.01;
        cost_scaling[1401] = 0.01;
        cost_scaling[1402] = 0.01;
        cost_scaling[1403] = 0.01;
        cost_scaling[1404] = 0.01;
        cost_scaling[1405] = 0.01;
        cost_scaling[1406] = 0.01;
        cost_scaling[1407] = 0.01;
        cost_scaling[1408] = 0.01;
        cost_scaling[1409] = 0.01;
        cost_scaling[1410] = 0.01;
        cost_scaling[1411] = 0.01;
        cost_scaling[1412] = 0.01;
        cost_scaling[1413] = 0.01;
        cost_scaling[1414] = 0.01;
        cost_scaling[1415] = 0.01;
        cost_scaling[1416] = 0.01;
        cost_scaling[1417] = 0.01;
        cost_scaling[1418] = 0.01;
        cost_scaling[1419] = 0.01;
        cost_scaling[1420] = 0.01;
        cost_scaling[1421] = 0.01;
        cost_scaling[1422] = 0.01;
        cost_scaling[1423] = 0.01;
        cost_scaling[1424] = 0.01;
        cost_scaling[1425] = 0.01;
        cost_scaling[1426] = 0.01;
        cost_scaling[1427] = 0.01;
        cost_scaling[1428] = 0.01;
        cost_scaling[1429] = 0.01;
        cost_scaling[1430] = 0.01;
        cost_scaling[1431] = 0.01;
        cost_scaling[1432] = 0.01;
        cost_scaling[1433] = 0.01;
        cost_scaling[1434] = 0.01;
        cost_scaling[1435] = 0.01;
        cost_scaling[1436] = 0.01;
        cost_scaling[1437] = 0.01;
        cost_scaling[1438] = 0.01;
        cost_scaling[1439] = 0.01;
        cost_scaling[1440] = 0.01;
        cost_scaling[1441] = 0.01;
        cost_scaling[1442] = 0.01;
        cost_scaling[1443] = 0.01;
        cost_scaling[1444] = 0.01;
        cost_scaling[1445] = 0.01;
        cost_scaling[1446] = 0.01;
        cost_scaling[1447] = 0.01;
        cost_scaling[1448] = 0.01;
        cost_scaling[1449] = 0.01;
        cost_scaling[1450] = 0.01;
        cost_scaling[1451] = 0.01;
        cost_scaling[1452] = 0.01;
        cost_scaling[1453] = 0.01;
        cost_scaling[1454] = 0.01;
        cost_scaling[1455] = 0.01;
        cost_scaling[1456] = 0.01;
        cost_scaling[1457] = 0.01;
        cost_scaling[1458] = 0.01;
        cost_scaling[1459] = 0.01;
        cost_scaling[1460] = 0.01;
        cost_scaling[1461] = 0.01;
        cost_scaling[1462] = 0.01;
        cost_scaling[1463] = 0.01;
        cost_scaling[1464] = 0.01;
        cost_scaling[1465] = 0.01;
        cost_scaling[1466] = 0.01;
        cost_scaling[1467] = 0.01;
        cost_scaling[1468] = 0.01;
        cost_scaling[1469] = 0.01;
        cost_scaling[1470] = 0.01;
        cost_scaling[1471] = 0.01;
        cost_scaling[1472] = 0.01;
        cost_scaling[1473] = 0.01;
        cost_scaling[1474] = 0.01;
        cost_scaling[1475] = 0.01;
        cost_scaling[1476] = 0.01;
        cost_scaling[1477] = 0.01;
        cost_scaling[1478] = 0.01;
        cost_scaling[1479] = 0.01;
        cost_scaling[1480] = 0.01;
        cost_scaling[1481] = 0.01;
        cost_scaling[1482] = 0.01;
        cost_scaling[1483] = 0.01;
        cost_scaling[1484] = 0.01;
        cost_scaling[1485] = 0.01;
        cost_scaling[1486] = 0.01;
        cost_scaling[1487] = 0.01;
        cost_scaling[1488] = 0.01;
        cost_scaling[1489] = 0.01;
        cost_scaling[1490] = 0.01;
        cost_scaling[1491] = 0.01;
        cost_scaling[1492] = 0.01;
        cost_scaling[1493] = 0.01;
        cost_scaling[1494] = 0.01;
        cost_scaling[1495] = 0.01;
        cost_scaling[1496] = 0.01;
        cost_scaling[1497] = 0.01;
        cost_scaling[1498] = 0.01;
        cost_scaling[1499] = 0.01;
        cost_scaling[1500] = 0.01;
        cost_scaling[1501] = 0.01;
        cost_scaling[1502] = 0.01;
        cost_scaling[1503] = 0.01;
        cost_scaling[1504] = 0.01;
        cost_scaling[1505] = 0.01;
        cost_scaling[1506] = 0.01;
        cost_scaling[1507] = 0.01;
        cost_scaling[1508] = 0.01;
        cost_scaling[1509] = 0.01;
        cost_scaling[1510] = 0.01;
        cost_scaling[1511] = 0.01;
        cost_scaling[1512] = 0.01;
        cost_scaling[1513] = 0.01;
        cost_scaling[1514] = 0.01;
        cost_scaling[1515] = 0.01;
        cost_scaling[1516] = 0.01;
        cost_scaling[1517] = 0.01;
        cost_scaling[1518] = 0.01;
        cost_scaling[1519] = 0.01;
        cost_scaling[1520] = 0.01;
        cost_scaling[1521] = 0.01;
        cost_scaling[1522] = 0.01;
        cost_scaling[1523] = 0.01;
        cost_scaling[1524] = 0.01;
        cost_scaling[1525] = 0.01;
        cost_scaling[1526] = 0.01;
        cost_scaling[1527] = 0.01;
        cost_scaling[1528] = 0.01;
        cost_scaling[1529] = 0.01;
        cost_scaling[1530] = 0.01;
        cost_scaling[1531] = 0.01;
        cost_scaling[1532] = 0.01;
        cost_scaling[1533] = 0.01;
        cost_scaling[1534] = 0.01;
        cost_scaling[1535] = 0.01;
        cost_scaling[1536] = 0.01;
        cost_scaling[1537] = 0.01;
        cost_scaling[1538] = 0.01;
        cost_scaling[1539] = 0.01;
        cost_scaling[1540] = 0.01;
        cost_scaling[1541] = 0.01;
        cost_scaling[1542] = 0.01;
        cost_scaling[1543] = 0.01;
        cost_scaling[1544] = 0.01;
        cost_scaling[1545] = 0.01;
        cost_scaling[1546] = 0.01;
        cost_scaling[1547] = 0.01;
        cost_scaling[1548] = 0.01;
        cost_scaling[1549] = 0.01;
        cost_scaling[1550] = 0.01;
        cost_scaling[1551] = 0.01;
        cost_scaling[1552] = 0.01;
        cost_scaling[1553] = 0.01;
        cost_scaling[1554] = 0.01;
        cost_scaling[1555] = 0.01;
        cost_scaling[1556] = 0.01;
        cost_scaling[1557] = 0.01;
        cost_scaling[1558] = 0.01;
        cost_scaling[1559] = 0.01;
        cost_scaling[1560] = 0.01;
        cost_scaling[1561] = 0.01;
        cost_scaling[1562] = 0.01;
        cost_scaling[1563] = 0.01;
        cost_scaling[1564] = 0.01;
        cost_scaling[1565] = 0.01;
        cost_scaling[1566] = 0.01;
        cost_scaling[1567] = 0.01;
        cost_scaling[1568] = 0.01;
        cost_scaling[1569] = 0.01;
        cost_scaling[1570] = 0.01;
        cost_scaling[1571] = 0.01;
        cost_scaling[1572] = 0.01;
        cost_scaling[1573] = 0.01;
        cost_scaling[1574] = 0.01;
        cost_scaling[1575] = 0.01;
        cost_scaling[1576] = 0.01;
        cost_scaling[1577] = 0.01;
        cost_scaling[1578] = 0.01;
        cost_scaling[1579] = 0.01;
        cost_scaling[1580] = 0.01;
        cost_scaling[1581] = 0.01;
        cost_scaling[1582] = 0.01;
        cost_scaling[1583] = 0.01;
        cost_scaling[1584] = 0.01;
        cost_scaling[1585] = 0.01;
        cost_scaling[1586] = 0.01;
        cost_scaling[1587] = 0.01;
        cost_scaling[1588] = 0.01;
        cost_scaling[1589] = 0.01;
        cost_scaling[1590] = 0.01;
        cost_scaling[1591] = 0.01;
        cost_scaling[1592] = 0.01;
        cost_scaling[1593] = 0.01;
        cost_scaling[1594] = 0.01;
        cost_scaling[1595] = 0.01;
        cost_scaling[1596] = 0.01;
        cost_scaling[1597] = 0.01;
        cost_scaling[1598] = 0.01;
        cost_scaling[1599] = 0.01;
        cost_scaling[1600] = 0.01;
        cost_scaling[1601] = 0.01;
        cost_scaling[1602] = 0.01;
        cost_scaling[1603] = 0.01;
        cost_scaling[1604] = 0.01;
        cost_scaling[1605] = 0.01;
        cost_scaling[1606] = 0.01;
        cost_scaling[1607] = 0.01;
        cost_scaling[1608] = 0.01;
        cost_scaling[1609] = 0.01;
        cost_scaling[1610] = 0.01;
        cost_scaling[1611] = 0.01;
        cost_scaling[1612] = 0.01;
        cost_scaling[1613] = 0.01;
        cost_scaling[1614] = 0.01;
        cost_scaling[1615] = 0.01;
        cost_scaling[1616] = 0.01;
        cost_scaling[1617] = 0.01;
        cost_scaling[1618] = 0.01;
        cost_scaling[1619] = 0.01;
        cost_scaling[1620] = 0.01;
        cost_scaling[1621] = 0.01;
        cost_scaling[1622] = 0.01;
        cost_scaling[1623] = 0.01;
        cost_scaling[1624] = 0.01;
        cost_scaling[1625] = 0.01;
        cost_scaling[1626] = 0.01;
        cost_scaling[1627] = 0.01;
        cost_scaling[1628] = 0.01;
        cost_scaling[1629] = 0.01;
        cost_scaling[1630] = 0.01;
        cost_scaling[1631] = 0.01;
        cost_scaling[1632] = 0.01;
        cost_scaling[1633] = 0.01;
        cost_scaling[1634] = 0.01;
        cost_scaling[1635] = 0.01;
        cost_scaling[1636] = 0.01;
        cost_scaling[1637] = 0.01;
        cost_scaling[1638] = 0.01;
        cost_scaling[1639] = 0.01;
        cost_scaling[1640] = 0.01;
        cost_scaling[1641] = 0.01;
        cost_scaling[1642] = 0.01;
        cost_scaling[1643] = 0.01;
        cost_scaling[1644] = 0.01;
        cost_scaling[1645] = 0.01;
        cost_scaling[1646] = 0.01;
        cost_scaling[1647] = 0.01;
        cost_scaling[1648] = 0.01;
        cost_scaling[1649] = 0.01;
        cost_scaling[1650] = 0.01;
        cost_scaling[1651] = 0.01;
        cost_scaling[1652] = 0.01;
        cost_scaling[1653] = 0.01;
        cost_scaling[1654] = 0.01;
        cost_scaling[1655] = 0.01;
        cost_scaling[1656] = 0.01;
        cost_scaling[1657] = 0.01;
        cost_scaling[1658] = 0.01;
        cost_scaling[1659] = 0.01;
        cost_scaling[1660] = 0.01;
        cost_scaling[1661] = 0.01;
        cost_scaling[1662] = 0.01;
        cost_scaling[1663] = 0.01;
        cost_scaling[1664] = 0.01;
        cost_scaling[1665] = 0.01;
        cost_scaling[1666] = 0.01;
        cost_scaling[1667] = 0.01;
        cost_scaling[1668] = 0.01;
        cost_scaling[1669] = 0.01;
        cost_scaling[1670] = 0.01;
        cost_scaling[1671] = 0.01;
        cost_scaling[1672] = 0.01;
        cost_scaling[1673] = 0.01;
        cost_scaling[1674] = 0.01;
        cost_scaling[1675] = 0.01;
        cost_scaling[1676] = 0.01;
        cost_scaling[1677] = 0.01;
        cost_scaling[1678] = 0.01;
        cost_scaling[1679] = 0.01;
        cost_scaling[1680] = 0.01;
        cost_scaling[1681] = 0.01;
        cost_scaling[1682] = 0.01;
        cost_scaling[1683] = 0.01;
        cost_scaling[1684] = 0.01;
        cost_scaling[1685] = 0.01;
        cost_scaling[1686] = 0.01;
        cost_scaling[1687] = 0.01;
        cost_scaling[1688] = 0.01;
        cost_scaling[1689] = 0.01;
        cost_scaling[1690] = 0.01;
        cost_scaling[1691] = 0.01;
        cost_scaling[1692] = 0.01;
        cost_scaling[1693] = 0.01;
        cost_scaling[1694] = 0.01;
        cost_scaling[1695] = 0.01;
        cost_scaling[1696] = 0.01;
        cost_scaling[1697] = 0.01;
        cost_scaling[1698] = 0.01;
        cost_scaling[1699] = 0.01;
        cost_scaling[1700] = 0.01;
        cost_scaling[1701] = 0.01;
        cost_scaling[1702] = 0.01;
        cost_scaling[1703] = 0.01;
        cost_scaling[1704] = 0.01;
        cost_scaling[1705] = 0.01;
        cost_scaling[1706] = 0.01;
        cost_scaling[1707] = 0.01;
        cost_scaling[1708] = 0.01;
        cost_scaling[1709] = 0.01;
        cost_scaling[1710] = 0.01;
        cost_scaling[1711] = 0.01;
        cost_scaling[1712] = 0.01;
        cost_scaling[1713] = 0.01;
        cost_scaling[1714] = 0.01;
        cost_scaling[1715] = 0.01;
        cost_scaling[1716] = 0.01;
        cost_scaling[1717] = 0.01;
        cost_scaling[1718] = 0.01;
        cost_scaling[1719] = 0.01;
        cost_scaling[1720] = 0.01;
        cost_scaling[1721] = 0.01;
        cost_scaling[1722] = 0.01;
        cost_scaling[1723] = 0.01;
        cost_scaling[1724] = 0.01;
        cost_scaling[1725] = 0.01;
        cost_scaling[1726] = 0.01;
        cost_scaling[1727] = 0.01;
        cost_scaling[1728] = 0.01;
        cost_scaling[1729] = 0.01;
        cost_scaling[1730] = 0.01;
        cost_scaling[1731] = 0.01;
        cost_scaling[1732] = 0.01;
        cost_scaling[1733] = 0.01;
        cost_scaling[1734] = 0.01;
        cost_scaling[1735] = 0.01;
        cost_scaling[1736] = 0.01;
        cost_scaling[1737] = 0.01;
        cost_scaling[1738] = 0.01;
        cost_scaling[1739] = 0.01;
        cost_scaling[1740] = 0.01;
        cost_scaling[1741] = 0.01;
        cost_scaling[1742] = 0.01;
        cost_scaling[1743] = 0.01;
        cost_scaling[1744] = 0.01;
        cost_scaling[1745] = 0.01;
        cost_scaling[1746] = 0.01;
        cost_scaling[1747] = 0.01;
        cost_scaling[1748] = 0.01;
        cost_scaling[1749] = 0.01;
        cost_scaling[1750] = 0.01;
        cost_scaling[1751] = 0.01;
        cost_scaling[1752] = 0.01;
        cost_scaling[1753] = 0.01;
        cost_scaling[1754] = 0.01;
        cost_scaling[1755] = 0.01;
        cost_scaling[1756] = 0.01;
        cost_scaling[1757] = 0.01;
        cost_scaling[1758] = 0.01;
        cost_scaling[1759] = 0.01;
        cost_scaling[1760] = 0.01;
        cost_scaling[1761] = 0.01;
        cost_scaling[1762] = 0.01;
        cost_scaling[1763] = 0.01;
        cost_scaling[1764] = 0.01;
        cost_scaling[1765] = 0.01;
        cost_scaling[1766] = 0.01;
        cost_scaling[1767] = 0.01;
        cost_scaling[1768] = 0.01;
        cost_scaling[1769] = 0.01;
        cost_scaling[1770] = 0.01;
        cost_scaling[1771] = 0.01;
        cost_scaling[1772] = 0.01;
        cost_scaling[1773] = 0.01;
        cost_scaling[1774] = 0.01;
        cost_scaling[1775] = 0.01;
        cost_scaling[1776] = 0.01;
        cost_scaling[1777] = 0.01;
        cost_scaling[1778] = 0.01;
        cost_scaling[1779] = 0.01;
        cost_scaling[1780] = 0.01;
        cost_scaling[1781] = 0.01;
        cost_scaling[1782] = 0.01;
        cost_scaling[1783] = 0.01;
        cost_scaling[1784] = 0.01;
        cost_scaling[1785] = 0.01;
        cost_scaling[1786] = 0.01;
        cost_scaling[1787] = 0.01;
        cost_scaling[1788] = 0.01;
        cost_scaling[1789] = 0.01;
        cost_scaling[1790] = 0.01;
        cost_scaling[1791] = 0.01;
        cost_scaling[1792] = 0.01;
        cost_scaling[1793] = 0.01;
        cost_scaling[1794] = 0.01;
        cost_scaling[1795] = 0.01;
        cost_scaling[1796] = 0.01;
        cost_scaling[1797] = 0.01;
        cost_scaling[1798] = 0.01;
        cost_scaling[1799] = 0.01;
        cost_scaling[1800] = 0.01;
        cost_scaling[1801] = 0.01;
        cost_scaling[1802] = 0.01;
        cost_scaling[1803] = 0.01;
        cost_scaling[1804] = 0.01;
        cost_scaling[1805] = 0.01;
        cost_scaling[1806] = 0.01;
        cost_scaling[1807] = 0.01;
        cost_scaling[1808] = 0.01;
        cost_scaling[1809] = 0.01;
        cost_scaling[1810] = 0.01;
        cost_scaling[1811] = 0.01;
        cost_scaling[1812] = 0.01;
        cost_scaling[1813] = 0.01;
        cost_scaling[1814] = 0.01;
        cost_scaling[1815] = 0.01;
        cost_scaling[1816] = 0.01;
        cost_scaling[1817] = 0.01;
        cost_scaling[1818] = 0.01;
        cost_scaling[1819] = 0.01;
        cost_scaling[1820] = 0.01;
        cost_scaling[1821] = 0.01;
        cost_scaling[1822] = 0.01;
        cost_scaling[1823] = 0.01;
        cost_scaling[1824] = 0.01;
        cost_scaling[1825] = 0.01;
        cost_scaling[1826] = 0.01;
        cost_scaling[1827] = 0.01;
        cost_scaling[1828] = 0.01;
        cost_scaling[1829] = 0.01;
        cost_scaling[1830] = 0.01;
        cost_scaling[1831] = 0.01;
        cost_scaling[1832] = 0.01;
        cost_scaling[1833] = 0.01;
        cost_scaling[1834] = 0.01;
        cost_scaling[1835] = 0.01;
        cost_scaling[1836] = 0.01;
        cost_scaling[1837] = 0.01;
        cost_scaling[1838] = 0.01;
        cost_scaling[1839] = 0.01;
        cost_scaling[1840] = 0.01;
        cost_scaling[1841] = 0.01;
        cost_scaling[1842] = 0.01;
        cost_scaling[1843] = 0.01;
        cost_scaling[1844] = 0.01;
        cost_scaling[1845] = 0.01;
        cost_scaling[1846] = 0.01;
        cost_scaling[1847] = 0.01;
        cost_scaling[1848] = 0.01;
        cost_scaling[1849] = 0.01;
        cost_scaling[1850] = 0.01;
        cost_scaling[1851] = 0.01;
        cost_scaling[1852] = 0.01;
        cost_scaling[1853] = 0.01;
        cost_scaling[1854] = 0.01;
        cost_scaling[1855] = 0.01;
        cost_scaling[1856] = 0.01;
        cost_scaling[1857] = 0.01;
        cost_scaling[1858] = 0.01;
        cost_scaling[1859] = 0.01;
        cost_scaling[1860] = 0.01;
        cost_scaling[1861] = 0.01;
        cost_scaling[1862] = 0.01;
        cost_scaling[1863] = 0.01;
        cost_scaling[1864] = 0.01;
        cost_scaling[1865] = 0.01;
        cost_scaling[1866] = 0.01;
        cost_scaling[1867] = 0.01;
        cost_scaling[1868] = 0.01;
        cost_scaling[1869] = 0.01;
        cost_scaling[1870] = 0.01;
        cost_scaling[1871] = 0.01;
        cost_scaling[1872] = 0.01;
        cost_scaling[1873] = 0.01;
        cost_scaling[1874] = 0.01;
        cost_scaling[1875] = 0.01;
        cost_scaling[1876] = 0.01;
        cost_scaling[1877] = 0.01;
        cost_scaling[1878] = 0.01;
        cost_scaling[1879] = 0.01;
        cost_scaling[1880] = 0.01;
        cost_scaling[1881] = 0.01;
        cost_scaling[1882] = 0.01;
        cost_scaling[1883] = 0.01;
        cost_scaling[1884] = 0.01;
        cost_scaling[1885] = 0.01;
        cost_scaling[1886] = 0.01;
        cost_scaling[1887] = 0.01;
        cost_scaling[1888] = 0.01;
        cost_scaling[1889] = 0.01;
        cost_scaling[1890] = 0.01;
        cost_scaling[1891] = 0.01;
        cost_scaling[1892] = 0.01;
        cost_scaling[1893] = 0.01;
        cost_scaling[1894] = 0.01;
        cost_scaling[1895] = 0.01;
        cost_scaling[1896] = 0.01;
        cost_scaling[1897] = 0.01;
        cost_scaling[1898] = 0.01;
        cost_scaling[1899] = 0.01;
        cost_scaling[1900] = 0.01;
        cost_scaling[1901] = 0.01;
        cost_scaling[1902] = 0.01;
        cost_scaling[1903] = 0.01;
        cost_scaling[1904] = 0.01;
        cost_scaling[1905] = 0.01;
        cost_scaling[1906] = 0.01;
        cost_scaling[1907] = 0.01;
        cost_scaling[1908] = 0.01;
        cost_scaling[1909] = 0.01;
        cost_scaling[1910] = 0.01;
        cost_scaling[1911] = 0.01;
        cost_scaling[1912] = 0.01;
        cost_scaling[1913] = 0.01;
        cost_scaling[1914] = 0.01;
        cost_scaling[1915] = 0.01;
        cost_scaling[1916] = 0.01;
        cost_scaling[1917] = 0.01;
        cost_scaling[1918] = 0.01;
        cost_scaling[1919] = 0.01;
        cost_scaling[1920] = 0.01;
        cost_scaling[1921] = 0.01;
        cost_scaling[1922] = 0.01;
        cost_scaling[1923] = 0.01;
        cost_scaling[1924] = 0.01;
        cost_scaling[1925] = 0.01;
        cost_scaling[1926] = 0.01;
        cost_scaling[1927] = 0.01;
        cost_scaling[1928] = 0.01;
        cost_scaling[1929] = 0.01;
        cost_scaling[1930] = 0.01;
        cost_scaling[1931] = 0.01;
        cost_scaling[1932] = 0.01;
        cost_scaling[1933] = 0.01;
        cost_scaling[1934] = 0.01;
        cost_scaling[1935] = 0.01;
        cost_scaling[1936] = 0.01;
        cost_scaling[1937] = 0.01;
        cost_scaling[1938] = 0.01;
        cost_scaling[1939] = 0.01;
        cost_scaling[1940] = 0.01;
        cost_scaling[1941] = 0.01;
        cost_scaling[1942] = 0.01;
        cost_scaling[1943] = 0.01;
        cost_scaling[1944] = 0.01;
        cost_scaling[1945] = 0.01;
        cost_scaling[1946] = 0.01;
        cost_scaling[1947] = 0.01;
        cost_scaling[1948] = 0.01;
        cost_scaling[1949] = 0.01;
        cost_scaling[1950] = 0.01;
        cost_scaling[1951] = 0.01;
        cost_scaling[1952] = 0.01;
        cost_scaling[1953] = 0.01;
        cost_scaling[1954] = 0.01;
        cost_scaling[1955] = 0.01;
        cost_scaling[1956] = 0.01;
        cost_scaling[1957] = 0.01;
        cost_scaling[1958] = 0.01;
        cost_scaling[1959] = 0.01;
        cost_scaling[1960] = 0.01;
        cost_scaling[1961] = 0.01;
        cost_scaling[1962] = 0.01;
        cost_scaling[1963] = 0.01;
        cost_scaling[1964] = 0.01;
        cost_scaling[1965] = 0.01;
        cost_scaling[1966] = 0.01;
        cost_scaling[1967] = 0.01;
        cost_scaling[1968] = 0.01;
        cost_scaling[1969] = 0.01;
        cost_scaling[1970] = 0.01;
        cost_scaling[1971] = 0.01;
        cost_scaling[1972] = 0.01;
        cost_scaling[1973] = 0.01;
        cost_scaling[1974] = 0.01;
        cost_scaling[1975] = 0.01;
        cost_scaling[1976] = 0.01;
        cost_scaling[1977] = 0.01;
        cost_scaling[1978] = 0.01;
        cost_scaling[1979] = 0.01;
        cost_scaling[1980] = 0.01;
        cost_scaling[1981] = 0.01;
        cost_scaling[1982] = 0.01;
        cost_scaling[1983] = 0.01;
        cost_scaling[1984] = 0.01;
        cost_scaling[1985] = 0.01;
        cost_scaling[1986] = 0.01;
        cost_scaling[1987] = 0.01;
        cost_scaling[1988] = 0.01;
        cost_scaling[1989] = 0.01;
        cost_scaling[1990] = 0.01;
        cost_scaling[1991] = 0.01;
        cost_scaling[1992] = 0.01;
        cost_scaling[1993] = 0.01;
        cost_scaling[1994] = 0.01;
        cost_scaling[1995] = 0.01;
        cost_scaling[1996] = 0.01;
        cost_scaling[1997] = 0.01;
        cost_scaling[1998] = 0.01;
        cost_scaling[1999] = 0.01;
        cost_scaling[2000] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }



    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        
    }

    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 1;
    W_0[1+(NY0) * 1] = 1;
    W_0[2+(NY0) * 2] = 1;
    W_0[3+(NY0) * 3] = 1;
    W_0[4+(NY0) * 4] = 1;
    W_0[5+(NY0) * 5] = 1;
    W_0[6+(NY0) * 6] = 1;
    W_0[7+(NY0) * 7] = 1;
    W_0[8+(NY0) * 8] = 1;
    W_0[9+(NY0) * 9] = 1;
    W_0[10+(NY0) * 10] = 1;
    W_0[11+(NY0) * 11] = 1;
    W_0[12+(NY0) * 12] = 1;
    W_0[13+(NY0) * 13] = 1;
    W_0[14+(NY0) * 14] = 1;
    W_0[15+(NY0) * 15] = 1;
    W_0[16+(NY0) * 16] = 1;
    W_0[17+(NY0) * 17] = 1;
    W_0[18+(NY0) * 18] = 1;
    W_0[19+(NY0) * 19] = 1;
    W_0[20+(NY0) * 20] = 1;
    W_0[21+(NY0) * 21] = 1;
    W_0[22+(NY0) * 22] = 1;
    W_0[23+(NY0) * 23] = 1;
    W_0[24+(NY0) * 24] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 1;
    W[1+(NY) * 1] = 1;
    W[2+(NY) * 2] = 1;
    W[3+(NY) * 3] = 1;
    W[4+(NY) * 4] = 1;
    W[5+(NY) * 5] = 1;
    W[6+(NY) * 6] = 1;
    W[7+(NY) * 7] = 1;
    W[8+(NY) * 8] = 1;
    W[9+(NY) * 9] = 1;
    W[10+(NY) * 10] = 1;
    W[11+(NY) * 11] = 1;
    W[12+(NY) * 12] = 1;
    W[13+(NY) * 13] = 1;
    W[14+(NY) * 14] = 1;
    W[15+(NY) * 15] = 1;
    W[16+(NY) * 16] = 1;
    W[17+(NY) * 17] = 1;
    W[18+(NY) * 18] = 1;
    W[19+(NY) * 19] = 1;
    W[20+(NY) * 20] = 1;
    W[21+(NY) * 21] = 1;
    W[22+(NY) * 22] = 1;
    W[23+(NY) * 23] = 1;
    W[24+(NY) * 24] = 1;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 1;
    W_e[1+(NYN) * 1] = 1;
    W_e[2+(NYN) * 2] = 1;
    W_e[3+(NYN) * 3] = 1;
    W_e[4+(NYN) * 4] = 1;
    W_e[5+(NYN) * 5] = 1;
    W_e[6+(NYN) * 6] = 1;
    W_e[7+(NYN) * 7] = 1;
    W_e[8+(NYN) * 8] = 1;
    W_e[9+(NYN) * 9] = 1;
    W_e[10+(NYN) * 10] = 1;
    W_e[11+(NYN) * 11] = 1;
    W_e[12+(NYN) * 12] = 1;
    W_e[13+(NYN) * 13] = 1;
    W_e[14+(NYN) * 14] = 1;
    W_e[15+(NYN) * 15] = 1;
    W_e[16+(NYN) * 16] = 1;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun", &capsule->cost_y_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun_jac", &capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun", &capsule->cost_y_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun_jac", &capsule->cost_y_fun_jac_ut_xt[i-1]);
    }
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun", &capsule->cost_y_e_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun_jac", &capsule->cost_y_e_fun_jac_ut_xt);







    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);












    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = -0.1745329251994329;
    ubu[0] = 0.1745329251994329;
    lbu[1] = -0.1745329251994329;
    ubu[1] = 0.1745329251994329;
    lbu[2] = -0.1745329251994329;
    ubu[2] = 0.1745329251994329;
    lbu[3] = -0.1745329251994329;
    ubu[3] = 0.1745329251994329;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);






    /* Path constraints */














    /* terminal constraints */




















}


static void nomtraj_aug_acados_create_set_opts(nomtraj_aug_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/



    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);

    double globalization_fixed_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_fixed_step_length", &globalization_fixed_step_length);




    int with_solution_sens_wrt_params = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params", &with_solution_sens_wrt_params);

    int with_value_sens_wrt_params = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    double levenberg_marquardt = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 2000;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    int log_primal_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);

    int log_dual_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_dual_step_norm", &log_dual_step_norm);

    double nlp_solver_tol_min_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_min_step_norm", &nlp_solver_tol_min_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    // set SQP specific options
    double nlp_solver_tol_stat = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 30;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    // set options for adaptive Levenberg-Marquardt Update
    bool with_adaptive_levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_adaptive_levenberg_marquardt", &with_adaptive_levenberg_marquardt);

    double adaptive_levenberg_marquardt_lam = 5;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_lam", &adaptive_levenberg_marquardt_lam);

    double adaptive_levenberg_marquardt_mu_min = 0.0000000000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu_min", &adaptive_levenberg_marquardt_mu_min);

    double adaptive_levenberg_marquardt_mu0 = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu0", &adaptive_levenberg_marquardt_mu0);

    double adaptive_levenberg_marquardt_obj_scalar = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_obj_scalar", &adaptive_levenberg_marquardt_obj_scalar);

    bool eval_residual_at_max_iter = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "eval_residual_at_max_iter", &eval_residual_at_max_iter);

    // QP scaling
    double qpscaling_ub_max_abs_eig = 100000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_ub_max_abs_eig", &qpscaling_ub_max_abs_eig);

    double qpscaling_lb_norm_inf_grad_obj = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_lb_norm_inf_grad_obj", &qpscaling_lb_norm_inf_grad_obj);

    qpscaling_scale_objective_type qpscaling_scale_objective = NO_OBJECTIVE_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_objective", &qpscaling_scale_objective);

    ocp_nlp_qpscaling_constraint_type qpscaling_scale_constraints = NO_CONSTRAINT_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_constraints", &qpscaling_scale_constraints);

    // NLP QP tol strategy
    ocp_nlp_qp_tol_strategy_t nlp_qp_tol_strategy = FIXED_QP_TOL;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_strategy", &nlp_qp_tol_strategy);

    double nlp_qp_tol_reduction_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_reduction_factor", &nlp_qp_tol_reduction_factor);

    double nlp_qp_tol_safety_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_safety_factor", &nlp_qp_tol_safety_factor);

    double nlp_qp_tol_min_stat = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_stat", &nlp_qp_tol_min_stat);

    double nlp_qp_tol_min_eq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_eq", &nlp_qp_tol_min_eq);

    double nlp_qp_tol_min_ineq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_ineq", &nlp_qp_tol_min_ineq);

    double nlp_qp_tol_min_comp = 0.00000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_comp", &nlp_qp_tol_min_comp);

    bool with_anderson_acceleration = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    double anderson_activation_threshold = 10;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "anderson_activation_threshold", &anderson_activation_threshold);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
}


/**
 * Internal function for nomtraj_aug_acados_create: step 7
 */
void nomtraj_aug_acados_set_nlp_out(nomtraj_aug_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for nomtraj_aug_acados_create: step 9
 */
int nomtraj_aug_acados_create_precompute(nomtraj_aug_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int nomtraj_aug_acados_create_with_discretization(nomtraj_aug_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != NOMTRAJ_AUG_N && !new_time_steps) {
        fprintf(stderr, "nomtraj_aug_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, NOMTRAJ_AUG_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    nomtraj_aug_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = nomtraj_aug_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    nomtraj_aug_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    nomtraj_aug_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    nomtraj_aug_acados_create_setup_functions(capsule);
    nomtraj_aug_acados_setup_nlp_in(capsule, N, new_time_steps);
    nomtraj_aug_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = nomtraj_aug_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int nomtraj_aug_acados_update_qp_solver_cond_N(nomtraj_aug_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from nomtraj_aug_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = nomtraj_aug_acados_create_precompute(capsule);
    return status;
}


int nomtraj_aug_acados_reset(nomtraj_aug_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+2*NS0+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NH0+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "pi", buffer);
        }
    }
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    free(buffer);
    return 0;
}




int nomtraj_aug_acados_update_params(nomtraj_aug_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 16;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int nomtraj_aug_acados_update_params_sparse(nomtraj_aug_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int nomtraj_aug_acados_set_p_global_and_precompute_dependencies(nomtraj_aug_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, nomtraj_aug_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int nomtraj_aug_acados_solve(nomtraj_aug_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int nomtraj_aug_acados_setup_qp_matrices_and_factorize(nomtraj_aug_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int nomtraj_aug_acados_free(nomtraj_aug_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);
  
  

    // cost
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->cost_y_fun[i]);
        external_function_external_param_casadi_free(&capsule->cost_y_fun_jac_ut_xt[i]);
    }
    free(capsule->cost_y_fun);
    free(capsule->cost_y_fun_jac_ut_xt);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun_jac_ut_xt);

    // constraints



    return 0;
}


void nomtraj_aug_acados_print_stats(nomtraj_aug_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    int stat_n_max = 16;
    if (stat_n > stat_n_max)
    {
        printf("stat_n_max = %d is too small, increase it in the template!\n", stat_n_max);
        exit(1);
    }
    double stat[496];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int nomtraj_aug_acados_custom_update(nomtraj_aug_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *nomtraj_aug_acados_get_nlp_in(nomtraj_aug_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *nomtraj_aug_acados_get_nlp_out(nomtraj_aug_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *nomtraj_aug_acados_get_sens_out(nomtraj_aug_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *nomtraj_aug_acados_get_nlp_solver(nomtraj_aug_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *nomtraj_aug_acados_get_nlp_config(nomtraj_aug_solver_capsule* capsule) { return capsule->nlp_config; }
void *nomtraj_aug_acados_get_nlp_opts(nomtraj_aug_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *nomtraj_aug_acados_get_nlp_dims(nomtraj_aug_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *nomtraj_aug_acados_get_nlp_plan(nomtraj_aug_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
