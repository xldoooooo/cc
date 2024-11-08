//
// Created by Wei Luo on 2021/3/18.
//

#include "NMPC_app.h"

int main()
{
    // create a capsule according to the pre-defined model
    NMPC_model_solver_capsule *acados_ocp_capsule = NMPC_model_acados_create_capsule();

    // optimizer
    status = NMPC_model_acados_create(acados_ocp_capsule);

    if (status)
    {
        printf("NMPC_model_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    // sim
    NMPC_model_sim_solver_capsule *sim_capsule = NMPC_model_acados_sim_solver_create_capsule();
    status = NMPC_model_acados_sim_create(sim_capsule);
    sim_config *NMPC_model_sim_config = NMPC_model_acados_get_sim_config(sim_capsule);
    void *NMPC_model_sim_dims = NMPC_model_acados_get_sim_dims(sim_capsule);
    sim_in *NMPC_model_sim_in = NMPC_model_acados_get_sim_in(sim_capsule);
    sim_out *NMPC_model_sim_out = NMPC_model_acados_get_sim_out(sim_capsule);
    if (status)
    {
        printf("acados_create() simulator returned status %d. Exiting.\n", status);
        exit(1);
    }

    // some important structure of ocp
    ocp_nlp_config *nlp_config = NMPC_model_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = NMPC_model_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = NMPC_model_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = NMPC_model_acados_get_nlp_out(acados_ocp_capsule);
    //    ocp_nlp_solver *nlp_solver = NMPC_model_acados_get_nlp_solver(acados_ocp_capsule);
    //    void *nlp_opts = NMPC_model_acados_get_nlp_opts(acados_ocp_capsule);

    N = nlp_dims->N;
    nx = *nlp_dims->nx;
    nu = *nlp_dims->nu;
    printf("time horizion is %d, with state %d and input %d \n", N, nx, nu);

    N_sim = 2000;
    Eigen::MatrixXd simX((N_sim + 1), nx);
    Eigen::MatrixXd simU(N_sim, nu);
    Eigen::VectorXd time_record(N_sim);

    for (int i = 0; i < nx; i++)
        simX(0, i) = x_current[i];
    
    double param[18];
    
    double lbu[] = {-25,-25,-25};
    double ubu[] = {25,25,25};
    // closed loop simulation
    for (int ii = 0; ii < N_sim; ii++)
    {
        auto t_start = std::chrono::high_resolution_clock::now();
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", x_current);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", x_current);
        for (int j = 0; j < N+1; j++){
            param[0] = ii*0.01*0.5 + j*0.01*0.5;
            param[1] = 0;
            param[2] = 0;
            param[3] = 0;
            param[4] = 0;
            param[5] = 0;
            param[6] = 0;
            param[7] = 0;
            param[8] = -0.98*9.81;
            for (int i=9; i<18; i++){
                param[i] = 1;
            }
            NMPC_model_acados_update_params(acados_ocp_capsule, j, param, 18);
            }
        for (int j = 0; j < N; j++){
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, j, "lbu", lbu);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, j, "ubu", ubu);}
        status = NMPC_model_acados_solve(acados_ocp_capsule);

        // get the optimized control input
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);
        for (int i = 0; i < nu; i++)
            simU(ii, i) = u_current[i];

        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        time_record(ii) = elapsed_time_ms;

        // simulation
        sim_in_set(NMPC_model_sim_config, NMPC_model_sim_dims,
                   NMPC_model_sim_in, "u", u_current);
        sim_in_set(NMPC_model_sim_config, NMPC_model_sim_dims,
                   NMPC_model_sim_in, "x", x_current);

        status = NMPC_model_acados_sim_solve(sim_capsule);
        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(NMPC_model_sim_config, NMPC_model_sim_dims,
                    NMPC_model_sim_out, "x", x_current);

        for (int i = 0; i < nx; i++)
            simX(ii + 1, i) = x_current[i];
    }

    // print results
    for (int i = 900; i < N_sim + 1; i++)
        printf("Final result index %d %f, %f, %f \n", i, simX(i, 0), simX(i, 1), simX(i, 2));
    printf("average estimation time %f ms \n", time_record.mean());
    printf("max estimation time %f ms \n", time_record.maxCoeff());
    printf("min estimation time %f ms \n", time_record.minCoeff());
    return status;
}