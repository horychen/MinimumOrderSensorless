#ifndef SYNCHRONOUS_MOTOR_H
#define SYNCHRONOUS_MOTOR_H
#if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

// The function name for the dynamics
#define MACHINE_DYNAMICS SM_Dynamics
// id, iq, omega, theta_d, theta_d_accum, stator flux alpha, stator flux beta
#define NUMBER_OF_STATES 7

#define INDEX_CURRENT_D 0
#define INDEX_CURRENT_Q 1
#define INDEX_OMEGA_ELEC 2
#define INDEX_THETA_D 3 // elec
#define INDEX_THETA_D_ACCUM 4 // mech
#define INDEX_THETA_STATOR_FLUX_ALPHA 5 // in alpha-beta frame
#define INDEX_THETA_STATOR_FLUX_BETA 6 // in alpha-beta frame

#define INDEX_CURRENT_D 0
#define INDEX_CURRENT_Q 1
#define INDEX_OMEGA_ELEC 2
#define INDEX_THETA_D 3 // elec
#define INDEX_THETA_D_ACCUM 4 // mech

struct SynchronousMachineSimulated{

    double timebase;
    double Ts;

    // State
    double x[NUMBER_OF_STATES];
    double x_dot[NUMBER_OF_STATES];

    // Auxliary
    double omg_elec;
    double rpm;
    double rpm_cmd;
    double rpm_deriv_cmd;
    double TLoad;
    double Tem;

    // Electrical parameter
    double R;
    double Ld;
    double Lq;
    double KE; // psi_PM

    // Mechanical parameter
    double npp;
    double npp_inv;
    double Js;
    double mu_m;

    //
    double ual;
    double ube;
    double ial;
    double ibe;

    double ual_c_dist;
    double ube_c_dist;
    double dist_al;
    double dist_be;

    double theta_d; // elec
    double theta_d_accum; // mech
    double theta_d_accum_increment;
    double ud;
    double uq;
    double id;
    double iq;

    // active flux
    double psi_stator[2];
    double psi_active[2];
    double theta_d_activeFlux;

    // phase quantities
    double ua;
    double ub;
    double uc;
    double ia;
    double ib;
    double ic;
    double ea;
    double eb;
    double ec;
};
extern struct SynchronousMachineSimulated ACM;

extern int machine_simulation();
extern void init_Machine();

#endif
#endif