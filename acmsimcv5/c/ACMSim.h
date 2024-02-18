#ifndef ACMSIM_H
#define ACMSIM_H

/* Standard lib */
// #include <stdbool.h> // bool for _Bool and true for 1
#include <stdio.h> // printf #include <stdbool.h> // bool for _Bool and true for 1
// #include <process.h>//reqd. for system function prototype
#include <conio.h> // for clrscr, and getch()
#include "stdlib.h" // for rand()
#include "math.h"
#include "time.h"
// #include <unistd.h> // getcwd
    // char cwd[1024]; // current working directory
    // printf(getcwd(cwd, sizeof(cwd)));

/* This is not Experiment */
#define PC_SIMULATION TRUE

/* Data Types consistent with iSMC */
#ifndef DSP28_DATA_TYPES
#define DSP28_DATA_TYPES
typedef int int16;
typedef unsigned int Uint16;
typedef long int32;
typedef unsigned long Uint32;
typedef long long int64;
typedef unsigned long long Uint64;
typedef float float32;
typedef long double float64;
#endif
typedef float32 REAL;

/* General Constants */
#define PHASE_NUMBER 3 // three phase machine
#ifndef BOOL
#define BOOL int
#endif
#ifndef TRUE
#define TRUE  (1)
#endif
#ifndef FALSE
#define FALSE (0)
#endif
#define ONE_OVER_2PI          0.15915494309189535 // 1/(2*pi)
#define ONE_OVER_60           0.01666666666666667
#define TWO_PI_OVER_3         2.0943951023931953
#define SIN_2PI_SLASH_3       0.86602540378443871 // sin(2*pi/3)
#define SIN_DASH_2PI_SLASH_3 -0.86602540378443871 // sin(-2*pi/3)
#define SQRT_2_SLASH_3        0.81649658092772603 // sqrt(2.0/3.0)
#define abs                   use_fabs_instead_or_you_will_regret
#define ELEC_RAD_PER_SEC_2_RPM ( 60.0*ONE_OVER_2PI*CTRL.motor->npp_inv )
// #define ELEC_RAD_PER_SEC_2_RPM ( 60.0/(2*M_PI*CTRL.motor->npp) )
#define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*CTRL.motor->npp)*ONE_OVER_60 )
// #define RPM_2_ELEC_RAD_PER_SEC ( (2*M_PI*CTRL.motor->npp)/60.0 )
#define M_PI_OVER_180   0.017453292519943295
// New convention
#define CONST_PI_OVER_180 (0.0174533)
#define CONST_180_OVER_PI (57.2958)
#define CONST_1_OVER_SQRT3 (0.57735)

/* Motor Control Related Utility Macros */
#define CLARKE_TRANS_TORQUE_GAIN (1.5) // consistent with experiment
#define CLARKE_TRANS_TORQUE_GAIN_INVERSE (0.666666667)
#define POW2AMPL (0.816496581) // = 1/sqrt(1.5) power-invariant to aplitude-invariant (the dqn vector becomes shorter to have the same length as the abc vector)
#define AMPL2POW (1.22474487)


// 模拟测量环节，可以在此为测量电机添加噪声、温飘等，也可以在此实现类似光电编码器的转速测量环节。
#define RANDOM ( ((double) rand() / (RAND_MAX))*2 - 1 ) // [-1, 1]

/* Macro for Park transformation*/
#define AB2M(A, B, COS, SIN)  ( (A)*COS  + (B)*SIN )
#define AB2T(A, B, COS, SIN)  ( (A)*-SIN + (B)*COS )
#define MT2A(M, T, COS, SIN)  ( (M)*COS - (T)*SIN )
#define MT2B(M, T, COS, SIN)  ( (M)*SIN + (T)*COS )

/* Macro for two-phase Amplitude-invariant Clarke transformation*/
#define UV2A_AI(U, V) ( U )
#define UV2B_AI(U, V) ( (U + 2*(V)) * CONST_1_OVER_SQRT3 )
#define AB2U_AI(A, B) ( ( A ) )
#define AB2V_AI(A, B) ( ( (A)*-0.5 + (B)*0.866 ) )
#define AB2W_AI(A, B) ( ( (A)*-0.5 + (B)*-0.866 ) )

/* Macro for Power-invariant inverse Clarke transformation */
#define AB2U_PI(A, B) ( 0.816496580927726 * ( A ) )
#define AB2V_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*0.8660254037844387 ) )
#define AB2W_PI(A, B) ( 0.816496580927726 * ( A*-0.5 + B*-0.8660254037844385 ) )

/* Header files in one place*/
// Everthing that is configurable is in here
#include "ACMConfig.h"
// Shared Flux Estimators
#include "shared_flux_estimator.h"
// PID Regulator
#include "pid_regulator.h"
// SM
#include "synchronous_motor.h"
#include "pmsm_comm.h"
#include "pmsm_controller.h"
#include "pmsm_observer.h"
// IM
#include "induction_motor.h"
#include "im_controller.h"
#include "im_observer.h"
// Sensor, Inverter and Load
#include "measurement.h"
#include "inverter.h"
#include "load.h"
// Sweep Frequency Test
#include "sweep_frequency.h"

// Header for global_variabels_definition.c
struct GlobalWatch
{
    #if MACHINE_TYPE % 10 == 1
        // induction motor
        double offset[2];
        double offset_compensation[2];
        double psi_2[2];
    #elif MACHINE_TYPE % 10 == 2
        // PM motor
    #endif
};
extern struct GlobalWatch watch;



/* Declaration of Utility Function defined in utility.c */
// For main.c
void write_header_to_file(FILE *fw);
void write_data_to_file(FILE *fw);
void print_info();
// General ones
int isNumber(double x);
double sign(double x);
int64 sign_integer(int64 x);
double fabs(double x);
// low pass filter
REAL _lpf(REAL x, REAL y_tminus1, REAL time_const_inv);
extern REAL one_over_six;
REAL PostionSpeedMeasurement_MovingAvergage(Uint32 QPOSCNT, st_enc *p_enc);
double difference_between_two_angles(double first, double second);

#if PC_SIMULATION == TRUE
    #define SYSTEM_HALF_PWM_MAX_COUNT 10000

    // 写变量名到文件
#define DATA_FORMAT "%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g,%g\n"
#define DATA_LABELS "ACM.TLoad,marino.xTL,marino.xOmg*ELEC_RAD_PER_SEC_2_RPM,ACM.rpm,ACM.rpm_cmd,watch.offset[0],watch.offset_compensation[0],ACM.psi_Amu - watch.psi_2[0],ACM.psi_Amu,watch.psi_2[0],FE.htz.psi_aster_max,-FE.htz.psi_aster_max,marino.e_psi_Dmu,marino.e_psi_Qmu,ACM.rpm - marino.xOmg*ELEC_RAD_PER_SEC_2_RPM,CTRL.S->omega_syn,FE.htz.field_speed_est,marino.xAlpha,FE.htz.sat_max_time[0],FE.htz.sat_max_time[1],FE.htz.sat_min_time[0],FE.htz.sat_min_time[1],FE.htz.gain_off,FE.htz.sat_time_offset[0],FE.htz.sat_time_offset[1],FE.htz.maximum_of_sat_max_time[0] - FE.htz.maximum_of_sat_min_time[0],FE.htz.maximum_of_sat_max_time[0],FE.htz.maximum_of_sat_min_time[0],CTRL.I->idq_cmd[0],ACM.iMs,CTRL.I->idq_cmd[1],ACM.iTs,CTRL.O->udq_cmd[0],CTRL.O->udq_cmd[1],IS_C(0),IS_C(1),FE.htz.extra_limit\n"
#define DATA_DETAILS ACM.TLoad,marino.xTL,marino.xOmg*ELEC_RAD_PER_SEC_2_RPM,ACM.rpm,ACM.rpm_cmd,watch.offset[0],watch.offset_compensation[0],ACM.psi_Amu - watch.psi_2[0],ACM.psi_Amu,watch.psi_2[0],FE.htz.psi_aster_max,-FE.htz.psi_aster_max,marino.e_psi_Dmu,marino.e_psi_Qmu,ACM.rpm - marino.xOmg*ELEC_RAD_PER_SEC_2_RPM,CTRL.S->omega_syn,FE.htz.field_speed_est,marino.xAlpha,FE.htz.sat_max_time[0],FE.htz.sat_max_time[1],FE.htz.sat_min_time[0],FE.htz.sat_min_time[1],FE.htz.gain_off,FE.htz.sat_time_offset[0],FE.htz.sat_time_offset[1],FE.htz.maximum_of_sat_max_time[0] - FE.htz.maximum_of_sat_min_time[0],FE.htz.maximum_of_sat_max_time[0],FE.htz.maximum_of_sat_min_time[0],CTRL.I->idq_cmd[0],ACM.iMs,CTRL.I->idq_cmd[1],ACM.iTs,CTRL.O->udq_cmd[0],CTRL.O->udq_cmd[1],IS_C(0),IS_C(1),FE.htz.extra_limit
#endif


#endif
