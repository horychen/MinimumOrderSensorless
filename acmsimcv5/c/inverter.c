#include "ACMSim.h"

#if INVERTER_NONLINEARITY
    void InverterNonlinearity_ExperimentalLUT_Indexed(double ual, double ube, double ial, double ibe){

        double ualbe_dist[2];
        /* 查表法-逆变器建模 */
        get_distorted_voltage_via_LUT_indexed( ial, ibe, ualbe_dist );

        // 我们把逆变器产生的电压视作压降，所以要从给定的电压中减去
        UAL_C_DIST = ual - ualbe_dist[0];
        UBE_C_DIST = ube - ualbe_dist[1];

        DIST_AL = ualbe_dist[0];
        DIST_BE = ualbe_dist[1];
    }
    void InverterNonlinearity_ExperimentalLUT(double ual, double ube, double ial, double ibe){
        // #define LENGTH_OF_LUT 21 // 80 V?
        // double lut_current[LENGTH_OF_LUT] = {-4.19999, -3.77999, -3.36001, -2.94002, -2.51999, -2.10004, -1.68004, -1.26002, -0.840052, -0.419948, 5.88754e-06, 0.420032, 0.839998, 1.26003, 1.67998, 2.10009, 2.51996, 2.87326, 3.36001, 3.78002, 4.2};
        // double lut_voltage[LENGTH_OF_LUT] = {-5.20719, -5.2079, -5.18934, -5.15954, -5.11637, -5.04723, -4.93463, -4.76367, -4.42522, -3.46825, 0.317444, 3.75588, 4.55737, 4.87773, 5.04459, 5.15468, 5.22904, 5.33942, 5.25929, 5.28171, 5.30045};

        #define LENGTH_OF_LUT  40 // 180 V
        REAL lut_current[LENGTH_OF_LUT] = {-3.14998, -2.99249, -2.83499, -2.67749, -2.51999, -2.36252, -2.205, -2.04749, -1.89, -1.7325, -1.57501, -1.41752, -1.26, -1.10251, -0.944981, -0.787495, -0.629978, -0.472486, -0.315002, -0.157496, 0.157467, 0.314993, 0.472519, 0.629995, 0.787465, 0.945158, 1.1025, 1.25999, 1.41751, 1.57502, 1.73252, 1.89002, 2.0475, 2.205, 2.36248, 2.51997, 2.6775, 2.83498, 2.99249, 3.14999};
        REAL lut_voltage[LENGTH_OF_LUT] = {-6.6419, -6.6724, -6.70256, -6.7277, -6.74972, -6.76818, -6.78277, -6.79267, -6.79659, -6.78879, -6.78193, -6.75717, -6.71881, -6.6574, -6.56772, -6.4199, -6.18427, -5.78867, -4.95014, -2.78283, 1.9668, 4.68597, 5.68227, 6.13478, 6.39548, 6.5496, 6.66489, 6.73335, 6.77611, 6.79843, 6.81645, 6.81862, 6.81904, 6.80278, 6.79212, 6.77605, 6.75211, 6.72556, 6.69713, 6.67054};

        double ualbe_dist[2];
        /* 查表法-逆变器建模 */
        get_distorted_voltage_via_LUT( ual, ube, ial, ibe, ualbe_dist, lut_voltage, lut_current, LENGTH_OF_LUT);

        // 我们把逆变器产生的电压视作压降，所以要从给定的电压中减去
        UAL_C_DIST = ual - ualbe_dist[0];
        UBE_C_DIST = ube - ualbe_dist[1];

        DIST_AL = ualbe_dist[0];
        DIST_BE = ualbe_dist[1];
    }
    void InverterNonlinearity_ExperimentalSigmoid(double ual, double ube, double ial, double ibe){

        double ualbe_dist[2];
        /* 拟合法-逆变器建模 */
        get_distorted_voltage_via_CurveFitting( ual, ube, ial, ibe, ualbe_dist);

        // 我们把逆变器产生的电压视作压降，所以要从给定的电压中减去
        UAL_C_DIST = ual - ualbe_dist[0];
        UBE_C_DIST = ube - ualbe_dist[1];

        DIST_AL = ualbe_dist[0];
        DIST_BE = ualbe_dist[1];
    }
    void InverterNonlinearity_SKSul96(double ual, double ube, double ial, double ibe){
        double ua,ub,uc;
        double ia,ib,ic;
        double Udist;
        double TM;
        double Rce=0.04958, Rdiode=0.05618;

        TM    = _Toff - _Ton - _Tdead + _Tcomp; // Sul1996
        Udist = (_Udc*TM*CL_TS_INVERSE - _Vce0 - _Vd0) / 6.0; // Udist = (_Udc*TM/1e-4 - _Vce0 - _Vd0) / 6.0;
        /* 我规定Udist为正值 */
        Udist = fabs(Udist);
        // Udist = (_Udc*TM*TS_INVERSE) / 6.0;
        // Udist = 0.0;
        static int bool_printed = FALSE;
        if(bool_printed==FALSE){
            printf("\t[inverter.c] Vsat = %g V\n", INV.Vsat);
            bool_printed=TRUE;
        }

        ia = 1 * (       ial                              );
        ib = 1 * (-0.5 * ial - SIN_DASH_2PI_SLASH_3 * ibe );
        ic = 1 * (-0.5 * ial - SIN_2PI_SLASH_3      * ibe );

        if(FALSE){
            /* compute in abc frame (in Amplitude Invariant Transformation) */
            ua = 1 * (       ual                              );
            ub = 1 * (-0.5 * ual - SIN_DASH_2PI_SLASH_3 * ube );
            uc = 1 * (-0.5 * ual - SIN_2PI_SLASH_3      * ube );
            REAL ua_dist, ub_dist, uc_dist;
            ua_dist = Udist * (2*sign(ia) - sign(ib) - sign(ic)) + 0 * 0.5*(Rce+Rdiode)*ia;
            ub_dist = Udist * (2*sign(ib) - sign(ic) - sign(ia)) + 0 * 0.5*(Rce+Rdiode)*ib;
            uc_dist = Udist * (2*sign(ic) - sign(ia) - sign(ib)) + 0 * 0.5*(Rce+Rdiode)*ic;
            DIST_AL = 2.0/3.0 *             (ua_dist - 0.5*ub_dist - 0.5*uc_dist);
            DIST_BE = 2.0/3.0 * sqrt(3)/2 * (              ub_dist -     uc_dist);
            // 我们把逆变器产生的电压视作压降，所以要从给定的电压中减去
            ua -= ua_dist;
            ub -= ub_dist;
            uc -= uc_dist;
            UAL_C_DIST = 2.0/3.0 *             (ua - 0.5*ub - 0.5*uc); // sqrt(2/3.)
            UBE_C_DIST = 2.0/3.0 * sqrt(3)/2 * (         ub -     uc); // sqrt(2/3.)*sin(2*pi/3) = sqrt(2/3.)*(sqrt(3)/2)            
        }else{
            /* directly compute in alpha-beta frame (Do note doing this injects a zero-sqeuence voltage!!!) */
            // CHECK the sign of the distortion voltage!
            DIST_AL =         Udist*(2*sign(ia) - sign(ib) - sign(ic)) + 0 * 0.5*(Rce+Rdiode)*ial;
            DIST_BE = sqrt(3)*Udist*(             sign(ib) - sign(ic)) + 0 * 0.5*(Rce+Rdiode)*ibe;
            // Sul把Udist视为补偿的电压（假定上升下降时间都已经知道了而且是“补偿”上去的）
            UAL_C_DIST = ual - DIST_AL;
            UBE_C_DIST = ube - DIST_BE;
        }
    }
#endif

// 逆变器建模
double tri_stage;
void inverter_model(){

    // amplitude-invariant to power-invariant
    // 考虑控制器和电机所用Clarke变换不同导致的系数变化

    // 根据给定电压CTRL.O->uab_cmd[0]和实际的电机电流ACM.ial，计算畸变的逆变器输出电压ACM.ual。
    #if INVERTER_NONLINEARITY == 4
        InverterNonlinearity_ExperimentalLUT_Indexed( 
                                              CTRL.O->uab_cmd_to_inverter[0], \
                                              CTRL.O->uab_cmd_to_inverter[1], \
                                              ACM.ial, \
                                              ACM.ibe);
        ACM.ual = UAL_C_DIST;
        ACM.ube = UBE_C_DIST;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - CTRL.O->uab_cmd[0];
        // DIST_BE = ACM.ube - CTRL.O->uab_cmd[1];
    #elif INVERTER_NONLINEARITY == 3
        InverterNonlinearity_ExperimentalLUT( CTRL.O->uab_cmd_to_inverter[0], \
                                              CTRL.O->uab_cmd_to_inverter[1], \
                                              ACM.ial, \
                                              ACM.ibe);
        ACM.ual = UAL_C_DIST;
        ACM.ube = UBE_C_DIST;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - CTRL.O->uab_cmd[0];
        // DIST_BE = ACM.ube - CTRL.O->uab_cmd[1];
    #elif INVERTER_NONLINEARITY == 2
        InverterNonlinearity_ExperimentalSigmoid( CTRL.O->uab_cmd_to_inverter[0], \
                                              CTRL.O->uab_cmd_to_inverter[1], \
                                              ACM.ial, \
                                              ACM.ibe);
        ACM.ual = UAL_C_DIST;
        ACM.ube = UBE_C_DIST;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - CTRL.O->uab_cmd[0];
        // DIST_BE = ACM.ube - CTRL.O->uab_cmd[1];
    #elif INVERTER_NONLINEARITY == 1
        InverterNonlinearity_SKSul96( CTRL.O->uab_cmd_to_inverter[0], \
                                      CTRL.O->uab_cmd_to_inverter[1], \
                                      ACM.ial, \
                                      ACM.ibe);
        ACM.ual = UAL_C_DIST;
        ACM.ube = UBE_C_DIST;
        // 计算畸变电压 = 实际电压 - 给定电压 （仅用于可视化用途）
        // DIST_AL = ACM.ual - CTRL.O->uab_cmd[0];
        // DIST_BE = ACM.ube - CTRL.O->uab_cmd[1];
    #else
                            // 考虑控制器和电机所用Clarke变换不同导致的系数变化
        ACM.ual = CTRL.O->uab_cmd_to_inverter[0]; //*sqrt(CLARKE_TRANS_TORQUE_GAIN); *AMPL2POW
        ACM.ube = CTRL.O->uab_cmd_to_inverter[1]; //*sqrt(CLARKE_TRANS_TORQUE_GAIN); *AMPL2POW
    #endif

    #if MACHINE_TYPE == 2
        // 永磁电机仿真的输入电压是在dq系下的，所以要把alpha-beta系下的电压经过Park变换变为dq系下的电压。
        ACM.ud = AB2M(ACM.ual, ACM.ube, cos(ACM.theta_d), sin(ACM.theta_d));
        ACM.uq = AB2T(ACM.ual, ACM.ube, cos(ACM.theta_d), sin(ACM.theta_d));
    #endif


    /* PWM Model */
    // // triangular carrier wave
    // REAL carrier_freq = CL_TS_INVERSE; 
    // REAL ceiling = 10;
    // tri_stage = (int32)(4*ceiling*ACM.timebase*carrier_freq) % (int32)(4*ceiling); // https://pubs.opengroup.org/onlinepubs/9699919799/functions/fmod.html
    // // if (tri_stage<ceiling)
    // //     tri_stage = tri_stage;
    // // else if (tri_stage<(3*ceiling) )
    // //     tri_stage = 2*ceiling-tri_stage;
    // // else
    // //     tri_stage = tri_stage-4*ceiling;
}


/* --------------------------这里以上的都是没用的，下面的是从 pmsm_controller.c 挪过来的公用逆变器死区电压辨识&补偿代码 */

