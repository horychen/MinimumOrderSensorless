#include "ACMSim.h"

double load_model(){
    static double Tload = 0.0;

    // Tload = 0.0001 * ACM.rpm + 0.002 * cos(3*ACM.x[3]); // this load causes zero speed oscillation in controller 
    // Tload = 0.005*sign(ACM.rpm);
    // Tload = LOAD_TORQUE*sign(ACM.rpm) + 0*sin(2*M_PI*1*ACM.timebase);

    static int load_state = 0;

    static double dc_part = LOAD_TORQUE;
    static double viscous_part = 0.0;

    if(FALSE){
        if(CTRL.timebase>40.0){
            dc_part = LOAD_TORQUE*2;
        }else if(CTRL.timebase>35.0){
            dc_part = 0.2*LOAD_TORQUE;
        }else if(CTRL.timebase>30.0){
            dc_part = 0.6*LOAD_TORQUE;
        }else if(CTRL.timebase>25.0){
            dc_part = LOAD_TORQUE;
        }else if(CTRL.timebase>12.0){
            dc_part = LOAD_TORQUE*2;
        }else if(CTRL.timebase>2.0){
            if(fabs(ACM.rpm)>10 && load_state==0){
                load_state = 1;
                dc_part = LOAD_TORQUE;
                // dc_part = LOAD_TORQUE*sign(ACM.rpm);
                // dc_part = - LOAD_TORQUE*sign(ACM.rpm);
            }else if(fabs(ACM.rpm)<-10 && load_state==1){
                load_state = 0;
                // TODO this is not reached?!
                dc_part = 99999* LOAD_TORQUE*sign(ACM.rpm);
            }
        }        
    }

    viscous_part = VISCOUS_COEFF*ACM.rpm*RPM_2_ELEC_RAD_PER_SEC;
    Tload = dc_part + viscous_part;

    // Tload = -sign(ACM.rpm) * Tload;

    #if ENABLE_COMMISSIONING
        Tload = 0.0;
    #endif

    // Tload = 2;

    // printf("%g, %g, %g\n", Tload, LOAD_TORQUE*sign(ACM.rpm), VISCOUS_COEFF*ACM.rpm*RPM_2_ELEC_RAD_PER_SEC);

    // Tload = 0.0;
        // EPA-2019-0568.R1
        // double friction;
        // if(fabs(ACM.rpm*RPM_2_ELEC_RAD_PER_SEC)<0.08){
        //     if(ACM.rpm>0){
        //         friction = 0.0125 / 0.08 * ACM.rpm*RPM_2_ELEC_RAD_PER_SEC; // <- using elec rad/s is wrong
        //     }else{
        //         friction = - 0.0125 / 0.08 * ACM.rpm*RPM_2_ELEC_RAD_PER_SEC;
        //     }
        // }else{
        //     if(ACM.rpm>0){
        //         friction = 0.0025 + 0.0025 * ACM.rpm*RPM_2_ELEC_RAD_PER_SEC;
        //     }else{
        //         friction = - 0.0025 - 0.0025 * ACM.rpm*RPM_2_ELEC_RAD_PER_SEC;
        //     }
        // }
        // ACM.Tload += friction;

        // 齿轮箱背隙建模(在背隙形程内，负载为零，否则突然加载)
    return Tload;
}
