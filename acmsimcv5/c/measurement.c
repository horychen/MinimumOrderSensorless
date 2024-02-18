#include "ACMSim.h"
#define CURRENT_OFFSET_A ( 0.1*0)
#define CURRENT_OFFSET_B (-0.02*0)



void measurement(){
    // 本函数每隔采样时间 CL_TS 执行一次

    // 下面出现的US_C，IS_C等，都是全局的宏变量，方便在不同的.c文件内共享。

    // 电压测量
    US_P(0) = CTRL.O->uab_cmd[0]; // 后缀_P表示上一步的电压，P = Previous
    US_P(1) = CTRL.O->uab_cmd[1]; // 后缀_C表示当前步的电压，C = Current
    US_C(0) = CTRL.O->uab_cmd[0]; // 后缀_P表示上一步的电压，P = Previous
    US_C(1) = CTRL.O->uab_cmd[1]; // 后缀_C表示当前步的电压，C = Current

    // 电流测量存在噪声，只会影响反馈控制，实际的扰动可能是逆变器引入的，见逆变器建模
        // power-invariant to amplitude-invariant via Clarke transformation???
    // REAL sqrt_2slash3 = sqrt(2.0/3.0);
    // REAL ia = sqrt_2slash3 * ACM.ial                              + 0*2*5e-2*RANDOM;
    // REAL ib = sqrt_2slash3 * (-0.5*ACM.ial + 0.5*sqrt(3)*ACM.ibe) + 0*2*5e-2*RANDOM;
    // REAL ic = sqrt_2slash3 * (-0.5*ACM.ial - 0.5*sqrt(3)*ACM.ibe) + 0*2*5e-2*RANDOM;
    // IS_C(0) = 2.0/3.0 * (ia - 0.5*ib - 0.5*ic);
    // IS_C(1) = 2.0/3.0 * 0.5*sqrt(3.0) * (ib - ic);
    IS_C(0)        = ACM.ial + CURRENT_OFFSET_A;
    IS_C(1)        = ACM.ibe + 1*CURRENT_OFFSET_B; // TODO: what makes beta current so special that adding offset here unstablizes the control system
    CTRL.I->iab[0] = ACM.ial + CURRENT_OFFSET_A;
    CTRL.I->iab[1] = ACM.ibe + 1*CURRENT_OFFSET_B;

    // 转速和位置传感器建模
        // sensors();
        int64 ActualPosInCnt = (ACM.theta_d_accum / (2*M_PI) * SYSTEM_QEP_PULSES_PER_REV);
        int64 the_sign = sign_integer(ActualPosInCnt);
        Uint32 QPOSCNT = the_sign * (the_sign * ActualPosInCnt) % (int64)SYSTEM_QEP_PULSES_PER_REV; //EQep1Regs.QPOSCNT;
        ENC.rpm          = PostionSpeedMeasurement_MovingAvergage(QPOSCNT, CTRL.enc);
        ENC.omg_elec     = ENC.rpm * RPM_2_ELEC_RAD_PER_SEC; // 机械转速（单位：RPM）-> 电气角速度（单位：elec.rad/s)
        ENC.theta_d_elec = ENC.theta_d__state;

    #if MACHINE_TYPE == PM_SYNCHRONOUS_MACHINE

        #if USE_QEP_RAW
            /*（编码器反馈，含滑动平均）*/
            // 转速
            CTRL.I->omg_elec              = ENC.omg_elec;
            CTRL.I->rpm = CTRL.I->omg_elec*ELEC_RAD_PER_SEC_2_RPM;
            // 位置
            CTRL.I->theta_d_elec_previous = CTRL.I->theta_d_elec;
            CTRL.I->theta_d_elec          = ENC.theta_d_elec;

        #else
            /*（实际反馈，实验中不可能）*/
            // 转速
            CTRL.I->omg_elec              = ACM.omg_elec;
            CTRL.I->rpm = CTRL.I->omg_elec*ELEC_RAD_PER_SEC_2_RPM;
            // 位置
            CTRL.I->theta_d_elec_previous = CTRL.I->theta_d_elec;
            CTRL.I->theta_d_elec          = ACM.theta_d; // + 30.0/180*M_PI;
        #endif

        //（霍尔反馈）
        // CTRL.I->omg_elec     = EXP.omg_elec_hall;
        // CTRL.S->theta_D_elec__fb = EXP.theta_d_hall;

    #elif MACHINE_TYPE == INDUCTION_MACHINE_CLASSIC_MODEL || MACHINE_TYPE == INDUCTION_MACHINE_FLUX_ONLY_MODEL
        CTRL.I->omg_elec = ACM.omg_elec;
        CTRL.I->rpm = CTRL.I->omg_elec*ELEC_RAD_PER_SEC_2_RPM;
    #endif
}



struct HallSensor HALL;
void sensors(){
    #define USE_EQEP FALSE
    #define USE_HALL FALSE
    #if USE_EQEP == TRUE
        #define NUMBER_OF_CURRENT_LOOP (1*SPEED_LOOP_CEILING)
        #define QEP_SPEED_RESOLUTION_RPM (60.0 / (SYSTEM_QEP_PULSES_PER_REV * (NUMBER_OF_CURRENT_LOOP*CL_TS)))

        #define BOOL_USE_ABOSOLUTE_POSITION TRUE

        // TODO 这里的建模没有考虑初始d轴的偏置角！

        // TODO 反转的时候qep的转速好像是错的！

        static int sensor_vc_count = 1;
        if(sensor_vc_count++ == NUMBER_OF_CURRENT_LOOP){
            sensor_vc_count = 1;

            // qep.theta_d_mech += qep.QPOSLAT;

            // qep.difference_in_cnt = qep.QPOSLAT/(2*M_PI) * SYSTEM_QEP_PULSES_PER_REV; // 这句建模的是纯粹的增量
            // 增量式编码器测量的其实不是纯粹的增量，更像是是一个累加器，然后round off，所以用qep.QPOSLAT控制会不稳定

            /* Position */
            if(BOOL_USE_ABOSOLUTE_POSITION){
                // 使用ActualPosInCnt的方案是错误的，因为一开始转子位置可能不为零，QEP能知道的其实只是增量。
                qep.ActualPosInCnt_Previous = qep.ActualPosInCnt;
                qep.RoundedPosInCnt_Previous = qep.RoundedPosInCnt;                    

                qep.ActualPosInCnt = (ACM.theta_d_accum / (2*M_PI) * SYSTEM_QEP_PULSES_PER_REV);
                qep.number_of_revolution = qep.ActualPosInCnt / SYSTEM_QEP_PULSES_PER_REV;
                qep.theta_d_accum  = (         qep.ActualPosInCnt*SYSTEM_QEP_REV_PER_PULSE)*2*M_PI;

                int64 the_sign = sign_integer(qep.ActualPosInCnt);
                qep.RoundedPosInCnt = the_sign * (the_sign * qep.ActualPosInCnt) % (int64)SYSTEM_QEP_PULSES_PER_REV;
                qep.theta_d_mech = ( qep.RoundedPosInCnt * SYSTEM_QEP_REV_PER_PULSE ) *2*M_PI \
                                   + 0*1*15.0 / 180.0 * M_PI; // IPD Offset?
            }else{
                qep.theta_d_mech += qep.difference_in_cnt * SYSTEM_QEP_REV_PER_PULSE *(2*M_PI);
                // qep.theta_d_mech += qep.QPOSLAT/(2*M_PI) * SYSTEM_QEP_PULSES_PER_REV * SYSTEM_QEP_REV_PER_PULSE *(2*M_PI);
            }
            qep.QPOSLAT = 0;
            // Directly Calculate
            // Directly Calculate
            // Directly Calculate
            // qep.theta_d = ACM.npp*qep.theta_d_mech + CTRLQEP.theta_d_offset;
            // while(qep.theta_d> M_PI) qep.theta_d -= 2*M_PI;
            // while(qep.theta_d<-M_PI) qep.theta_d += 2*M_PI;

            if(qep.ActualPosInCnt==-1){
                int a = 0;
                a++;
            }

            /* Speed */
            // if(CTRL.timebase>0.0008){ //2*CL_TS
            if(CTRL.timebase>CL_TS){
                qep.moving_average[3] = qep.moving_average[2];
                qep.moving_average[2] = qep.moving_average[1];
                qep.moving_average[1] = qep.moving_average[0];
                if(BOOL_USE_ABOSOLUTE_POSITION){
                    qep.difference_in_cnt = (int32)qep.RoundedPosInCnt - (int32)qep.RoundedPosInCnt_Previous; // 无符号数相减需要格外小心

                    // 增量超过一半则认为是Cnt被清零了
                    if( qep.difference_in_cnt < -0.5*SYSTEM_QEP_PULSES_PER_REV){
                        qep.difference_in_cnt =  (int32)SYSTEM_QEP_PULSES_PER_REV + qep.RoundedPosInCnt - qep.RoundedPosInCnt_Previous;
                    }else if ( qep.difference_in_cnt > 0.5*SYSTEM_QEP_PULSES_PER_REV){
                        qep.difference_in_cnt = -(int32)SYSTEM_QEP_PULSES_PER_REV + qep.RoundedPosInCnt - qep.RoundedPosInCnt_Previous;
                    }
                }
            }

            // Incrementally Accumulated
            // Incrementally Accumulated
            // Incrementally Accumulated
            CTRL.I->encoder_cnt += qep.difference_in_cnt;
            if(CTRL.I->encoder_cnt > SYSTEM_QEP_PULSES_PER_REV){CTRL.I->encoder_cnt -= SYSTEM_QEP_PULSES_PER_REV;}
            if(CTRL.I->encoder_cnt < 0)                        {CTRL.I->encoder_cnt += SYSTEM_QEP_PULSES_PER_REV;}

            CTRLQEP.theta_d__state += qep.difference_in_cnt*SYSTEM_QEP_REV_PER_PULSE * (2*M_PI) * ACM.npp;
            if(CTRLQEP.theta_d__state> M_PI) CTRLQEP.theta_d__state -= 2*M_PI;
            if(CTRLQEP.theta_d__state<-M_PI) CTRLQEP.theta_d__state += 2*M_PI;
            qep.theta_d = CTRLQEP.theta_d__state + CTRLQEP.theta_d_offset;
            while(qep.theta_d> M_PI) qep.theta_d -= 2*M_PI;
            while(qep.theta_d<-M_PI) qep.theta_d += 2*M_PI;

            qep.moving_average[0] = qep.difference_in_cnt * QEP_SPEED_RESOLUTION_RPM; // [rpm]

            // qep.moving_average[0] = (Uint32)(ACM.theta_d_accum_increment/ (2*M_PI) * SYSTEM_QEP_PULSES_PER_REV) * QEP_SPEED_RESOLUTION_RPM; // [rpm]

            // this gives wrong results when SYSTEM_QEP_PULSES_PER_REV is larger than 1e3, e.g., 1e4.
            // qep.moving_average[0] = (qep.ActualPosInCnt - qep.ActualPosInCnt_Previous) * QEP_SPEED_RESOLUTION_RPM; // [rpm]

            // no filter
            qep.omg_elec = qep.moving_average[0] * RPM_2_ELEC_RAD_PER_SEC;
            // qep.omg_elec = (ACM.theta_d_accum - qep.ActualPosInCnt_Previous) *ACM.npp_inv;

            // moving average filter
            // qep.omg_elec = 0.25*(qep.moving_average[3]+qep.moving_average[2]+qep.moving_average[1]+qep.moving_average[0]) * RPM_2_ELEC_RAD_PER_SEC;

            qep.omg_mech = qep.omg_elec * ACM.npp_inv;

            /* Position (Option 2) */
            // qep.theta_d += (NUMBER_OF_CURRENT_LOOP*CL_TS) * qep.omg_elec;
            // while(qep.theta_d> M_PI) qep.theta_d -= 2*M_PI;
            // while(qep.theta_d<-M_PI) qep.theta_d += 2*M_PI;
        }
    #endif
    #if USE_HALL == TRUE

        EXP.theta_d  = ACM.theta_d; // + 30.0/180*M_PI;
        EXP.omg_elec = ACM.omg_elec;
        EXP.omg_mech = EXP.omg_elec * EXP.npp_inv;

        // 霍尔传感器检测
        EXP.hallABC = hall_sensor(ACM.theta_d/M_PI*180.0);
        hall_resolver(EXP.hallABC, &EXP.theta_d_hall, &EXP.omg_elec_hall);

        // 测量转子d轴位置限幅
        if(EXP.theta_d_hall > M_PI){
            EXP.theta_d_hall -= 2*M_PI;
        }else if(EXP.theta_d_hall < -M_PI){
            EXP.theta_d_hall += 2*M_PI;
        }

        // // if(CTRL.timebase>19){
        //     EXP.theta_d  = EXP.theta_d_hall;
        //     EXP.omg_elec = EXP.omg_elec_hall;
        // // }
    #endif
}
void HALL_init(){

    HALL.acceleration_avg = 0.0;
    HALL.last_omg_elec_hall = 0.0;

    HALL.omg_elec_hall = 0.0;
    HALL.omg_mech_hall = 0.0;
    HALL.theta_d_hall = 0.0;

    HALL.hallA = 0;
    HALL.hallB = 1;
    HALL.hallC = 1;
    HALL.hallABC = 0x3;
    HALL.last_hallABC = 0x1;
    HALL.speed_direction = +1;
    HALL.bool_position_correction_pending = FALSE;

    HALL.timebase = 0.0;
    HALL.timeStamp = 0.0;
    HALL.timeDifference = 0.0;
    HALL.timeDifferenceStamp = 100000.0;
}

