
#ifndef MEASUREMENT_H
#define MEASUREMENT_H

extern void measurement();


// Hall Sensor
struct HallSensor{

    REAL omg_elec_hall;
    REAL omg_mech_hall;
    REAL theta_d_hall;

    REAL last_omg_elec_hall;
    REAL acceleration_avg;

    int16 hallA;
    int16 hallB;
    int16 hallC;
    int16 hallABC;
    int16 last_hallABC;
    int16 speed_direction;
    int16 bool_position_correction_pending;

    REAL timebase;
    REAL timeStamp;
    REAL timeDifference;
    REAL timeDifferenceStamp;



    /* In hall_resolver() */

    REAL hall_speedRad; // 电气转速输出
    REAL hall_speedRad_pre; // 用于低通滤波（无效）
    REAL hall_speedRad_tmp; // 这个变量其实是不必要的，等于60°/时间

    Uint32 hall_speed_cnt;
    Uint32 hall_speed_cnt_pre; // 作为扇区内计数的一个指标，标志着电机转速是否慢于上一个扇区。只能判断电机是不是变慢了。

    REAL hall_change_cnt; // 电机走过了几个扇区？一般情况下是1。
    Uint16 hall_change_angle; // 两个扇区角度的平均值，在切换扇区时，等于N极的位置

    Uint16 hall_angle_est; // 电气角度输出
    REAL hall_angle_est_tmp; // 基于霍尔反馈的角度估计的增量，限幅在[-60, 60] elec. deg

    Uint16 hall_previous_previous_angle; // 上上个扇区角度
    Uint16 hall_previous_angle; // 上个扇区角度
    Uint16 hall_current_angle; // 当前扇区角度
};
extern struct HallSensor HALL;


void sensors();


// #define QEP_DEFAULTS {0, 0,0,0,0,0,{0,0,0,0},0,0}
struct eQEP_Variables{
    // double excitation_angle_deg; // init in synchronous_motor.c
    // double theta_d__state;   // init in synchronous_motor.c
    // double theta_d_offset;
    double QPOSLAT;
    int32 RoundedPosInCnt;
    int32 RoundedPosInCnt_Previous;
    int32 number_of_revolution;
    int32 difference_in_cnt;
    double theta_d;
    double theta_d_mech;
    double theta_d_accum;
    double omg_mech;
    double omg_elec;
    double moving_average[4];
    int64 ActualPosInCnt;
    int64 ActualPosInCnt_Previous;
};
extern struct eQEP_Variables qep;
#endif
