#include "ACMSim.h"
// 主函数
int main(){
    // 初始化
    init_Machine(); // 仿真电机初始化
    init_experiment();
    // 打印
    print_info();
    // 声明文件，并将变量名写入文件
    FILE *fw; fw = fopen(DATA_FILE_NAME, "w"); write_header_to_file(fw);
    // 主循环
    clock_t begin, end; begin = clock(); int _, dfe_counter=0; // _ for the outer iteration // dfe_counter for down frequency execution （降频执行变量）
    for(_=0;_<NUMBER_OF_STEPS*TS_UPSAMPLING_FREQ_EXE_INVERSE;++_){
        // 负载转矩
        ACM.TLoad = load_model();
        // 每隔 MACHINE_TS 调用电机仿真代码一次
        ACM.timebase += MACHINE_TS; if(machine_simulation()){ printf("main.c: Break the loop.\n"); break;}
        // 降频执行控制器代码（比如电机仿真执行两次或多次，才执行控制器代码一次）
        if(++dfe_counter == TS_UPSAMPLING_FREQ_EXE_INVERSE){
            dfe_counter = 0;
            // DSP中的时间
            CTRL.timebase += CL_TS;
            // 采样，包括DSP中的ADC采样等
            measurement();
            // 写数据到文件
            write_data_to_file(fw);
            // 根据指令，产生控制输出（电压）
            #if ENABLE_COMMISSIONING
                commissioning();
            #else
                // 生成转速指令
                REAL set_rpm_speed_command, set_iq_cmd, set_id_cmd;
                // commands(&set_rpm_speed_command, &set_iq_cmd, &set_id_cmd);
                controller(set_rpm_speed_command, 0, set_iq_cmd, set_id_cmd, 0, 0.0, 0.0, 0.0);
            #endif
            // 电压指令CTRL.O->uab_cmd[0/1]通过逆变器，产生实际电压ACM.ual, ACM.ube（变换到dq系下得到ACM.ud，ACM.uq）
            // voltage_commands_to_pwm(); // in DSP
            // CTRL.O->uab_cmd_to_inverter[0] = 10*cos(5*2*M_PI*ACM.timebase);
            // CTRL.O->uab_cmd_to_inverter[1] = 10*sin(5*2*M_PI*ACM.timebase);
            inverter_model(); // in Simulation
        }
    }
    end = clock(); printf("\t[main.c] The simulation in C costs %g sec.\n", (double)(end - begin)/CLOCKS_PER_SEC);
    fclose(fw); // getch(); // system("cd .. && python ACMPlot.py"); system("pause"); // 调用python脚本绘图
    return 0;
}
void print_info(){
    if(SENSORLESS_CONTROL==TRUE){
        printf("\t[main.c] Sensorless using observer.\n");
    }else{
        printf("\t[main.c] Sensored control.\n");
    }
    printf("\t[main.c] NUMBER_OF_STEPS: %d\n", NUMBER_OF_STEPS);
    printf("\t[main.c] Speed PID:   Kp=%.4f, Ki=%.3f, limit=%.1f A\n", pid1_spd.Kp, pid1_spd.Ki/CL_TS, pid1_spd.OutLimit);
    printf("\t[main.c] Current PID: Kp=%.3f, Ki=%.3f, limit=%.1f V\n", pid1_id.Kp, pid1_id.Ki/CL_TS, pid1_id.OutLimit);
}



/* 不用整这些虚的，你不是一开始就知道死区时间是多少的吗？ */
//     #define LUT_N_LC  70
//     #define LUT_N_HC  29
//     REAL lut_lc_voltage[70] = {0, -0.0105529, 0.31933, 0.364001, 0.415814, 0.489953, 0.602715, 0.769718, 0.971424, 1.21079, 1.50055, 1.83306, 2.16318, 2.54303, 2.92186, 3.24129, 3.51575, 3.75058, 3.97849, 4.16454, 4.33493, 4.49719, 4.64278, 4.76509, 4.88146, 4.99055, 5.06347, 5.16252, 5.24808, 5.30369, 5.36092, 5.44246, 5.50212, 5.5786, 5.63384, 5.69022, 5.74442, 5.79613, 5.8491, 5.89762, 5.93325, 5.98141, 6.01726, 6.06201, 6.09346, 6.13419, 6.16634, 6.19528, 6.2233, 6.25819, 6.29004, 6.31378, 6.34112, 6.3669, 6.38991, 6.4147, 6.4381, 6.46156, 6.48171, 6.49962, 6.51565, 6.53689, 6.5566, 6.57761, 6.59515, 6.60624, 6.62549, 6.64589, 6.65606, 6.67132};
//     REAL lut_hc_voltage[29] = {6.69023, 6.80461, 6.89879, 6.96976, 7.02613, 7.08644, 7.12535, 7.17312, 7.20858, 7.2444, 7.27558, 7.30321, 7.32961, 7.35726, 7.38272, 7.39944, 7.42055, 7.43142, 7.4416, 7.43598, 7.44959, 7.45352, 7.45434, 7.45356, 7.45172, 7.45522, 7.45602, 7.44348, 7.43926};
//     #define LUT_STEPSIZE_BIG 0.11641244037931034
//     #define LUT_STEPSIZE_SMALL 0.01237159786376811
//     #define LUT_STEPSIZE_BIG_INVERSE 8.59014721057018
//     #define LUT_STEPSIZE_SMALL_INVERSE 80.83030268294078
//     #define LUT_I_TURNING_LC 0.8660118504637677
//     #define LUT_I_TURNING_HC 4.241972621463768
//     #define V_PLATEAU 7.43925517763064
// REAL lookup_compensation_voltage_indexed(REAL current_value){
//     REAL abs_current_value = fabs(current_value);

//     if(abs_current_value < LUT_I_TURNING_LC){
//         REAL float_index = abs_current_value * LUT_STEPSIZE_SMALL_INVERSE;
//         int index = (int)float_index;
//         REAL slope;
//         if(index+1 >= LUT_N_LC)
//             slope = (lut_hc_voltage[0] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         else
//             slope = (lut_lc_voltage[index+1] - lut_lc_voltage[index]) * LUT_STEPSIZE_SMALL_INVERSE;
//         return sign(current_value) * (lut_lc_voltage[index] + slope * (abs_current_value - index*LUT_STEPSIZE_SMALL));
//     }else{
//         REAL float_index = (abs_current_value - LUT_I_TURNING_LC) * LUT_STEPSIZE_BIG_INVERSE;
//         int index = (int)float_index; // THIS IS A RELATIVE INDEX!
//         REAL slope;
//         if(index+1 >= LUT_N_HC)
//             return V_PLATEAU;
//         else
//             slope = (lut_hc_voltage[index+1] - lut_hc_voltage[index]) * LUT_STEPSIZE_BIG_INVERSE;
//         return sign(current_value) * (lut_hc_voltage[index] + slope * (abs_current_value - LUT_I_TURNING_LC - index*LUT_STEPSIZE_BIG));
//     }
// }
//     int i=0;
//     while(TRUE){
//         i+=1;
//         printf("%g, %g\n", lookup_compensation_voltage_indexed(0.02*i), 0.02*i);
//         if(i>100)
//             break;
//     }
//     return 0;
