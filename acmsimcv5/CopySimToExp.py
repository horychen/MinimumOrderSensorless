from shutil import copyfile
def main():
    # Do not copy ACMSim.h
    for fname in ['pid_regulator.c',
                  'pid_regulator.h',
                  'shared_flux_estimator.c',
                  'shared_flux_estimator.h',
                  'im_controller.c',
                  'im_controller.h',
                  'im_observer.c',
                  'im_observer.h',
                  'pmsm_controller.c',
                  'pmsm_controller.h',
                  'pmsm_observer.c',
                  'pmsm_observer.h',
                  'pmsm_comm.c',
                  'pmsm_comm.h',
                  'global_variables_definitions.c',
                  'utility.c',
                  'ACMConfig.h',
                  'sweep_frequency.h',
               ]:
        # copyfile(f'c/{fname}', rf'D:\DrH\Codes\ProjectPanGu-C\The acmsimcv5/{fname}')
        copyfile(f'c/{fname}', rf'D:\codes\Pangu_induction\The acmsimcv5/{fname}')
if __name__ == '__main__':
    main()
