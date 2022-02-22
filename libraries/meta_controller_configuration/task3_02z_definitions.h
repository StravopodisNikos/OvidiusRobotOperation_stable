#ifndef task3_02z_definitions_h
#define task3_02z_definitions_h

#include <avr/pgmspace.h>


// TASK3 - +0.2z - [4-12-21]

// SIM MATLAB LOG DATA ARE LOCATED IN:
// >> load('/media/nikos/NikosT5-SSD/OVIDIUS-TASK-EXECUTIONS/4-12-21/MATLAB/02z/sTask3Eval_4_12_180_7.mat')

// BEST MU/MMA + WORST MVR 
// anatomy: 1-49, anatomy: [1.1220 -1.5708]->[13 1]
float point_mu_00[] = {-0.5000    , -0.3500    ,  0.5000     };
float point_mu_01[] = {-0.4730    , -0.2664    ,  0.5593     };
float point_mu_02[] = {-0.3303    , 0.0937    ,  0.8351     };
float point_mu_03[] = {-0.2666    , 0.2578    ,  0.9481     };
float point_mu_04[] = {-0.2344    , 0.3785    , 1.0041     };
float point_mu_05[] = {-0.2251    , 0.4802    ,  1.0201     };
float point_mu_06[] = {-0.2370    , 0.5713    ,  0.9996     };
float point_mu_07[] = {-0.2723    , 0.6543    ,  0.9381     };
float point_mu_08[] = {-0.3412    ,  0.7267    ,  0.8153     };
float point_mu_09[] = {-0.4671    , 0.9939    ,  0.7010    };

// BEST MVR + WORST MU/MMA
// anatomy: 4-52, anatomy: [1.1220 -0.2244]->[13 7]
float point_mvr_00[] = {-0.5000    , -0.3500    ,  0.5000     };
float point_mvr_01[] = {-0.4998    , -0.3445    ,  0.5037     };
float point_mvr_02[] = {-0.4984    , -0.2800    ,  0.5382     };
float point_mvr_03[] = {-0.4976    , -0.2234    ,  0.5541     };
float point_mvr_04[] = {-0.4977    , -0.1743    ,  0.5528     };
float point_mvr_05[] = {-0.4985   ,  -0.1330    ,  0.5343     };
float point_mvr_06[] = {-0.5001    , -0.1004    ,  0.4967     };
float point_mvr_07[] = {-0.5025    , -0.0788   ,  0.4355     };
float point_mvr_08[] = {-0.5057    , -0.0737    ,  0.3390     };
float point_mvr_09[] = {-0.5093    , -0.1029    ,  0.1764    };

#endif
