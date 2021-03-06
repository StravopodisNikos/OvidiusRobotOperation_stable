#ifndef task2_R075_definitions_h
#define task2_R075_definitions_h

#include <avr/pgmspace.h>

// CIRC TASKS, R0.075, 0.5kg
// SIM MATLAB LOG DATA ARE LOCATED IN:
// >> load('/media/nikos/NikosT5-SSD/OVIDIUS-TASK-EXECUTIONS/2-12-21/MATLAB/075circ05load/sTask2Eval_l05_180_7.mat')

// anatomy: 10-21, anatomy: [-0.6732 0.2244]->[5 9]
float point130[] = {-1.000    , -0.2500     ,  0.5000      };
float point131[] = {-0.9428    ,-0.2467      , 0.4686      };
float point132[] = {-0.8527    , -0.2458     , 0.4276      };
float point133[] = {-0.7597    , -0.2469     , 0.4013      };
float point134[] = {-0.6660   , -0.2472      , 0.3963      };
float point135[] = {-0.5735    , -0.2462      , 0.4138      };
float point136[] = {-0.4849    ,  -0.2459     , 0.4493      };
float point137[] = {-0.4035    , -0.2493     , 0.4949      };
float point138[] = {-0.3339    ,  -0.2594     , 0.5427      };
float point139[] = {-0.2828    , -0.2779     , 0.5862      };
float point140[] = {-0.2607   , -0.3054     , 0.6204      };
float point141[] = {-0.2839   , -0.3407     ,  0.6421      };
float point142[] = {-0.3753   , -0.3800      , 0.6504       };
float point143[] = {-0.5554    , -0.4152     , 0.6487      };
float point144[] = {-0.8043   , -0.4334     , 0.6452      };
float point145[] = {-1.0324   , -0.4249     , 0.6471      };
float point146[] = {-1.1676   , -0.3946     , 0.6506      };
float point147[] = {-1.2064   , -0.3182     , 0.6303      };
float point148[] = {-1.0496   , -0.2557     , 0.5294      };
float point149[] = {-1.0996    , -0.2654     , 0.5602      };

// anatomy: 9-18, anatomy: [-0.6732 -1.1220]->[5 3]
float point150[] = {-1.0000    , -0.2500    , 0.5000      };
float point151[] = {-0.8710    , -0.3712    , 0.4871      };
float point152[] = {-0.7538    , -0.3964    , 0.5941      };
float point153[] = {-0.6745    , -0.3770    , 0.7322      };
float point154[] = {-0.6335    , -0.3407    , 0.8711      };
float point155[] = {-0.6361    , -0.2972    , 0.9976      };
float point156[] = {-0.6851    , -0.2534    , 1.1021      };
float point157[] = {-0.7727    , -0.2175    , 1.1750      };
float point158[] = {-0.8761    , -0.2000    , 1.2069      };
float point159[] = {-0.9677    , -0.2082    , 1.1923      };
float point160[] = {-1.0356    , -0.2383    , 1.1339      };
float point161[] = {-1.0837    , -0.2803    , 1.0402      };
float point162[] = {-1.1197    , -0.3247    , 0.9210      };
float point163[] = {-1.1496    , -0.3646    , 0.7855      };
float point164[] = {-1.1782    , -0.3921    , 0.6450      };
float point165[] = {-1.2827    , -0.2742    , 0.4874      };
float point166[] = {-1.3236    , -0.1314    , 0.5137      };
float point167[] = {-1.2581    ,  0.1866    , 0.7599      };
float point168[] = {-1.2670    ,  0.1270    , 0.6841      };
float point169[] = {-1.0500    , -0.2700    , 0.5500      };

// anatomy: 5-12, anatomy: [-1.1220 -0.2244]->[3 7]
float point170[] = {-1.000     , -0.2500    ,  0.5000     };
float point171[] = {-0.9212    , -0.2609    ,  0.4625     };
float point172[] = {-0.8744    , -0.2460    ,  0.5142     };
float point173[] = {-0.8308    , -0.2229    ,  0.6040     };
float point174[] = {-0.7929    , -0.2002    ,  0.7120     };
float point175[] = {-0.7633    , -0.1836    ,  0.8238     };
float point176[] = {-0.7450    , -0.1760    ,  0.9297     };
float point177[] = {-0.7410    , -0.1778    ,  1.0230     };
float point178[] = {-0.7543    , -0.1871    ,  1.0991     };
float point179[] = {-0.7872    , -0.1998    ,  1.1548     };
float point180[] = {-0.8391    , -0.2108    ,  1.1889     };
float point181[] = {-0.9049    , -0.2153    ,  1.2008     };
float point182[] = {-0.9755    , -0.2114    ,  1.1905     };
float point183[] = {-1.0393    , -0.2008    ,  1.1582     };
float point184[] = {-1.0876    , -0.1880    ,  1.1040     };
float point185[] = {-1.1157    , -0.1783    ,  1.0294     };
float point186[] = {-1.1236    , -0.1758    ,  0.9373     };
float point187[] = {-1.1137    , -0.1827    ,  0.8321     };
float point188[] = {-1.0272    , -0.2370    ,  0.5475     };
float point189[] = {-1.0546    , -0.2211    ,  0.6118     };

// WORST ANATOMIES FOR CIRC TASKS, R0.075, 0.5kg
// anatomy: 4-11, anatomy: [-1.1220 -0.6732]->[3 5]
float point280[] = {-1.000     , -0.2500    ,  0.5000     };
float point281[] = {-0.9689    , -0.2445    ,  0.5138     };
float point282[] = {-0.9271    , -0.2181    ,  0.5827     };
float point283[] = {-0.8928    , -0.1810    ,  0.6889     };
float point284[] = {-0.8674    , -0.1450    ,  0.8111     };
float point285[] = {-0.8523    , -0.1174    ,  0.9347     };
float point286[] = { -0.8491   ,-0.1019      , 1.0499      };
float point287[] = {-0.8597    ,  -0.0989   ,  1.1498     };
float point288[] = {-0.8857    , -0.1060     , 1.2295       };
float point289[] = {-0.9286    , -0.1178    ,  1.2861     };
float point290[] = {-0.9872    , -0.1278    ,  1.3184     };
float point291[] = {-1.0562    , -0.1307    , 1.3263       };
float point292[] = {-1.1256    , -0.1249    , 1.3098     };
float point293[] = {-1.1836    , -0.1135    , 1.2689      };
float point294[] = {-1.2214    , -0.1026    , 1.2040       };
float point295[] = {-1.2357    , -0.0986    , 1.1168       };
float point296[] = {-1.2276    , -0.1058    ,1.0109        };
float point297[] = {-1.2014    , -0.1258    , 0.8918      };
float point298[] = {-1.1621    , -0.1570    ,  0.7674      };
float point299[] = {-1.0640    , -0.2292    ,   0.5530     };

// anatomy: 2-5, anatomy: [-1.5708 0.2244]->[1  9]
float point300[] = {-1.000     , -0.2500    ,  0.5000     };
float point301[] = {-0.9829    , -0.2966    ,  0.3780     };
float point302[] = {-0.9523    ,  -0.3615   ,  0.2168     };
float point303[] = { -0.9173    ,-0.3755     , 0.1831      };
float point304[] = {-0.8803    , -0.3190    ,  0.3214     };
float point305[] = {-0.8433    , -0.2492    ,  0.5021     };
float point306[] = {-0.8091    , -0.1835    ,  0.6862     };
float point307[] = {-0.7802    ,-0.1266      , 0.8630      };
float point308[] = {-0.7595     , -0.0805    , 1.0266      };
float point309[] = {-0.7499    , -0.0462    ,  1.1715     };
float point310[] = { -0.7537    ,-0.0234     , 1.2926      };
float point311[] = {-0.7719    ,  -0.0102   ,  1.3851     };
float point312[] = {-0.8038    , -0.0041    , 1.4445      };
float point313[] = {-0.8461    , -0.0023    , 1.4680      };
float point314[] = {-0.8935    , -0.0033    , 1.4544      };
float point315[] = { -0.9398    , -0.0081    ,1.4042      };
float point316[] = {-0.9793     , -0.0190    ,1.3202       };
float point317[] = {-1.0083    , -0.0391    , 1.2062      };
float point318[] = {-1.0251    , -0.0702    , 1.0671      };
float point319[] = {-1.0228    , -0.1674    ,    0.7342    };

// anatomy: 3-9, anatomy: [-1.1220 -1.5708]->[3  1]
float point320[] = {-1.000     , -0.2500    ,  0.5000     };
float point321[] = { -0.9019    ,  -0.4342    ,  0.4633     };
float point322[] = { -0.7607   , -0.6569    ,  0.4762     };
float point323[] = { -0.6337   , -0.7934    ,  0.5607     };
float point324[] = { -0.5280   , -0.8690    ,   0.6814     };
float point325[] = { -0.4510   , -0.9112    ,  0.8091     };
float point326[] = { -0.4102   , -0.9351     , 0.9248      };
float point327[] = { -0.4110   , -0.9482    ,  1.0146     };
float point328[] = { -0.4537   , -0.9543    , 1.0675      };
float point329[] = { -0.5311   , -0.9553    , 1.0766      };
float point330[] = { -0.6298   , -0.9514    , 1.0406      };
float point331[] = {  -0.7355   ,-0.9414     , 0.9645      };
float point332[] = {-0.8377    , -0.9224    ,  0.8576      };
float point333[] = { -0.9314   , -0.8888    , 0.7328      };
float point334[] = { -1.0154   , -0.8290    , 0.6065      };
float point335[] = { -1.0901    , -0.7209    , 0.5033      };
float point336[] = { -1.1565   , -0.5334    ,  0.4583     };
float point337[] = { -1.2144   ,-0.2687     ,  0.4950     };
float point338[] = { -1.2590   , -0.0030    ,  0.5897     };
float point339[] = { -1.1797   , 0.0688    ,  0.6240     };

#endif
