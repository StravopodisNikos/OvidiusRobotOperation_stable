#ifndef task_definitions_h
#define task_definitions_h

#include <avr/pgmspace.h>

// P2P TASK POINTS -> Cspace
const PROGMEM int robot_dof = 3;
const PROGMEM float random0[robot_dof] = {0      , 0     , 0     };
const PROGMEM float random1[robot_dof] = {0.1500 , 0.5000, 0.2500};
const PROGMEM float random2[robot_dof] = {-1.0, -1.2854, -0.189};
const PROGMEM float random3[robot_dof] = {0.7854 , -0.5  , 0.356 };
const PROGMEM float random4[robot_dof] = {0.7854 , -0.5  , 0.356 };
const PROGMEM float random5[robot_dof] = {0.1454 , 0.375  , -0.156};
const PROGMEM float random6[robot_dof] = {-0.7854 , -1.12  , 0.756 };
const PROGMEM float random7[robot_dof] = {1.574 , 1.000  , -0.4567 };

const PROGMEM float texec0 = 6.0;
const PROGMEM float texec1 = 3.0;
const PROGMEM float texec2 = 3.0;
const PROGMEM float texec3 = 3.5;
const PROGMEM float texec4 = 5.0;
const PROGMEM float texec5 = 3.0;
const PROGMEM float texec6 = 4.0;
const PROGMEM float texec7 = 4.0;

const PROGMEM int p2pPoints = 8;
const PROGMEM float * p2p_list[p2pPoints]   = {random0, random1, random2, random3, random4, random5, random6, random7};		// same as below but not in PROGMEM , [28-7-21] compiles and without PROGMEM
//float * const p2p_list[p2pPoints] PROGMEM  = {random0, random1, random2, random3};           // accepted in MEGA not DUE
const PROGMEM float p2p_dur[p2pPoints]      = {texec0, texec1, texec2, texec3, texec4, texec5, texec6, texec7};
 
// TRAJECTORY TASK POINTS -> Cspace POINTS + Texec
const PROGMEM int numTaskPoints = 5;
//const PROGMEM float * traj1[numTaskPoints]   = {random1, random2};
//const PROGMEM float traj1_dur[numTaskPoints] = {texec1,texec2};
/*
const PROGMEM float point0[robot_dof] = {0     , 0     ,  0      , 0      };
const PROGMEM float point1[robot_dof] = {0.7854, 0.1253, -0.4587 , 1.4584 };
const PROGMEM float point2[robot_dof] = {0.1478, -0.5253, 0.3587 , 0.1245 };
const PROGMEM float point3[robot_dof] = {0.7854, 0.6253, -0.4587 , -1.4584 };
const PROGMEM float point4[robot_dof] = {-0.5827, -0.3678, 1.4507 , 0.7854 };
const PROGMEM float t_inv0 = 4.0;
const PROGMEM float t_inv1 = 4.0;
const PROGMEM float t_inv2 = 4.0;
const PROGMEM float t_inv3 = 5.0;
const PROGMEM float t_inv4 = 5.0;
const PROGMEM float time_inv[numTaskPoints] = {t_inv0, t_inv1, t_inv2, t_inv3, t_inv4};
const PROGMEM float * traj1[numTaskPoints]  = {point0, point1, point2, point3, point4};
*/


// TRAJECTORY TASK POINTS -> Cspace POINTS + Texec
float start_point[] = {0.1000     ,0.1000      ,0.1000  };
float point0[] = {0     , 0     ,  0      };
float point1[] = {0.7854, 0.1253, -0.4587  };
float point2[] = {0.1478, -0.5253, 0.3587  };
float point3[] = {0.7854, 0.6253, -0.4587 };
float point4[] = {-0.5827, -0.3678, 1.4507  };
// ANATOMY_0
float point5[] = {0.1000   , 0.5000     ,  -0.2500      };
float point6[] = {0.0500   , 0.3886     ,  -0.5586      };
float point7[] = {-0.050   , 0.2786     ,  -0.9631      };
float point8[] = {0.0500   , 0.2395     ,  -1.2354      };
float point9[] = {-0.050   , 0.2362     ,  -1.3680      };
 // ANATOMY_1_1 = best_anat_bounded_mbs(1,:) = [1.3464, - 0.6732] = [14,5]
float point10[] = {-0.050   , 0.5000    ,  -0.2500      };
float point11[] = {0.0422   , 0.2977     ,  -0.5492     };
float point12[] = {0.1307   ,-0.0029     ,  -0.9261      };
float point13[] = {0.1992   ,-0.2263     ,  -1.1616       };
float point14[] = {0.2322   ,-0.3442     ,  -1.2682      };
// ANATOMY_2_1 = best_anat_bounded_mbs(2,:); = [0.2244, 0.2244] = [9,9]
float point15[] = {-0.050   , 0.5000     ,  -0.2500      };
float point16[] = {0.0374   , 0.3670     ,  -0.5943      };
float point17[] = {0.1234   , 0.2230     ,  -1.0426      };
float point18[] = {0.1994   , 0.1506     ,  -1.3531       };
float point19[] = {0.2404   , 0.1268     ,  -1.5100      };
//ANATOMY_3_1 = best_anat_bounded_mbs(3,:); = [-0.2244, 1.3464] = [7,14];
float point20[] = {-0.050   , 0.5000     ,  -0.2500      };
float point21[] = {0.1124   , 0.3665     ,  -0.6040      };
float point22[] = {0.2793   , 0.3910     ,  -0.9532      };
float point23[] = {0.3604   , 0.4993     ,  -1.1043      };
float point24[] = {0.3816   , 0.6119     ,  -1.1428      };

//ANATOMY_4_1 = best_anat_bounded_mbs(4,:); = [0, 1.5708] =  [8,15] ;
float point25[] = {-0.050   , 0.5000     ,  -0.2500      };
float point26[] = {0.1949   , 0.4058     ,  -0.6405      };
float point27[] = {0.4198   , 0.5148     ,  -0.9862      };
float point28[] = {0.5053   , 0.6677     ,  -1.1120      };
float point29[] = {0.5063   , 0.8092     ,  -1.1228      };

//ANATOMY_1_2 = worst_anat_bounded_mbs(1,:); = [-1.3464, - 0.2244] = [2,7];
float point30[] = {-0.050   , 0.5000     ,  -0.2500      };
float point31[] = {0.0049   , 0.6334     ,  -0.0313      };
float point32[] = {0.0223   , 0.7078     ,  -0.0997      };
float point33[] = {0.0440   , 0.8422     ,  -0.0215      };
float point34[] = {0.0702   , 0.9764     ,   0.0585      };

//ANATOMY_2_2 = worst_anat_bounded_mbs(2,:); = [1.1220, - 0.4488] = [13,6];
float point35[] = {-0.050   , 0.5000     ,  -0.2500      };
float point36[] = {0.0398   , 0.3120     ,  -0.5779      };
float point37[] = {0.1270   , 0.0354     ,  -0.9989      };
float point38[] = {0.1991   , -0.1754    ,  -1.2795      };
float point39[] = {0.2360   , -0.2895    ,  -1.4159      };

//ANATOMY_3_2 = worst_anat_bounded_mbs(3,:); = [-0.2244, - 1.1220] = [7,3]
float point40[] = {0.0500    , 0.5000     ,  -0.2500      };
float point41[] = {-0.0604   , 0.4189     ,  -0.4969      };
float point42[] = {-0.1674   , 0.4308     ,  -0.7867      };
float point43[] = {-0.2305   , 0.5181    ,   -0.9318      };
float point44[] = {-0.2563   , 0.6145    ,   -0.9885      };

//ANATOMY_4_2 = worst_anat_bounded_mbs(4,:); = [0, 1.3464] = [8, 14]
float point45[] = {0.050   , 0.5000     ,  -0.2500      };
float point46[] = {0.1503   , 0.3987     , -0.6137      };
float point47[] = {0.3807   , 0.4410     , -1.0086      };
float point48[] = {0.5128   , 0.5459    ,  -1.2177      };
float point49[] = {0.5720   , 0.6488    ,  -1.3107      };

// [30-11-21] CIRCULAR TASKS - MMA VS Mu
//ANATOMY43_CIRC_TASK -> sTaskEval(6).Qrmrc, avgEfMass: 6.2188, avgMMA: 12.4530, avgMVR: 0.1677, anatomy: [0.6732 -0.6732]
float point50[] = {0.1000    , 0.2500     ,  -0.2500     };
float point51[] = {0.1189    , 0.2193     ,  -0.3302     };
float point52[] = {0.1356    , 0.1836     ,  -0.4436      };
float point53[] = {0.1339    , 0.1616     ,  -0.5461      };
float point54[] = {0.1079    , 0.1546     ,  -0.6347      };
float point55[] = {0.0519    , 0.1633     ,  -0.7064      };
float point56[] = {-0.0382    , 0.1876     ,  -0.7585       };
float point57[] = {-0.1616    , 0.2260     ,  -0.7887      };
float point58[] = {-0.3058    , 0.2735     ,  -0.7965      };
float point59[] = {-0.4087    , 0.2961     , -0.7938      };
float point60[] = {-0.4008   , 0.2555     ,  -0.7960     };
float point61[] = {-0.3649   , 0.2102     ,  -0.7801      };
float point62[] = {-0.3235    , 0.1768     ,  -0.7416     };
float point63[] = {-0.2797    , 0.1583     ,  -0.6818      };
float point64[] = {-0.2341    , 0.1554     ,  -0.6035      };
float point65[] = {-0.1869    , 0.1681     , -0.5093      };
float point66[] = {-0.0894    , 0.2358     ,  -0.2857      };
float point67[] = {-0.0403    , 0.2879     ,  -0.1620      };
float point68[] = {0.0657    , 0.4425    ,  0.1359     };
float point69[] = {0.1000    , 0.2500     ,  -0.2500      };
//ANATOMY33_CIRC_TASK -> sTaskEval(2).Qrmrc, avgEfMass: 6.4405, avgMMA: 7.6870, avgMVR: 0.1250,anatomy: [0.2244 -1.5708]
float point70[] = {0.1000    , 0.2500     ,  -0.2500      };
float point71[] = {0.1646    , 0.2591     ,  -0.1787      };
float point72[] = {0.1809    , 0.2532     ,  -0.2194      };
float point73[] = {0.1286    , 0.2420     ,  -0.3749      };
float point74[] = {0.0543    , 0.2367     ,  -0.5557      };
float point75[] = {-0.0292    , 0.2341     , -0.7380      };
float point76[] = {-0.1190    , 0.2326     ,  -0.9132      };
float point77[] = {-0.2139    , 0.2318     , -1.0754      };
float point78[] = {-0.4107    ,  0.2311    ,  -1.3376     };
float point79[] = {-0.5040    , 0.2310     ,  -1.4253      };
float point80[] = {-0.5844    , 0.2310     ,  -1.4763      };
float point81[] = {-0.6428    , 0.2310     ,  -1.4868      };
float point82[] = {-0.6716    , 0.2310     ,  -1.4560      };
float point83[] = {-0.6663    , 0.2310     ,  -1.3863      };
float point84[] = {-0.6267    , 0.2312     ,  -1.2826      };
float point85[] = {-0.5559    , 0.2315     ,  -1.1509      };
float point86[] = {-0.3419    , 0.2332     ,  -0.8281      };
float point87[] = {-0.0241    , 0.2408     ,  -0.4056      };
float point88[] = {-0.0707    , 0.2388     ,   -0.4656       };
float point89[] = {0.1000    , 0.2500     ,  -0.2500      };

//ANATOMY41_CIRC_TASK -> sTaskEval(4).Qrmrc, avgEfMass: 6.5069, avgMMA: 8.5093, avgMVR: 0.1368, anatomy: [0.6732 -1.5708]
float point90[] = {0.1000    , 0.2500      ,  -0.2500      };
float point91[] = {0.0969    , 0.3234     ,  -0.2470      };
float point92[] = {0.0545    , 0.4616     ,  -0.2853      };
float point93[] = {-0.0095   , 0.5414      ,  -0.3930      };
float point94[] = {-0.0844   , 0.5797     ,  -0.5344      };
float point95[] = {-0.1666   , 0.5995     , -0.6831      };
float point96[] = {-0.2546    , 0.6106     , -0.8255      };
float point97[] = {-0.3463    , 0.6170     , -0.9525      };
float point98[] = {-0.4383    , 0.6208     , -1.0566      };
float point99[] = {-0.5251    , 0.6228     , -1.1311      };
float point100[] = {-0.5989    , 0.6237     , -1.1705      };
float point101[] = {-0.6513    , 0.6238     ,  -1.1717      };
float point102[] = {-0.6750    , 0.6229     ,  -1.1346      };
float point103[] = {-0.6663    , 0.6209     ,  -1.0622      };
float point104[] = {-0.6253    , 0.6173     ,  -0.9598      };
float point105[] = {-0.5552    , 0.6111     ,  -0.8340      };
float point106[] = {-0.4612    , 0.6004     ,  -0.6924      };
float point107[] = {-0.3489    , 0.5813     ,  -0.5437      };
float point108[] = {-0.2249    , 0.5447     ,  -0.4012      };
float point109[] = {0.0500    , 0.2869     ,  -0.2473     };

//ANATOMY42_CIRC_TASK -> sTaskEval(5).Qrmrc, avgEfMass: 6.0898, avgMMA: 7.9456,avgMVR: 0.1334, anatomy: [0.6732 -1.1220]
float point110[] = {0.1000    , 0.2500     ,  -0.2500      };
float point111[] = {0.1533    , 0.1443     ,  -0.3602      };
float point112[] = {0.2396    , -0.0122     , -0.5266      };
float point113[] = {0.3220    , -0.1466     , -0.6830      };
float point114[] = {0.3930    , -0.2534      , -0.8259      };
float point115[] = {0.4470    , -0.3326     , -0.9529      };
float point116[] = {0.4798    , -0.3871     , -1.0610      };
float point117[] = {0.4887    , -0.4211     , -1.1476      };
float point118[] = {0.4383    , -0.4484     , -1.2467      };
float point119[] = {0.3885    , -0.4503     , -1.2560      };
float point120[] = {0.3318    , -0.4465     , -1.2378      };
float point121[] = {0.2748    , -0.4352     ,  -1.1927      };
float point122[] = {0.2215    , -0.4120      , -1.1222      };
float point123[] = {0.1736    , -0.3719     ,  -1.0283      };
float point124[] = {0.1315    , -0.3099     , -0.9138      };
float point125[] = {0.0950    , -0.2220     , -0.7814      };
float point126[] = {0.0394    , 0.0365     , -0.4736      };
float point127[] = {0.0446    , 0.3406     , -0.1523      };
float point128[] = {0.0335    , 0.3287     , -0.1654      };
float point129[] = {0.1000     , 0.2500     , -0.2500      };

#endif
