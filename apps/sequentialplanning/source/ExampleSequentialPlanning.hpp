#pragma once


#include <mpsv/mpsv.hpp>

// required for filename generation
#include <filesystem>
#ifdef _WIN32
#include <windows.h>
#endif


namespace example {


/**
 * @brief Get an example parameter set for the sequential planner.
 * @return Example parameter set.
 */
inline mpsv::planner::SequentialPlannerParameterSet GetParameterSet(void){
    mpsv::planner::SequentialPlannerParameterSet parameterSet;

    // model
    parameterSet.model.matF                                            = {-0.0019, 0.0, 0.0, 1.5328, 0.0, 0.0, 0.0, 0.0, 0.0, -0.0008, 0.0, 0.0, 0.0, -0.0121, -0.1886, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.1036, 0.0, 0.0, -0.0005, -0.0197, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -71.37};
    parameterSet.model.matB                                            = {1.9854e-6, 0.0, 0.0, 0.0, 1.3247e-6, 0.0, 0.0, 0.0, 5.0e-9};
    parameterSet.model.vecTimeconstantsXYN                             = {2.0, 2.0, 2.0};
    parameterSet.model.vecTimeconstantsInput                           = {4.0, 4.0, 4.0};
    parameterSet.model.lowerLimitXYN                                   = {-9600.0, -5000.0, -90000.0};
    parameterSet.model.upperLimitXYN                                   = {9600.0, 5000.0, 90000.0};

    // cost map
    parameterSet.costMap.modBreakpoints                                = 10;
    parameterSet.costMap.resolution                                    = 0.5;
    parameterSet.costMap.distanceScale                                 = 5.0;
    parameterSet.costMap.distanceDecay                                 = 0.0015;

    // geometry
    parameterSet.geometry.collisionCheckMaxPositionDeviation           = 0.100000000000000;
    parameterSet.geometry.collisionCheckMaxAngleDeviation              = mpsv::math::deg2rad(5.0);
    std::vector<std::array<double,2>> vertices                         = {{28.0, 0.0},{20.0,8.5},{-26.0,8.5},{-26.0,-8.5},{20.0,-8.5}};
    parameterSet.geometry.vehicleShape.Create(vertices);
    parameterSet.geometry.skeletalPoints.clear();
    parameterSet.geometry.skeletalPoints.push_back({28.0, 0.0});
    parameterSet.geometry.skeletalPoints.push_back({-26.0, 0.0});

    // metric
    parameterSet.metric.weightPsi                                      = 5.0;
    parameterSet.metric.weightSway                                     = 1.0;
    parameterSet.metric.weightReverseScale                             = 0.1;
    parameterSet.metric.weightReverseDecay                             = 1.0;

    // path planner
    parameterSet.pathPlanner.periodGoalSampling                        = 100;

    // motion panner
    parameterSet.motionPlanner.sampletime                              = 0.5;
    parameterSet.motionPlanner.samplingRangePosition                   = 50.0;
    parameterSet.motionPlanner.samplingRangeAngle                      = 1.0;
    parameterSet.motionPlanner.maxInputPathLength                      = 1000.0;
    parameterSet.motionPlanner.periodGoalSampling                      = 100;
    parameterSet.motionPlanner.regionOfAttraction.rangePose            = {0.2, 0.2, 0.0873};
    parameterSet.motionPlanner.regionOfAttraction.rangeUVR             = {0.05, 0.05, 0.001};
    parameterSet.motionPlanner.regionOfAttraction.rangeXYN             = {300.0, 300.0, 1000.0};
    parameterSet.motionPlanner.controller.matK                         = {3.6e-4, 0.0, 0.0, 1.8e-2, 0.0, 0.0, 2.63e-1, 0.0, 0.0, 9.9e-1, 0.0, 0.0, 0.0, 3.6e-4, 0.0, 0.0, 1.8e-2, 0.0, 0.0, 2.63e-1, 0.0, 0.0, 9.9e-1, 0.0, 0.0, 0.0, 3.6e-4, 0.0, 0.0, 1.8e-2, 0.0, 0.0, 2.63e-1, 0.0, 0.0, 9.9e-1};
    parameterSet.motionPlanner.controller.maxRadiusX                   = 50.0;
    parameterSet.motionPlanner.controller.maxRadiusY                   = 9.0;
    parameterSet.motionPlanner.controller.maxRadiusPsi                 = 0.43;
    parameterSet.motionPlanner.controller.minRadiusPosition            = 6.0;
    return parameterSet;
}


/**
 * @brief Get an example input data for the sequential planner.
 * @return Example input data.
 */
inline mpsv::planner::SequentialPlannerInput GetInput(void){
    mpsv::planner::SequentialPlannerInput dataIn;
    dataIn.initialStateAndInput  = {-70.0, -130.0, 0.7, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    dataIn.finalPose             = {-37.0, 243.0, 1.6923};
    dataIn.finalPose             = {-37.0, 243.0, -1.4492};
    dataIn.originOldToNew        = {0.0, 0.0};
    dataIn.samplingBoxCenterPose = {-20.0, 50.0, 0.0};
    dataIn.samplingBoxDimension  = {200.0, 550.0};
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-400.0000,400.0000},{-237.3436,-52.9620},{-240.0308,-64.7144},{-242.5056,-74.0184},{-249.4236,-86.4836},{-400.0000,-319.9404}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-108.0060,219.7068},{-103.8024,162.7244},{-101.7804,133.2980},{-206.6656,-25.0496},{-223.4280,-34.5320},{-400.0000,400.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{84.1296,377.3896},{69.2080,368.7824},{2.9008,330.5352},{-5.6352,325.6116},{-7.6352,324.4676},{-39.0420,306.6492},{-40.8512,305.6252},{-400.0000,400.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{291.3608,-32.3052},{268.8532,-150.2768},{217.2360,-161.6740},{153.5112,-37.0256},{162.4244,-30.9712},{171.5236,-24.9164},{181.8204,-18.1500},{193.6340,-13.6536},{283.9640,-25.2268}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-78.1908,284.1640},{-80.1908,283.0224},{-97.0972,273.4392},{-400.0000,400.0000},{-40.8512,305.6252}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{123.4032,399.9960},{86.1296,378.5420},{-400.0000,400.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-237.3436,-52.9620},{-400.0000,400.0000},{-223.4280,-34.5320}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{78.4532,-76.6912},{86.8876,-65.7844},{95.9872,-58.7948},{111.4724,-52.1172},{142.5224,-41.1212},{217.2360,-161.6740},{167.1880,-177.4780},{95.3760,-198.1792},{71.4292,-87.2864}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{268.8532,-150.2768},{291.3608,-32.3052},{300.8064,-35.7772},{341.3036,-129.2632}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-110.0812,256.4780},{-109.3364,246.3728},{-108.0060,219.7068},{-400.0000,400.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-110.0812,256.4780},{-400.0000,400.0000},{-97.0972,273.4392}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-280.6620,-210.4644},{-291.4912,-236.5960},{-301.0700,-257.8304},{-312.0592,-278.1304},{-320.2544,-286.3216},{-321.1324,-287.0780},{-328.3432,-292.8652},{-341.3540,-299.9436},{-357.2920,-307.0660},{-385.1764,-316.1468},{-400.0000,-319.9404},{-256.0224,-116.9332},{-261.5304,-143.8664},{-268.5816,-174.4944},{-272.2004,-185.3120}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-101.7804,133.2980},{-98.4816,105.9644},{-98.5612,97.4172},{-99.6524,86.1096},{-111.3596,40.6572},{-206.6656,-25.0496}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-20.7372,-8.6684},{-14.7240,-16.0140},{-15.3892,-20.8664},{-26.0960,-29.7420},{-55.8848,-24.6948},{-111.3596,40.6572},{-79.1040,61.4260}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{386.5280,236.8940},{392.6208,249.3144},{399.4320,254.1228},{400.0000,254.2228},{400.0000,83.3940},{381.4732,219.8436}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-400.0000,-319.9404},{-249.4236,-86.4836},{-256.0224,-116.9332}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{12.1224,-143.2448},{16.6720,-130.2904},{54.4272,-101.7544},{62.4096,-96.0564},{95.3760,-198.1792},{77.6024,-204.2784},{70.1260,-205.1240},{63.5540,-205.8364},{55.4388,-204.9016},{48.6272,-201.5628},{37.0000,-195.2412},{24.7340,-186.2932},{19.7056,-180.8176},{14.1712,-169.8664},{12.2024,-161.4524},{11.7764,-154.2408}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{341.3036,-129.2632},{300.8064,-35.7772},{310.6508,-35.9552},{330.5528,-36.7116},{344.4416,-39.4268},{352.8228,-42.3648},{375.2540,-119.3796}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{388.4244,-112.4792},{361.8960,-43.3888},{397.9748,-40.2716},{393.8520,-110.1196}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-269.4540,-260.2144},{-278.9596,-258.1424},{-264.8304,-191.0104},{-262.7548,-185.1340},{-238.7816,-168.3512},{-258.7384,-262.5500}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{369.9000,195.8932},{375.6732,207.6904},{400.0000,83.3940},{360.6148,166.8228}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{153.5112,-37.0256},{217.2360,-161.6740},{142.5224,-41.1212}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{346.6468,142.8724},{355.5864,153.2004},{400.0000,83.3940},{389.0856,79.6584},{373.8400,77.4320},{348.9096,76.2296},{334.4348,109.2612},{335.0732,119.1888}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{95.3760,-198.1792},{62.4096,-96.0564},{71.4292,-87.2864}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-80.7692,63.4260},{-111.3596,40.6572},{-99.6524,86.1096}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{400.0000,83.3940},{375.6732,207.6904},{381.4732,219.8436}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{400.0000,83.3940},{355.5864,153.2004},{360.6148,166.8228}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-12.5420,202.7896},{-15.3092,202.3892},{-17.8632,202.0776},{-21.0296,201.7212},{-30.5280,279.6716},{-24.2220,298.2796}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{202.0684,-8.4448},{273.1352,-14.3204},{283.9640,-25.2268},{197.5984,-11.5164}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{101.0668,372.0012},{98.8320,370.6656},{39.8992,341.9068},{38.2496,344.7560},{123.2828,393.6368}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{400.0000,347.0244},{385.7804,374.1416},{400.0000,400.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{361.8960,-43.3888},{388.4244,-112.4792},{382.4112,-116.0408},{375.2540,-119.3796},{352.8228,-42.3648}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{400.0000,400.0000},{385.7804,374.1416},{373.3816,395.3760}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{86.1296,378.5420},{84.1296,377.3896},{-400.0000,400.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{44.6884,306.1148},{45.4068,303.7556},{53.4420,276.8224},{49.2648,264.5356},{29.7356,329.6200}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{217.8192,2.6404},{228.8612,6.6468},{258.9532,6.0240},{268.3192,-3.2800},{273.1352,-14.3204},{202.0684,-8.4448},{209.4384,-2.8800}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-238.7816,-168.3512},{-262.7548,-185.1340},{-258.6040,-163.8100}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{397.9748,-40.2716},{400.0000,-40.1204},{400.0000,-108.9384},{393.8520,-110.1196}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{348.9096,76.2296},{333.2116,72.3120},{324.4316,75.3832},{322.8084,81.7492},{323.7660,88.1600},{328.1292,97.2860},{334.4348,109.2612}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{24.3876,326.9044},{8.6100,317.6004},{-17.7036,303.7108},{-19.7256,307.0052},{-20.8964,309.0084},{20.3436,333.6712}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-26.0960,-29.7420},{-28.0960,-31.4040},{-42.4748,-40.4540},{-55.8848,-24.6948}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{74.0360,49.8724},{69.9388,56.3716},{104.9264,78.4080},{104.2192,68.8748}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{98.8320,370.6656},{41.9744,338.2564},{35.3228,337.2772},{39.8992,341.9068}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{41.4168,-104.9596},{42.0552,-104.6480},{54.4272,-101.7544},{16.6720,-130.2904},{20.6632,-122.2324},{26.3040,-115.1988},{33.6208,-108.9220}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{283.9640,-25.2268},{193.6340,-13.6536},{197.5984,-11.5164}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{88.6428,159.5188},{89.3612,157.1148},{80.9800,158.7620},{80.2616,161.1216},{71.5080,190.3248},{80.4212,186.8972}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{97.7688,129.1136},{98.4872,126.7540},{106.0968,101.2900},{97.7956,102.6700},{89.3876,130.7604}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{89.3876,130.7604},{88.6696,133.1200},{80.9800,158.7620},{89.3612,157.1148},{97.7688,129.1136}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{63.0468,244.8144},{55.3312,244.2356},{49.9564,262.1316},{49.2648,264.5356},{53.4420,276.8224},{54.1604,274.4628},{62.3552,247.2184}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{56.0492,241.8316},{55.3312,244.2356},{63.0468,244.8144},{71.1352,217.8812},{62.4616,220.5076}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{28.6448,333.1816},{28.0592,335.2740},{35.3228,337.2772},{36.6532,332.8700},{44.6884,306.1148},{29.7356,329.6200}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{80.4212,186.8972},{71.5080,190.3248},{70.7896,192.6844},{63.1800,218.1040},{71.8536,215.4772},{79.7292,189.2564}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{123.2828,393.6368},{123.8684,385.0892},{121.6332,383.7984},{101.0668,372.0012}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{107.0284,82.7708},{104.4208,81.9696},{103.8888,81.8360},{98.5140,100.2660},{106.0968,101.2900},{106.8152,98.9308}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{239.2908,11.0096},{247.8848,11.6332},{258.9532,6.0240},{228.8612,6.6468}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{400.0000,400.0000},{373.3816,395.3760},{370.3920,399.9992}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-24.9140,303.8444},{-24.7808,302.8652},{-24.2220,298.2796},{-30.5280,279.6716},{-31.5388,287.8628},{-33.3748,302.8652}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{335.0732,119.1888},{337.5476,133.2120},{346.6468,142.8724}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-80.7692,63.4260},{-78.9368,63.4260},{-79.1040,61.4260},{-111.3596,40.6572}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{107.0284,82.7708},{106.8152,98.9308},{111.2852,84.0172}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{104.4208,81.9696},{107.4008,79.9664},{113.0148,74.4016},{106.2192,70.1328},{104.9264,78.4080}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{54.1604,274.4628},{53.4420,276.8224},{76.3232,283.7228},{77.0684,281.3188}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{112.2692,164.0152},{89.3612,157.1148},{88.6428,159.5188},{111.5508,166.3748}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{74.9140,95.7696},{97.7956,102.6700},{98.5140,100.2660},{75.6324,93.4104}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{71.5080,190.3248},{48.6264,183.4248},{47.9080,185.8288},{70.7896,192.6844}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{39.5800,213.6076},{62.4616,220.5076},{63.1800,218.1040},{40.2984,211.2480}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{94.7352,222.3776},{71.8536,215.4772},{71.1352,217.8812},{94.0168,224.7372}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{32.4496,237.3800},{55.3312,244.2356},{56.0492,241.8316},{33.1680,234.9760}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{62.3552,247.2184},{85.2368,254.0740},{85.9284,251.6700},{63.0468,244.8144}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{49.2648,264.5356},{49.9564,262.1316},{27.0748,255.2760},{26.3832,257.6800}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{106.8152,98.9308},{106.0968,101.2900},{129.0052,108.1460},{129.7236,105.7864}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{66.5060,123.9048},{65.7876,126.2196},{88.6696,133.1200},{89.3876,130.7604}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{58.0984,151.9064},{57.3800,154.2656},{80.2616,161.1216},{80.9800,158.7620}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{45.4068,303.7556},{44.6884,306.1148},{67.5696,312.9708},{68.2880,310.6112}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{120.6504,135.9692},{121.3424,133.6100},{98.4872,126.7540},{97.7688,129.1136}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{79.7292,189.2564},{102.6108,196.1124},{103.3028,193.7528},{80.4212,186.8972}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-264.8304,-191.0104},{-272.2004,-185.3120},{-270.2048,-179.3468},{-262.7548,-185.1340}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{118.4160,80.0552},{144.0648,83.3944},{144.3572,81.2576},{118.6820,77.9184}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-31.5388,287.8628},{-30.5280,279.6716},{-31.4968,282.1072},{-40.8512,305.6252},{-39.0420,306.6492}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{360.6148,166.8228},{365.1108,186.1436},{369.9000,195.8932}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{8.6100,317.6004},{-16.9056,302.3308},{-17.7036,303.7108}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{109.2352,357.7556},{107.0268,356.5088},{98.8320,370.6656},{101.0668,372.0012}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{123.8684,385.0892},{132.0632,370.8884},{129.8280,369.5972},{121.6332,383.7984}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-15.9744,185.8284},{-17.8632,202.0776},{-15.3092,202.3892},{-13.4468,186.1400}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-19.7256,307.0052},{-17.7036,303.7108},{-24.2220,298.2796},{-24.7808,302.8652}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-42.4748,-40.4540},{-28.0960,-31.4040},{-38.8564,-40.3648}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{35.3228,337.2772},{41.9744,338.2564},{36.6532,332.8700}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{127.1144,386.9592},{123.8684,385.0892},{123.2828,393.6368}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{28.6448,333.1816},{29.7356,329.6200},{24.0948,327.3944},{22.1260,330.6884}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{106.0968,101.2900},{98.5140,100.2660},{97.7956,102.6700}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{71.8536,215.4772},{63.1800,218.1040},{62.4616,220.5076},{71.1352,217.8812}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{106.2192,68.5044},{104.2192,68.5044},{104.2192,68.8748},{104.9264,78.4080},{106.2192,70.1328}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{113.0148,74.4016},{107.4008,79.9664},{108.8908,80.9456}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{107.0284,82.7708},{107.4008,79.9664},{104.4208,81.9696}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-233.9236,-160.0060},{-233.9236,-162.0060},{-235.9236,-162.0060},{-235.9236,-160.0060}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-267.7992,-176.5660},{-265.7992,-176.5660},{-265.7992,-178.5660},{-267.7992,-178.5660}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-237.7848,-60.8652},{-235.7848,-60.8656},{-235.7848,-62.8656},{-237.7848,-62.8652}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-253.2132,-132.1824},{-253.2132,-134.1824},{-255.2132,-134.1820},{-255.2132,-132.1820}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-77.3720,-5.6204},{-75.3720,-5.6204},{-75.3720,-7.6204},{-77.3720,-7.6204}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-218.5700,-46.8840},{-220.5700,-46.8840},{-220.5700,-44.8840},{-218.5700,-44.8840}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-260.6428,-267.2928},{-258.6428,-267.2928},{-258.6432,-269.2928},{-260.6432,-269.2928}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-249.5200,-163.9644},{-249.5200,-161.9644},{-247.5200,-161.9644},{-247.5200,-163.9644}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-244.1932,-90.7364},{-244.1932,-92.7364},{-246.1932,-92.7364},{-246.1932,-90.7364}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-101.2648,21.4908},{-99.2648,21.4908},{-99.2648,19.4908},{-101.2648,19.4908}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{40.6816,352.1220},{42.6816,352.1220},{42.6816,350.1220},{40.6816,350.1220}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-21.2000,202.6356},{-23.2000,202.6356},{-23.2000,204.6356},{-21.3848,204.6356},{-21.2000,203.1220}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{48.1096,264.2480},{46.1096,264.2480},{46.1096,266.2480},{48.1096,266.2480}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{147.0824,81.9492},{147.0824,83.9492},{149.0824,83.9492},{149.0824,81.9492}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-33.4968,282.4076},{-31.6164,282.4076},{-31.4968,282.1072},{-31.4968,280.4076},{-33.4968,280.4076}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{113.8772,78.8740},{115.8772,78.8740},{115.8772,76.8740},{113.8772,76.8740}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-57.8904,293.6260},{-57.8904,291.6260},{-59.8904,291.6260},{-59.8904,293.6260}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-222.7472,-47.8632},{-224.7472,-47.8632},{-224.7472,-45.8632},{-222.7472,-45.8632}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{2.9008,330.5352},{2.9704,330.5756},{4.9008,330.5756},{4.9008,328.5756},{2.9008,328.5756}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{78.9948,371.9804},{78.9948,373.9804},{80.9948,373.9804},{80.9948,371.9804}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-251.6436,-123.5872},{-253.6436,-123.5872},{-253.6432,-121.5872},{-251.6432,-121.5872}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{120.2920,395.8388},{120.2920,393.8388},{118.2920,393.8388},{118.2920,395.8388}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-269.4540,-260.2144},{-268.6472,-260.3904},{-268.6472,-262.2144},{-270.6472,-262.2144},{-270.6472,-260.2144}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-26.0960,-29.7420},{-26.0960,-31.5712},{-28.0960,-31.5712},{-28.0960,-31.4040}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-5.6352,324.0348},{-7.6352,324.0348},{-7.6352,324.4676},{-5.6352,325.6116}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{84.1296,376.9664},{84.1296,377.3896},{86.1296,378.5420},{86.1296,376.9664}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-80.1908,282.5892},{-80.1908,283.0224},{-78.1908,284.1640},{-78.1908,282.5892}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-272.8376,-187.2056},{-271.3068,-187.2056},{-271.3068,-189.2056},{-273.3068,-189.2056},{-273.3068,-188.6012}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{72.6176,52.1220},{71.9188,52.1220},{71.9188,53.2312}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-78.9368,63.4260},{-78.9368,61.4260},{-79.1040,61.4260}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{67.6384,367.4840},{66.9572,367.4840},{67.6384,367.8768}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{69.2080,368.5968},{68.8864,368.5968},{69.2080,368.7824}}));
    for(auto&& polygon : dataIn.staticObstacles){
        polygon.EnsureCorrectVertexOrder();
    }
    return dataIn;
}

/**
 * @brief Get the filename for the data log file.
 * @return A string containing the absolute filename.
 */
std::string GetDataFilename(void){
    #ifdef _WIN32
    char* buffer = new char[65536];
    DWORD len = GetModuleFileNameA(NULL, &buffer[0], 65536);
    std::string str(buffer, len);
    delete[] buffer;
    #else
    std::string str("/proc/self/exe");
    #endif
    std::filesystem::path applicationPath;
    try {
        applicationPath = std::filesystem::canonical(str);
        applicationPath.remove_filename();
    }
    catch(...){ }
    std::filesystem::path filePath = applicationPath / "data";
    return filePath.string();
}


} /* namespace: example */

