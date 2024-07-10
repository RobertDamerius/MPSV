#pragma once


#include <mpsv/mpsv.hpp>


namespace example {


/**
 * @brief Get an example parameter set for the motion planner.
 * @return Example parameter set.
 */
inline mpsv::planner::MotionPlannerParameterSet GetParameterSet(void){
    mpsv::planner::MotionPlannerParameterSet parameterSet;

    // model
    parameterSet.model.matF                                            = {-0.031604977390302, 0.0, 0.0, 0.233741474945168, 0.0, 0.0, 0.0, 0.0, 0.133462292221887, -0.020792892088185, 0.0, 0.0, 0.0, -0.041708610292712, 0.017242263124379, 0.0, -0.251243228165994, -0.000476044356140, 0.0, 0.0, 0.005297185500344, 0.0, -0.167981638498434, 0.497442136687157, 0.0, 0.000017966020615, -0.023528504195578, 0.0, 0.000108223241795, 0.000649602175186, 0.0, 0.0, -0.000002281767319, 0.0, 0.000072358238719, -0.678801229030825};
    parameterSet.model.matB                                            = {0.000237948834266, -0.000004551592718, 0.000010003488944, -0.000009313932115, 0.000215194147058, -0.000024957572224, -0.000002202124158, -0.000002930260852, 0.000043018345190};
    parameterSet.model.vecTimeconstantsXYN                             = {0.2, 0.2, 0.2};
    parameterSet.model.vecTimeconstantsInput                           = {0.5, 0.5, 0.5};
    parameterSet.model.lowerLimitXYN                                   = {-630.0, -495.0, -675.0};
    parameterSet.model.upperLimitXYN                                   = {630.0, 495.0, 675.0};

    // cost map
    parameterSet.costMap.modBreakpoints                                = 10;
    parameterSet.costMap.resolution                                    = 0.05;
    parameterSet.costMap.distanceScale                                 = 5.0;
    parameterSet.costMap.distanceDecay                                 = 0.02;

    // geometry
    parameterSet.geometry.collisionCheckMaxPositionDeviation           = 0.100000000000000;
    parameterSet.geometry.collisionCheckMaxAngleDeviation              = mpsv::math::deg2rad(5.0);
    std::vector<std::array<double,2>> vertices                         = {{3.25,1.65},{-3.25,1.65},{-3.25,-1.65},{3.25,-1.65}};
    parameterSet.geometry.vehicleShape.Create(vertices);
    parameterSet.geometry.skeletalPoints.clear();
    parameterSet.geometry.skeletalPoints.push_back({3.0, 0.0});
    parameterSet.geometry.skeletalPoints.push_back({-3.0, 0.0});

    // metric
    parameterSet.metric.weightPsi                                      = 3.0;
    parameterSet.metric.weightSway                                     = 2.0;
    parameterSet.metric.weightReverseScale                             = 0.0;
    parameterSet.metric.weightReverseDecay                             = 1.0;

    // motion panner
    parameterSet.motionPlanner.sampletime                              = 0.05;
    parameterSet.motionPlanner.samplingRangePosition                   = 5.0;
    parameterSet.motionPlanner.samplingRangeAngle                      = 0.2;
    parameterSet.motionPlanner.maxInputPathLength                      = 50.0;
    parameterSet.motionPlanner.periodGoalSampling                      = 50;
    parameterSet.motionPlanner.regionOfAttraction.rangePose            = {0.25, 0.25, 0.15};
    parameterSet.motionPlanner.regionOfAttraction.rangeUVR             = {0.1, 0.1, 0.01};
    parameterSet.motionPlanner.regionOfAttraction.rangeXYN             = {10.0, 10.0, 10.0};
    parameterSet.motionPlanner.controller.matK                         = {0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35, 0.0, 0.0, 0.0, 0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35, 0.0, 0.0, 0.0, 0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35};
    parameterSet.motionPlanner.controller.maxRadiusX                   = 10.0;
    parameterSet.motionPlanner.controller.maxRadiusY                   = 6.0;
    parameterSet.motionPlanner.controller.maxRadiusPsi                 = 1.0;
    parameterSet.motionPlanner.controller.minRadiusPosition            = 2.0;
    return parameterSet;
}


/**
 * @brief Get an example input data for the motion planner.
 * @return Example input data.
 */
inline mpsv::planner::MotionPlannerInput GetInput(void){
    mpsv::planner::MotionPlannerInput dataIn;
    dataIn.initialStateAndInput = {-56.8924, -29.7461, 0.4302, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    dataIn.originOldToNew = {0.0, 0.0};
    dataIn.initialPath.clear();
    dataIn.initialPath.push_back({-56.8924,-29.7461,0.4302});
    dataIn.initialPath.push_back({-37.5868,-15.6169,0.7787});
    dataIn.initialPath.push_back({-28.4370,-6.3110,0.6899});
    dataIn.initialPath.push_back({-12.0707,3.0823,0.3009});
    dataIn.initialPath.push_back({3.0479,5.0647,-0.0196});
    dataIn.initialPath.push_back({17.3663,0.1339,-0.4240});
    dataIn.initialPath.push_back({32.9519,-9.1796,-0.3449});
    dataIn.initialPath.push_back({42.4645,-13.4085,-0.2170});
    dataIn.initialPath.push_back({53.6586,-12.3645,0.0266});
    dataIn.initialPath.push_back({54.4740,-8.8271,-0.1608});
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-79.9851,-100.0000},{-21.6209,-62.3559},{-18.5046,-60.6264},{-16.1786,-60.0077},{-13.2405,-59.3359},{100.0000,-100.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{100.0000,-100.0000},{-8.6330,-55.8570},{-6.2624,-51.6664},{33.3245,-25.4451},{40.6811,-25.9506},{54.9267,-27.0015}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{100.0000,-100.0000},{76.4063,-10.2128},{76.6623,-9.7605},{81.1169,-1.9088},{81.4029,-1.4088},{82.6338,0.7252},{92.1956,17.3020},{94.3474,21.0324}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-6.3067,70.9910},{-3.4134,48.4085},{-4.5375,45.4551},{-6.2291,42.8809},{-7.7428,40.6061},{-9.2564,38.3778},{-40.4185,54.3090},{-37.5692,67.2133},{-8.0763,72.8402}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{76.4063,-10.2128},{100.0000,-100.0000},{68.3598,-24.2743},{70.7556,-20.0477},{71.0410,-19.5477}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{100.0000,-100.0000},{94.6355,21.5324},{99.9990,30.8508}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-8.6330,-55.8570},{100.0000,-100.0000},{-13.2405,-59.3359}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-21.8216,17.8573},{-49.5448,23.8440},{-44.3695,41.7970},{-40.4185,54.3090},{-10.2803,35.6306},{-13.0293,27.8681},{-14.6987,23.9968},{-16.4461,21.7219},{-19.1728,19.6133}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-32.3158,85.3259},{-8.9443,75.2016},{-8.0763,72.8402},{-37.5692,67.2133}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{100.0000,-100.0000},{54.9267,-27.0015},{61.5932,-27.3341},{64.1195,-27.5203}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{68.3598,-24.2743},{100.0000,-100.0000},{64.1195,-27.5203}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-46.3280,-68.0501},{-43.6236,-67.1454},{-35.9666,-65.3826},{-29.2333,-64.0056},{-79.9851,-100.0000},{-79.0367,-96.2941},{-76.7665,-89.3230},{-74.9859,-85.3385},{-73.2163,-82.0858},{-71.7695,-80.2831},{-71.5804,-80.0636},{-69.5326,-78.0148},{-64.4576,-75.2675},{-59.1490,-72.8728},{-52.6161,-70.1655}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-6.2624,-51.6664},{10.1643,-27.8399},{21.5274,-24.9131},{24.3543,-24.6403},{26.4911,-24.6204},{33.3245,-25.4451}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{15.3565,-19.7760},{10.1643,-27.8399},{-6.1737,-13.9712},{-7.4355,-6.5240},{-5.2166,-3.8473},{-4.0035,-3.6810},{-2.1671,-5.1843}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{54.9609,95.3683},{20.8485,100.0000},{63.5557,100.0000},{63.5307,99.8580},{62.3286,98.1552},{59.2235,96.6320}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-29.2333,-64.0056},{-21.6209,-62.3559},{-79.9851,-100.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-38.5602,2.9441},{-40.3631,3.0506},{-42.4666,3.5428},{-45.2044,4.9264},{-46.5733,6.1835},{-48.8103,9.2500},{-50.3907,12.1568},{-51.2254,13.8597},{-51.4591,15.8885},{-51.2810,17.5315},{-51.0696,19.4006},{-49.5448,23.8440},{-24.0141,15.6024},{-25.4386,13.6068},{-32.5726,4.1680},{-35.8112,3.0306}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-29.8449,93.8135},{-10.5912,88.2057},{-9.8567,86.1104},{-9.1779,82.6382},{-8.9888,77.6627},{-8.9443,75.2016},{-32.3158,85.3259}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-27.5299,98.4630},{-10.0679,99.4937},{-10.8472,90.4740},{-28.1198,97.1061}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-65.6375,-64.6846},{-42.0878,-59.6954},{-46.2835,-65.6887},{-47.7526,-66.2076},{-64.5356,-69.7399},{-65.0536,-67.3635}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{41.7057,90.1537},{20.8485,100.0000},{51.9226,93.9183},{48.9733,92.4750}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-10.2803,35.6306},{-40.4185,54.3090},{-9.2564,38.3778}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{29.7972,83.7683},{27.3153,83.6087},{19.0574,87.2274},{19.3580,93.4600},{19.9146,97.2714},{20.8485,100.0000},{38.3001,88.8966},{35.7181,86.6617}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-21.8216,17.8573},{-24.0141,15.6024},{-49.5448,23.8440}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{21.5274,-24.9131},{10.1643,-27.8399},{15.8565,-20.1923}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{54.9609,95.3683},{51.9226,93.9183},{20.8485,100.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{41.7057,90.1537},{38.3001,88.8966},{20.8485,100.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{74.5699,-6.0555},{69.9179,-7.6320},{50.4303,-5.2574},{50.5194,-4.4658},{50.5973,-3.8273},{50.6974,-3.1355}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-2.8791,49.3996},{-6.3067,70.9910},{-3.5801,68.2838},{-2.1112,50.5171}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{98.4092,30.8207},{86.1890,9.5624},{85.4767,9.9748},{92.6664,24.7080},{93.0003,25.2667}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{100.0000,100.0000},{93.5354,96.4451},{86.7561,100.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-10.5912,88.2057},{-29.8449,93.8135},{-29.0102,95.6028},{-28.1198,97.1061},{-10.8472,90.4740}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{98.8440,93.3454},{93.5354,96.4451},{100.0000,100.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{100.0000,-100.0000},{94.3474,21.0324},{94.6355,21.5324}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{82.4050,7.4339},{66.1339,12.3162},{69.2056,13.3605},{75.9389,11.3517},{76.5287,11.1721}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-0.7200,52.3596},{-2.1112,50.5171},{-3.5801,68.2838},{-0.8200,67.0798},{1.5060,64.7383},{1.6617,57.2153},{0.6601,54.4548}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-40.9525,-64.6510},{-46.2835,-65.6887},{-42.0878,-59.6954}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-27.5299,98.4630},{-27.2346,100.0000},{-10.0301,100.0000},{-10.0679,99.4937}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{27.3153,83.6087},{24.3215,82.0323},{22.0400,80.9415},{20.4373,80.7021},{18.8458,81.1079},{18.0780,83.3029},{19.0574,87.2274}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{83.4178,5.0859},{77.2521,-5.2241},{76.7513,-4.9314},{75.9277,-4.4259},{79.4001,2.1525},{81.7261,6.0969}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-6.1737,-13.9712},{-10.1135,-10.6187},{-7.8510,-7.0240},{-7.4355,-6.5240}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{17.2187,26.0548},{19.6020,26.2316},{14.0929,17.4847},{12.4681,18.5090}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{85.4767,9.9748},{84.3193,8.8307},{84.5641,10.4936},{92.6664,24.7080}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-27.2305,8.4052},{-28.7997,6.5760},{-30.5581,5.1658},{-32.5726,4.1680},{-25.4386,13.6068},{-26.1620,10.5138},{-26.2399,10.3542}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-2.8791,49.3996},{-3.4134,48.4085},{-6.3067,70.9910}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{46.7243,20.1053},{47.5812,17.8770},{40.2804,20.0654},{39.6905,20.2450},{39.2787,22.3403},{39.8797,22.1607}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{32.6901,22.3469},{25.6675,24.4489},{25.3225,26.5242},{31.6885,24.6218},{32.2784,24.4422}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{32.2784,24.4422},{39.2787,22.3403},{39.6905,20.2450},{33.2800,22.1674},{32.6901,22.3469}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{61.8046,15.5888},{68.6157,13.5401},{69.2056,13.3605},{66.1339,12.3162},{65.5329,12.4891},{61.0589,13.8328},{61.2036,15.7617}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{55.1269,15.6154},{54.4703,17.7838},{61.2036,15.7617},{61.0589,13.8328},{60.4579,14.0123}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{82.4050,7.4339},{76.5287,11.1721},{83.2175,9.1633},{84.3193,8.8307},{83.8185,7.0148},{83.2954,7.1612}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{47.3141,19.9323},{53.8693,17.9634},{54.5260,15.7950},{48.1711,17.6974},{47.5812,17.8770},{46.7243,20.1053}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{93.0003,25.2667},{95.9496,30.4083},{96.2723,30.9671},{98.4092,30.8207}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{24.7327,26.7038},{25.3225,26.5242},{25.0665,24.6285},{20.4590,25.9722},{20.4924,26.1052},{20.6927,26.7571}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{1.6617,57.2153},{1.5060,64.7383},{2.9083,61.9712},{2.7524,59.8227}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{99.9998,92.5980},{98.8440,93.3454},{100.0000,100.0000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{75.7163,-8.3437},{71.9657,-7.8847},{69.9179,-7.6320},{74.5699,-6.0555},{75.7163,-6.1952},{75.9611,-6.2285}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{35.7181,86.6617},{33.3030,84.3869},{29.7972,83.7683}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{10.1643,-27.8399},{15.3565,-19.7760},{15.8565,-19.7342},{15.8565,-20.1923}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{21.0043,27.8213},{24.7327,26.7038},{20.6927,26.7571}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{19.6020,26.2316},{17.5332,26.5548},{18.6004,28.2537},{19.9916,26.8502},{20.4924,26.1052}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{70.3297,19.2671},{70.9307,19.0808},{69.2056,13.3605},{68.6157,13.5401}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{41.5937,27.8877},{39.8797,22.1607},{39.2787,22.3403},{41.0038,28.0673}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{23.3526,18.9081},{25.0665,24.6285},{25.6675,24.4489},{23.9424,18.7285}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{48.1711,17.6974},{46.4572,11.9770},{45.8562,12.1566},{47.5812,17.8770}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{52.8120,10.0746},{54.5260,15.7950},{55.1269,15.6154},{53.4019,9.8950}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{56.1843,23.5042},{54.4703,17.7838},{53.8693,17.9634},{55.5944,23.6838}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{58.7440,8.2920},{60.4579,14.0123},{61.0589,13.8328},{59.3450,8.1124}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{61.2036,15.7617},{62.9175,21.4821},{63.5185,21.3092},{61.8046,15.5888}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{64.4200,6.5958},{63.8190,6.7687},{65.5329,12.4891},{66.1339,12.3162}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{26.4466,32.4309},{27.0365,32.2513},{25.3225,26.5242},{24.7327,26.7038}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{32.6901,22.3469},{33.2800,22.1674},{31.5549,16.4469},{30.9762,16.6265}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{39.6905,20.2450},{40.2804,20.0654},{38.5664,14.3450},{37.9766,14.5246}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{77.6528,17.0720},{78.2427,16.8924},{76.5287,11.1721},{75.9389,11.3517}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{32.2784,24.4422},{31.6885,24.6218},{33.4025,30.3356},{33.9923,30.1626}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{46.7243,20.1053},{48.4382,25.8257},{49.0281,25.6527},{47.3141,19.9323}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-46.2835,-65.6887},{-44.8367,-67.5512},{-46.3280,-68.0501},{-47.7526,-66.2076}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{19.4796,29.6705},{20.3144,36.0893},{20.8486,36.0162},{20.0138,29.6040}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{76.6623,-9.7605},{76.4063,-10.2128},{70.5268,-7.8742},{69.9179,-7.6320},{71.9657,-7.8847}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{48.9733,92.4750},{46.5359,91.2777},{41.7057,90.1537}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{75.9277,-4.4259},{75.5827,-4.2264},{79.4001,2.1525}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{93.0003,25.2667},{92.6664,24.7080},{89.1272,26.7567},{89.4389,27.3088}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{95.9496,30.4083},{92.3993,32.4570},{92.7221,33.0158},{96.2723,30.9671}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{46.5350,-3.3617},{50.5973,-3.8273},{50.5194,-4.4658},{46.4571,-3.9936}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{75.7163,-6.1952},{74.5699,-6.0555},{75.9277,-4.4259},{76.7513,-4.9314}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-10.0912,-9.7141},{-7.8510,-7.0240},{-10.1135,-10.6187}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{83.2175,9.1633},{84.5641,10.4936},{84.3193,8.8307}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{98.4092,30.8207},{96.2723,30.9671},{96.7398,31.7786}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{82.6721,5.5315},{81.8486,6.0237},{82.4050,7.4339},{83.2954,7.1612}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{25.6675,24.4489},{25.0665,24.6285},{25.3225,26.5242}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{54.4703,17.7838},{55.1269,15.6154},{54.5260,15.7950},{53.8693,17.9634}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{17.5332,26.5548},{19.6020,26.2316},{17.2187,26.0548},{17.1261,26.0548},{17.1261,26.5548}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{20.2364,27.2227},{19.9916,26.8502},{18.6004,28.2537}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{20.4924,26.1052},{19.9916,26.8502},{20.6927,26.7571}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-40.0015,-58.9809},{-40.5015,-58.9809},{-40.5015,-58.4809},{-40.0015,-58.4809}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-44.6415,-66.9498},{-44.6415,-66.4498},{-44.1415,-66.4498},{-44.1415,-66.9498}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-15.7163,-59.4462},{-15.7164,-58.9462},{-15.2164,-58.9462},{-15.2163,-59.4462}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-33.0455,-63.8033},{-33.5455,-63.8033},{-33.5456,-63.3033},{-33.0456,-63.3033}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-1.9051,-19.3430},{-1.9051,-18.8430},{-1.4051,-18.8430},{-1.4051,-19.3430}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-11.2210,-54.6425},{-11.2210,-55.1425},{-11.7210,-55.1425},{-11.7210,-54.6425}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-67.3232,-65.1608},{-67.3232,-64.6608},{-66.8232,-64.6607},{-66.8232,-65.1607}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-40.9911,-61.8800},{-40.4911,-61.8800},{-40.4911,-62.3800},{-40.9911,-62.3800}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-22.6841,-61.5483},{-23.1841,-61.5483},{-23.1841,-61.0483},{-22.6841,-61.0483}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{4.8727,-25.3162},{4.8727,-24.8162},{5.3727,-24.8162},{5.3727,-25.3162}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{87.5305,10.1704},{87.5305,10.6704},{88.0305,10.6704},{88.0305,10.1704}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{50.7805,-5.3000},{51.1589,-5.3462},{51.1589,-5.8000},{50.6589,-5.8000},{50.6589,-5.3000}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{66.5620,12.0274},{66.5620,11.5274},{66.0620,11.5274},{66.0620,12.0274}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{20.4873,37.2706},{20.9873,37.2706},{20.9873,36.7706},{20.4873,36.7706}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{70.1019,-8.3742},{70.1019,-7.8742},{70.5268,-7.8742},{70.6019,-7.9041},{70.6019,-8.3742}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{19.2185,28.4693},{19.2185,28.9693},{19.7185,28.9693},{19.7185,28.4693}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{73.4065,-14.9726},{72.9065,-14.9726},{72.9065,-14.4726},{73.4065,-14.4726}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-11.4658,-55.6868},{-11.4658,-56.1868},{-11.9658,-56.1868},{-11.9658,-55.6868}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{82.1439,0.7252},{82.1439,1.2252},{82.6439,1.2252},{82.6439,0.7426},{82.6338,0.7252}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{92.9951,20.2487},{93.4951,20.2487},{93.4951,19.7487},{92.9951,19.7487}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-30.3968,-62.9108},{-30.3968,-63.4108},{-30.8968,-63.4109},{-30.8968,-62.9109}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{98.9597,29.5730},{98.4597,29.5730},{98.4597,30.0730},{98.9597,30.0730}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-65.0536,-67.6618},{-65.5536,-67.6618},{-65.5536,-67.1618},{-65.0976,-67.1618},{-65.0536,-67.3635}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-7.8510,-7.0240},{-7.8928,-7.0240},{-7.8928,-6.5240},{-7.4355,-6.5240}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{81.4029,-1.4088},{81.1169,-1.9088},{81.0087,-1.9088},{81.0087,-1.4088}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{94.2416,21.5324},{94.6355,21.5324},{94.3474,21.0324},{94.2416,21.0324}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{70.6473,-19.5477},{71.0410,-19.5477},{70.7556,-20.0477},{70.6473,-20.0477}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{-47.1503,-68.3267},{-47.3014,-68.3267},{-47.3014,-67.8267},{-46.8014,-67.8267},{-46.8014,-68.2094}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{13.3078,17.9797},{13.0305,17.9797},{13.0305,18.1544}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{15.3565,-19.7760},{15.3565,-19.7342},{15.8565,-19.7342}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{91.9692,16.9096},{91.8710,16.7393},{91.8710,16.9096}}));
    dataIn.staticObstacles.push_back(mpsv::geometry::StaticObstacle({{92.1956,17.3020},{92.1492,17.2216},{92.1492,17.3020}}));
    for(auto&& polygon : dataIn.staticObstacles){
        polygon.EnsureCorrectVertexOrder();
    }
    return dataIn;
}


} /* namespace: example */

