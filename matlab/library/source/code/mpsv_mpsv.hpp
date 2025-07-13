/**
 * @file mpsv.hpp
 * @author Robert Damerius (damerius.mail@gmail.com)
 * @brief This is the main header file for the MPSV library. Include this header to include all features.
 * @date 2023-11-01
 * @note Define a global compilation macro MPSV_DONT_USE_OMP if you don't want to use OpenMP parallelization.
 * 
 * @copyright Copyright (c) 2023
 */
#pragma once


/* Control */
#include <mpsv_control_RegionOfAttraction.hpp>
#include <mpsv_control_VehicleSimulator.hpp>

/* Core */
#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_core_DataLogFile.hpp>
#include <mpsv_core_Event.hpp>
#include <mpsv_core_FixedSizeTreeSE2.hpp>
#include <mpsv_core_LookUpTable2D.hpp>
#include <mpsv_core_LookUpTable2DScalar.hpp>
#include <mpsv_core_LookUpTable2DVector.hpp>
#include <mpsv_core_PerformanceCounter.hpp>
#include <mpsv_core_Time.hpp>
#include <mpsv_core_WorkerThread.hpp>

/* Geometry */
#include <mpsv_geometry_AABB.hpp>
#include <mpsv_geometry_ConvexPolygon.hpp>
#include <mpsv_geometry_DouglasPeucker.hpp>
#include <mpsv_geometry_MovingObstacle.hpp>
#include <mpsv_geometry_OrientedBox.hpp>
#include <mpsv_geometry_PathSmoother.hpp>
#include <mpsv_geometry_QuickHull2D.hpp>
#include <mpsv_geometry_StaticObstacle.hpp>
#include <mpsv_geometry_ConvexVehicleShape.hpp>
#include <mpsv_geometry_VehicleShape.hpp>

/* Math */
#include <mpsv_math_Additional.hpp>
#include <mpsv_math_Metric.hpp>
#include <mpsv_math_WGS84.hpp>


/* Planner */
#include <mpsv_planner_AsyncOnlinePlanner.hpp>
#include <mpsv_planner_AsyncOnlinePlannerInput.hpp>
#include <mpsv_planner_AsyncOnlinePlannerOutput.hpp>
#include <mpsv_planner_AsyncOnlinePlannerParameterSet.hpp>
#include <mpsv_planner_CostMap.hpp>
#include <mpsv_planner_MotionPlanner.hpp>
#include <mpsv_planner_MotionPlannerInput.hpp>
#include <mpsv_planner_MotionPlannerOutput.hpp>
#include <mpsv_planner_MotionPlannerParameterSet.hpp>
#include <mpsv_planner_MotionPlannerState.hpp>
#include <mpsv_planner_MotionPlannerTree.hpp>
#include <mpsv_planner_ParameterTypes.hpp>
#include <mpsv_planner_PathPlanner.hpp>
#include <mpsv_planner_PathPlannerInput.hpp>
#include <mpsv_planner_PathPlannerOutput.hpp>
#include <mpsv_planner_PathPlannerParameterSet.hpp>
#include <mpsv_planner_PathPlannerState.hpp>
#include <mpsv_planner_PathPlannerTree.hpp>
#include <mpsv_planner_OnlinePlanner.hpp>
#include <mpsv_planner_OnlinePlannerInput.hpp>
#include <mpsv_planner_OnlinePlannerOutput.hpp>
#include <mpsv_planner_OnlinePlannerParameterSet.hpp>
#include <mpsv_planner_SequentialPlanner.hpp>
#include <mpsv_planner_SequentialPlannerInput.hpp>
#include <mpsv_planner_SequentialPlannerOutput.hpp>
#include <mpsv_planner_SequentialPlannerParameterSet.hpp>
#include <mpsv_planner_Serialization.hpp>

/* Sampler */
#include <mpsv_sampler_HaltonSequence.hpp>
#include <mpsv_sampler_InformedSamplerSE2.hpp>
#include <mpsv_sampler_PathSampler.hpp>
#include <mpsv_sampler_UniformBallSampler3D.hpp>
#include <mpsv_sampler_UniformBoxSamplerSE2.hpp>

