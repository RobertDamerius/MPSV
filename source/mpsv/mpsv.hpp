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
#include <mpsv/control/RegionOfAttraction.hpp>
#include <mpsv/control/VehicleSimulator.hpp>

/* Core */
#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/core/Event.hpp>
#include <mpsv/core/FixedSizeTreeSE2.hpp>
#include <mpsv/core/LookUpTable2D.hpp>
#include <mpsv/core/LookUpTable2DScalar.hpp>
#include <mpsv/core/LookUpTable2DVector.hpp>
#include <mpsv/core/PerformanceCounter.hpp>
#include <mpsv/core/Time.hpp>
#include <mpsv/core/WorkerThread.hpp>

/* Geometry */
#include <mpsv/geometry/AABB.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/DouglasPeucker.hpp>
#include <mpsv/geometry/MovingObstacle.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/geometry/PathSmoother.hpp>
#include <mpsv/geometry/QuickHull2D.hpp>
#include <mpsv/geometry/StaticObstacle.hpp>
#include <mpsv/geometry/ConvexVehicleShape.hpp>
#include <mpsv/geometry/VehicleShape.hpp>

/* Math */
#include <mpsv/math/Additional.hpp>
#include <mpsv/math/Metric.hpp>
#include <mpsv/math/WGS84.hpp>


/* Planner */
#include <mpsv/planner/AsyncOnlinePlanner.hpp>
#include <mpsv/planner/AsyncOnlinePlannerInput.hpp>
#include <mpsv/planner/AsyncOnlinePlannerOutput.hpp>
#include <mpsv/planner/AsyncOnlinePlannerParameterSet.hpp>
#include <mpsv/planner/CostMap.hpp>
#include <mpsv/planner/MotionPlanner.hpp>
#include <mpsv/planner/MotionPlannerInput.hpp>
#include <mpsv/planner/MotionPlannerOutput.hpp>
#include <mpsv/planner/MotionPlannerParameterSet.hpp>
#include <mpsv/planner/MotionPlannerState.hpp>
#include <mpsv/planner/MotionPlannerTree.hpp>
#include <mpsv/planner/ParameterTypes.hpp>
#include <mpsv/planner/PathPlanner.hpp>
#include <mpsv/planner/PathPlannerInput.hpp>
#include <mpsv/planner/PathPlannerOutput.hpp>
#include <mpsv/planner/PathPlannerParameterSet.hpp>
#include <mpsv/planner/PathPlannerState.hpp>
#include <mpsv/planner/PathPlannerTree.hpp>
#include <mpsv/planner/OnlinePlanner.hpp>
#include <mpsv/planner/OnlinePlannerInput.hpp>
#include <mpsv/planner/OnlinePlannerOutput.hpp>
#include <mpsv/planner/OnlinePlannerParameterSet.hpp>
#include <mpsv/planner/SequentialPlanner.hpp>
#include <mpsv/planner/SequentialPlannerInput.hpp>
#include <mpsv/planner/SequentialPlannerOutput.hpp>
#include <mpsv/planner/SequentialPlannerParameterSet.hpp>
#include <mpsv/planner/Serialization.hpp>

/* Sampler */
#include <mpsv/sampler/HaltonSequence.hpp>
#include <mpsv/sampler/InformedSamplerSE2.hpp>
#include <mpsv/sampler/PathSampler.hpp>
#include <mpsv/sampler/UniformBallSampler3D.hpp>
#include <mpsv/sampler/UniformBoxSamplerSE2.hpp>

