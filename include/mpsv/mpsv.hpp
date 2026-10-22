/**
 * @file mpsv.hpp
 * @author Robert Damerius (damerius.mail@gmail.com)
 * @brief This is the main header file for the MPSV library. Include this header to include all features.
 * @date 2026-10-19
 * 
 * @copyright Copyright (c) 2026
 */
#pragma once


/* Common */
#include <mpsv/common/Common.hpp>
#include <mpsv/common/ErrorCode.hpp>

/* Structures */
#include <mpsv/structures/FixedSizeTreeSE2.hpp>
#include <mpsv/structures/CostMap.hpp>
#include <mpsv/structures/LookUpTable2D.hpp>
#include <mpsv/structures/LookUpTable2DScalar.hpp>
#include <mpsv/structures/LookUpTable2DVector.hpp>

/* Runtime */
#include <mpsv/runtime/Event.hpp>
#include <mpsv/runtime/PerformanceCounter.hpp>
#include <mpsv/runtime/Time.hpp>
#include <mpsv/runtime/WorkerThread.hpp>

/* Control */
#include <mpsv/control/RegionOfAttraction.hpp>
#include <mpsv/control/VehicleSimulator.hpp>

/* Geometry */
#include <mpsv/geometry/AABB.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>
#include <mpsv/geometry/DouglasPeucker.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
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
#include <mpsv/sampler/PathSampler.hpp>
#include <mpsv/sampler/UniformBoxSamplerSE2.hpp>

/* Logging */
#include <mpsv/logging/File.hpp>
