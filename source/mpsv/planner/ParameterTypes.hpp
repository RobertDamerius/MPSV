#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/geometry/VehicleShape.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/core/ErrorCode.hpp>


namespace mpsv {


namespace planner {


/**
 * @brief Parameter for the nonlinear dynamical motion model of the vehicle.
 */
class ParameterModel {
    public:
        std::array<double,36> matF;                   // 3-by-12 coefficient matrix (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double,9> matB;                    // 3-by-3 input matrix B (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double,3> vecTimeconstantsXYN;     // Timeconstants {TX, TY, TN} for input force dynamics.
        std::array<double,3> vecTimeconstantsInput;   // Timeconstants {Tf1, Tf2, Tf3} for input filter dynamics.
        std::array<double,3> lowerLimitXYN;           // Lower saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
        std::array<double,3> upperLimitXYN;           // Upper saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.

        /**
         * @brief Construct a new model parameter object and set default values.
         */
        ParameterModel() noexcept { Clear(); }

        /**
         * @brief Clear the model parameter and set default values.
         */
        void Clear(void) noexcept {
            matF                  = {-0.031604977390302, 0.0, 0.0, 0.233741474945168, 0.0, 0.0, 0.0, 0.0, 0.133462292221887, -0.020792892088185, 0.0, 0.0, 0.0, -0.041708610292712, 0.017242263124379, 0.0, -0.251243228165994, -0.000476044356140, 0.0, 0.0, 0.005297185500344, 0.0, -0.167981638498434, 0.497442136687157, 0.0, 0.000017966020615, -0.023528504195578, 0.0, 0.000108223241795, 0.000649602175186, 0.0, 0.0, -0.000002281767319, 0.0, 0.000072358238719, -0.678801229030825};
            matB                  = {0.000237948834266, -0.000004551592718, 0.000010003488944, -0.000009313932115, 0.000215194147058, -0.000024957572224, -0.000002202124158, -0.000002930260852, 0.000043018345190};
            vecTimeconstantsXYN   = {0.2, 0.2, 0.2};
            vecTimeconstantsInput = {0.5, 0.5, 0.5};
            lowerLimitXYN         = {-630.0, -495.0, -675.0};
            upperLimitXYN         = {630.0, 495.0, 675.0};
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(std::isfinite(matF[0]) &&
                 std::isfinite(matF[1]) &&
                 std::isfinite(matF[2]) &&
                 std::isfinite(matF[3]) &&
                 std::isfinite(matF[4]) &&
                 std::isfinite(matF[5]) &&
                 std::isfinite(matF[6]) &&
                 std::isfinite(matF[7]) &&
                 std::isfinite(matF[8]) &&
                 std::isfinite(matF[9]) &&
                 std::isfinite(matF[10]) &&
                 std::isfinite(matF[11]) &&
                 std::isfinite(matF[12]) &&
                 std::isfinite(matF[13]) &&
                 std::isfinite(matF[14]) &&
                 std::isfinite(matF[15]) &&
                 std::isfinite(matF[16]) &&
                 std::isfinite(matF[17]) &&
                 std::isfinite(matF[18]) &&
                 std::isfinite(matF[19]) &&
                 std::isfinite(matF[20]) &&
                 std::isfinite(matF[21]) &&
                 std::isfinite(matF[22]) &&
                 std::isfinite(matF[23]) &&
                 std::isfinite(matF[24]) &&
                 std::isfinite(matF[25]) &&
                 std::isfinite(matF[26]) &&
                 std::isfinite(matF[27]) &&
                 std::isfinite(matF[28]) &&
                 std::isfinite(matF[29]) &&
                 std::isfinite(matF[30]) &&
                 std::isfinite(matF[31]) &&
                 std::isfinite(matF[32]) &&
                 std::isfinite(matF[33]) &&
                 std::isfinite(matF[34]) &&
                 std::isfinite(matF[35])))
                return error_code::MODEL_MATF;
            if(!(std::isfinite(matB[0]) &&
                 std::isfinite(matB[1]) &&
                 std::isfinite(matB[2]) &&
                 std::isfinite(matB[3]) &&
                 std::isfinite(matB[4]) &&
                 std::isfinite(matB[5]) &&
                 std::isfinite(matB[6]) &&
                 std::isfinite(matB[7]) &&
                 std::isfinite(matB[8])))
                return error_code::MODEL_MATB;
            if(!(std::isfinite(vecTimeconstantsXYN[0]) &&
                 std::isfinite(vecTimeconstantsXYN[1]) &&
                 std::isfinite(vecTimeconstantsXYN[2]) &&
                 (vecTimeconstantsXYN[0] > 0.0) &&
                 (vecTimeconstantsXYN[1] > 0.0) &&
                 (vecTimeconstantsXYN[2] > 0.0)))
                return error_code::MODEL_VEC_TIMECONSTANTS_XYN;
            if(!(std::isfinite(vecTimeconstantsInput[0]) &&
                 std::isfinite(vecTimeconstantsInput[1]) &&
                 std::isfinite(vecTimeconstantsInput[2]) &&
                 (vecTimeconstantsInput[0] > 0.0) &&
                 (vecTimeconstantsInput[1] > 0.0) &&
                 (vecTimeconstantsInput[2] > 0.0)))
                return error_code::MODEL_VEC_TIMECONSTANTS_INPUT;
            if(!(std::isfinite(lowerLimitXYN[0]) &&
                 std::isfinite(lowerLimitXYN[1]) &&
                 std::isfinite(lowerLimitXYN[2])))
                return error_code::MODEL_LOWER_LIMIT_XYN;
            if(!(std::isfinite(upperLimitXYN[0]) &&
                 std::isfinite(upperLimitXYN[1]) &&
                 std::isfinite(upperLimitXYN[2])))
                return error_code::MODEL_UPPER_LIMIT_XYN;
            if(!((upperLimitXYN[0] >= lowerLimitXYN[0]) &&
                 (upperLimitXYN[1] >= lowerLimitXYN[1]) &&
                 (upperLimitXYN[2] >= lowerLimitXYN[2])))
                return error_code::MODEL_LOWER_GREATER_UPPER_LIMIT;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            std::array<double,36> tmp;
            tmp[0]  = matF[0];  tmp[1]  = matF[12]; tmp[2]  = matF[24];
            tmp[3]  = matF[1];  tmp[4]  = matF[13]; tmp[5]  = matF[25];
            tmp[6]  = matF[2];  tmp[7]  = matF[14]; tmp[8]  = matF[26];
            tmp[9]  = matF[3];  tmp[10] = matF[15]; tmp[11] = matF[27];
            tmp[12] = matF[4];  tmp[13] = matF[16]; tmp[14] = matF[28];
            tmp[15] = matF[5];  tmp[16] = matF[17]; tmp[17] = matF[29];
            tmp[18] = matF[6];  tmp[19] = matF[18]; tmp[20] = matF[30];
            tmp[21] = matF[7];  tmp[22] = matF[19]; tmp[23] = matF[31];
            tmp[24] = matF[8];  tmp[25] = matF[20]; tmp[26] = matF[32];
            tmp[27] = matF[9];  tmp[28] = matF[21]; tmp[29] = matF[33];
            tmp[30] = matF[10]; tmp[31] = matF[22]; tmp[32] = matF[34];
            tmp[33] = matF[11]; tmp[34] = matF[23]; tmp[35] = matF[35];
            file.WriteField("double", preString + "matF", {3,12}, &tmp[0], sizeof(matF));
            file.WriteField("double", preString + "vecTimeconstantsXYN", {3,1}, &vecTimeconstantsXYN[0], sizeof(vecTimeconstantsXYN));
            file.WriteField("double", preString + "vecTimeconstantsInput", {3,1}, &vecTimeconstantsInput[0], sizeof(vecTimeconstantsInput));
            tmp[0] = matB[0]; tmp[1] = matB[3]; tmp[2] = matB[6];
            tmp[3] = matB[1]; tmp[4] = matB[4]; tmp[5] = matB[7];
            tmp[6] = matB[2]; tmp[7] = matB[5]; tmp[8] = matB[8];
            file.WriteField("double", preString + "matB", {3,3}, &matB[0], sizeof(matB));
            file.WriteField("double", preString + "lowerLimitXYN", {3,1}, &lowerLimitXYN[0], sizeof(lowerLimitXYN));
            file.WriteField("double", preString + "upperLimitXYN", {3,1}, &upperLimitXYN[0], sizeof(upperLimitXYN));
        }
};


/**
 * @brief Parameter for the cost map (2D look-up table with additional cost values).
 */
class ParameterCostMap {
    public:
        int32_t modBreakpoints;    // A modulo factor (> 0) that indicates when to calculate the cost using the objective function and when to do bilinear interpolation.
        double resolution;         // Resolution (> 1e-3) of the grid map (dimension of one cell).
        double distanceScale;      // Scale factor (>= 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
        double distanceDecay;      // Decay factor (> 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).

        /**
         * @brief Construct a new costmap parameter object and set default values.
         */
        ParameterCostMap() noexcept { Clear(); }

        /**
         * @brief Clear the costmap parameter and set default values.
         */
        void Clear(void) noexcept {
            modBreakpoints = 10;
            resolution     = 0.1;
            distanceScale  = 5.0;
            distanceDecay  = 0.02;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(modBreakpoints > 0))
                return error_code::COSTMAP_MOD_BREAKPOINTS;
            if(!(std::isfinite(resolution) && (resolution > 1e-3)))
                return error_code::COSTMAP_RESOLUTION;
            if(!(std::isfinite(distanceScale) && (distanceScale >= 0.0)))
                return error_code::COSTMAP_DISTANCE_SCALE;
            if(!(std::isfinite(distanceDecay) && (distanceDecay > 0.0)))
                return error_code::COSTMAP_DISTANCE_DECAY;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("int32_t", preString + "modBreakpoints", {1}, &modBreakpoints, sizeof(modBreakpoints));
            file.WriteField("double", preString + "resolution", {1}, &resolution, sizeof(resolution));
            file.WriteField("double", preString + "distanceScale", {1}, &distanceScale, sizeof(distanceScale));
            file.WriteField("double", preString + "distanceDecay", {1}, &distanceDecay, sizeof(distanceDecay));
        }
};


/**
 * @brief Additional parameter for the distance metric function.
 */
class ParameterMetric {
    public:
        double weightPsi;            // Weighting (> 0) for heading angle (psi) in distance metric function.
        double weightSway;           // Weighting (>= 0) for sway movement (heading angle with respect to perpenticular direction of movement).
        double weightReverseScale;   // Weighting (>= 0) for sway and reverse movement (heading angle with respect to line angle).
        double weightReverseDecay;   // Decay factor (> 0) for the weighting function that weights sway and reverse movement.

        /**
         * @brief Construct a new metric parameter object and set default values.
         */
        ParameterMetric() noexcept { Clear(); }

        /**
         * @brief Clear the metric parameter and set default values.
         */
        void Clear(void) noexcept {
            weightPsi  = 3.0;
            weightSway = 2.0;
            weightReverseScale = 0.0;
            weightReverseDecay = 1.0;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(std::isfinite(weightPsi) && (weightPsi > 0.0)))
                return error_code::METRIC_WEIGHT_PSI;
            if(!(std::isfinite(weightSway) && (weightSway >= 0.0)))
                return error_code::METRIC_WEIGHT_SWAY;
            if(!(std::isfinite(weightReverseScale) && (weightReverseScale >= 0.0)))
                return error_code::METRIC_WEIGHT_REVERSE_SCALE;
            if(!(std::isfinite(weightReverseDecay) && (weightReverseDecay > 0.0)))
                return error_code::METRIC_WEIGHT_REVERSE_DECAY;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "weightPsi", {1}, &weightPsi, sizeof(weightPsi));
            file.WriteField("double", preString + "weightSway", {1}, &weightSway, sizeof(weightSway));
            file.WriteField("double", preString + "weightReverseScale", {1}, &weightReverseScale, sizeof(weightReverseScale));
            file.WriteField("double", preString + "weightReverseDecay", {1}, &weightReverseDecay, sizeof(weightReverseDecay));
        }
};


/**
 * @brief Geometry parameter.
 */
class ParameterGeometry {
    public:
        double collisionCheckMaxPositionDeviation;          // Maximum position deviation for path subdivision during collision checking. Must be at least 0.01 meters.
        double collisionCheckMaxAngleDeviation;             // Maximum angle deviation for path subdivision during collision checking. Must be at least 1 degree (0.0174532925199433 radians).
        mpsv::geometry::VehicleShape vehicleShape;          // Static shape of the vehicle to be used for path planning described by several convex polyons.
        std::vector<std::array<double,2>> skeletalPoints;   // Skeletal points (b-frame) at which the cost map is to be evaluated.

        /**
         * @brief Construct a new geometry parameter object and set default values.
         */
        ParameterGeometry() noexcept { Clear(); }

        /**
         * @brief Clear the geometry parameter and set default values.
         */
        void Clear(void) noexcept {
            collisionCheckMaxPositionDeviation = 0.1;
            collisionCheckMaxAngleDeviation    = mpsv::math::deg2rad(5.0);
            vehicleShape.Clear();
            skeletalPoints.clear();
        }

        /**
         * @brief Check whether this parameter is valid. Vertices of @ref vehicleShape might be reordered.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         * @details The vertex order of polygon data may be adjusted if possible.
         */
        error_code IsValid(void) noexcept {
            constexpr double minPositionDeviation = 0.01; // Minimum position deviation for path subdivision during collision checking.
            constexpr double minAngleDeviation = 0.0174532925199433; // Minimum angle deviation for path subdivision during collision checking (= 1 deg).
            if(!(std::isfinite(collisionCheckMaxPositionDeviation) && (collisionCheckMaxPositionDeviation >= minPositionDeviation)))
                return error_code::GEOMETRY_MAX_POSITION_DEVIATION;
            if(!(std::isfinite(collisionCheckMaxAngleDeviation) && (collisionCheckMaxAngleDeviation >= minAngleDeviation)))
                return error_code::GEOMETRY_MAX_ANGLE_DEVIATION;
            if(!(vehicleShape.IsFinite() && vehicleShape.EnsureCorrectVertexOrder()))
                return error_code::GEOMETRY_VEHICLE_SHAPE;
            if(skeletalPoints.empty())
                return error_code::GEOMETRY_SKELETAL_POINTS;
            for(auto&& p : skeletalPoints){
                if(!(std::isfinite(p[0]) && std::isfinite(p[1])))
                    return error_code::GEOMETRY_SKELETAL_POINTS;
            }
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "collisionCheckMaxPositionDeviation", {1}, &collisionCheckMaxPositionDeviation, sizeof(collisionCheckMaxPositionDeviation));
            file.WriteField("double", preString + "collisionCheckMaxAngleDeviation", {1}, &collisionCheckMaxAngleDeviation, sizeof(collisionCheckMaxAngleDeviation));
            file.WriteConvexPolygonsField(preString + "vehicleShape", vehicleShape.convexVehicleShapes);
            file.WriteVectorField(preString + "skeletalPoints", skeletalPoints);
        }
};


/**
 * @brief Parameter for the path planner.
 */
class ParameterPathPlanner {
    public:
        uint32_t periodGoalSampling;        // Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.

        /**
         * @brief Construct a new path planner parameter object and set default values.
         */
        ParameterPathPlanner() noexcept { Clear(); }

        /**
         * @brief Clear the path planner parameters and set default values.
         */
        void Clear(void) noexcept {
            periodGoalSampling = 50;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(periodGoalSampling > 0))
                return error_code::PATHPLANNER_PERIOD_GOAL_SAMPLING;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("uint32_t", preString + "periodGoalSampling", {1}, &periodGoalSampling, sizeof(periodGoalSampling));
        }
};


/**
 * @brief Parameter for the controller inside the motion planner.
 */
class ParameterController {
    public:
        double maxRadiusX;            // Maximum look-ahead distance for longitudinal distance during pose control.
        double maxRadiusY;            // Maximum look-ahead distance for lateral distance during pose control.
        double maxRadiusPsi;          // Maximum look-ahead distance for angular distance during pose control.
        double minRadiusPosition;     // Minimum look-ahead distance for position during pose control. The radius is limited by the guidance law according to nearby obstacles but is never lower than this value.
        std::array<double,36> matK;   // 3-by-12 control gain matrix (row-major order) for pose control.

        /**
         * @brief Construct a new controller parameter object and set default values.
         */
        ParameterController() noexcept { Clear(); }

        /**
         * @brief Clear the controller parameters and set default values.
         */
        void Clear(void) noexcept {
            matK                = {0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35, 0.0, 0.0, 0.0, 0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35, 0.0, 0.0, 0.0, 0.054, 0.0, 0.0, 0.531, 0.0, 0.0, 1.785, 0.0, 0.0, 2.35};
            maxRadiusX          = 10.0;
            maxRadiusY          = 6.0;
            maxRadiusPsi        = 1.0;
            minRadiusPosition   = 2.0;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(std::isfinite(matK[0]) &&
                 std::isfinite(matK[1]) &&
                 std::isfinite(matK[2]) &&
                 std::isfinite(matK[3]) &&
                 std::isfinite(matK[4]) &&
                 std::isfinite(matK[5]) &&
                 std::isfinite(matK[6]) &&
                 std::isfinite(matK[7]) &&
                 std::isfinite(matK[8]) &&
                 std::isfinite(matK[9]) &&
                 std::isfinite(matK[10]) &&
                 std::isfinite(matK[11]) &&
                 std::isfinite(matK[12]) &&
                 std::isfinite(matK[13]) &&
                 std::isfinite(matK[14]) &&
                 std::isfinite(matK[15]) &&
                 std::isfinite(matK[16]) &&
                 std::isfinite(matK[17]) &&
                 std::isfinite(matK[18]) &&
                 std::isfinite(matK[19]) &&
                 std::isfinite(matK[20]) &&
                 std::isfinite(matK[21]) &&
                 std::isfinite(matK[22]) &&
                 std::isfinite(matK[23]) &&
                 std::isfinite(matK[24]) &&
                 std::isfinite(matK[25]) &&
                 std::isfinite(matK[26]) &&
                 std::isfinite(matK[27]) &&
                 std::isfinite(matK[28]) &&
                 std::isfinite(matK[29]) &&
                 std::isfinite(matK[30]) &&
                 std::isfinite(matK[31]) &&
                 std::isfinite(matK[32]) &&
                 std::isfinite(matK[33]) &&
                 std::isfinite(matK[34]) &&
                 std::isfinite(matK[35])))
                return error_code::CONTROLLER_MAT_K;
            if(!(std::isfinite(maxRadiusX) && (maxRadiusX > 0.0)))
                return error_code::CONTROLLER_MAX_RADIUS_X;
            if(!(std::isfinite(maxRadiusY) && (maxRadiusY > 0.0)))
                return error_code::CONTROLLER_MAX_RADIUS_Y;
            if(!(std::isfinite(maxRadiusPsi) && (maxRadiusPsi > 0.0)))
                return error_code::CONTROLLER_MAX_RADIUS_PSI;
            if(!(std::isfinite(minRadiusPosition) && (minRadiusPosition > 0.0)))
                return error_code::CONTROLLER_MIN_RADIUS_POSITION;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            std::array<double,36> tmp;
            tmp[0]  = matK[0];  tmp[1]  = matK[12]; tmp[2]  = matK[24];
            tmp[3]  = matK[1];  tmp[4]  = matK[13]; tmp[5]  = matK[25];
            tmp[6]  = matK[2];  tmp[7]  = matK[14]; tmp[8]  = matK[26];
            tmp[9]  = matK[3];  tmp[10] = matK[15]; tmp[11] = matK[27];
            tmp[12] = matK[4];  tmp[13] = matK[16]; tmp[14] = matK[28];
            tmp[15] = matK[5];  tmp[16] = matK[17]; tmp[17] = matK[29];
            tmp[18] = matK[6];  tmp[19] = matK[18]; tmp[20] = matK[30];
            tmp[21] = matK[7];  tmp[22] = matK[19]; tmp[23] = matK[31];
            tmp[24] = matK[8];  tmp[25] = matK[20]; tmp[26] = matK[32];
            tmp[27] = matK[9];  tmp[28] = matK[21]; tmp[29] = matK[33];
            tmp[30] = matK[10]; tmp[31] = matK[22]; tmp[32] = matK[34];
            tmp[33] = matK[11]; tmp[34] = matK[23]; tmp[35] = matK[35];
            file.WriteField("double", preString + "matK", {3,12}, &tmp[0], sizeof(matK));
            file.WriteField("double", preString + "maxRadiusX", {1}, &maxRadiusX, sizeof(maxRadiusX));
            file.WriteField("double", preString + "maxRadiusY", {1}, &maxRadiusY, sizeof(maxRadiusY));
            file.WriteField("double", preString + "maxRadiusPsi", {1}, &maxRadiusPsi, sizeof(maxRadiusPsi));
            file.WriteField("double", preString + "minRadiusPosition", {1}, &minRadiusPosition, sizeof(minRadiusPosition));
        }
};


/**
 * @brief Parameter for the region of attraction inside the motion planner.
 */
class ParameterRegionOfAttraction {
    public:
        std::array<double,3> rangePose;   // Pose box constraints for the region of attraction.
        std::array<double,3> rangeUVR;    // Velocity box constraints for the region of attraction.
        std::array<double,3> rangeXYN;    // The force range {dX,dY,dN}. A given force must be in this range {[-dX,dX],[-dY,dY],[-dN,dN]} to be in the region of attraction.

        /**
         * @brief Construct a new region of attraction parameter object and set default values.
         */
        ParameterRegionOfAttraction() noexcept { Clear(); }

        /**
         * @brief Clear the region of attraction parameters and set default values.
         */
        void Clear(void) noexcept {
            rangePose = {0.25, 0.25, 0.15};
            rangeUVR  = {0.1, 0.1, 0.01};
            rangeXYN  = {10.0, 10.0, 10.0};
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(std::isfinite(rangePose[0]) &&
                 std::isfinite(rangePose[1]) &&
                 std::isfinite(rangePose[2]) &&
                 (rangePose[0] > 0.0) &&
                 (rangePose[1] > 0.0) &&
                 (rangePose[2] > 0.0)))
                return error_code::REGION_OF_ATTRACTION_RANGE_POSE;
            if(!(std::isfinite(rangeUVR[0]) &&
                 std::isfinite(rangeUVR[1]) &&
                 std::isfinite(rangeUVR[2]) &&
                 (rangeUVR[0] > 0.0) &&
                 (rangeUVR[1] > 0.0) &&
                 (rangeUVR[2] > 0.0)))
                return error_code::REGION_OF_ATTRACTION_RANGE_UVR;
            if(!(std::isfinite(rangeXYN[0]) &&
                 std::isfinite(rangeXYN[1]) &&
                 std::isfinite(rangeXYN[2]) &&
                 (rangeXYN[0] > 0.0) &&
                 (rangeXYN[1] > 0.0) &&
                 (rangeXYN[2] > 0.0)))
                return error_code::REGION_OF_ATTRACTION_RANGE_XYN;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "rangePose", {3,1}, &rangePose[0], sizeof(rangePose));
            file.WriteField("double", preString + "rangeUVR", {3,1}, &rangeUVR[0], sizeof(rangeUVR));
            file.WriteField("double", preString + "rangeXYN", {3,1}, &rangeXYN[0], sizeof(rangeXYN));
        }
};


/**
 * @brief Parameter for the motion planner.
 */
class ParameterMotionPlanner {
    public:
        double samplingRangePosition;     // Range in meters for sampling the position around a given path.
        double samplingRangeAngle;        // Range in radians for sampling the angle around a given path.
        uint32_t periodGoalSampling;      // Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
        double sampletime;                // Sampletime to be used for fixed-step trajectory simulation.
        double maxInputPathLength;        // Maximum length (> 0) of the input path (x,y only). The input path is trimmed to ensure this maximum length. The trimmed pose may be interpolated.
        ParameterController controller;
        ParameterRegionOfAttraction regionOfAttraction;

        /**
         * @brief Construct a new motion planner parameter object and set default values.
         */
        ParameterMotionPlanner() noexcept { Clear(); }

        /**
         * @brief Clear the motion planner parameters and set default values.
         */
        void Clear(void) noexcept {
            samplingRangePosition = 10.0;
            samplingRangeAngle    = 1.0;
            periodGoalSampling    = 50;
            sampletime            = 0.05;
            maxInputPathLength    = 50.0;
            controller.Clear();
            regionOfAttraction.Clear();
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(std::isfinite(samplingRangePosition) && (samplingRangePosition > 0.0)))
                return error_code::MOTIONPLANNER_SAMPLING_RANGE_POSITION;
            if(!(std::isfinite(samplingRangeAngle) && (samplingRangeAngle > 0.0)))
                return error_code::MOTIONPLANNER_SAMPLING_RANGE_ANGLE;
            if(!((periodGoalSampling > 0)))
                return error_code::MOTIONPLANNER_PERIOD_GOAL_SAMPLING;
            if(!(std::isfinite(sampletime) && (sampletime > 0.0)))
                return error_code::MOTIONPLANNER_SAMPLETIME;
            if(!(std::isfinite(maxInputPathLength) && (maxInputPathLength > 0.0)))
                return error_code::MOTIONPLANNER_MAX_INPUT_PATH_LENGTH;
            error_code e = controller.IsValid();
            if(error_code::NONE != e)
                return e;
            return regionOfAttraction.IsValid();
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "samplingRangePosition", {1}, &samplingRangePosition, sizeof(samplingRangePosition));
            file.WriteField("double", preString + "samplingRangeAngle", {1}, &samplingRangeAngle, sizeof(samplingRangeAngle));
            file.WriteField("uint32_t", preString + "periodGoalSampling", {1}, &periodGoalSampling, sizeof(periodGoalSampling));
            file.WriteField("double", preString + "sampletime", {1}, &sampletime, sizeof(sampletime));
            file.WriteField("double", preString + "maxInputPathLength", {1}, &maxInputPathLength, sizeof(maxInputPathLength));
            controller.WriteToFile(file, preString + "controller.");
            regionOfAttraction.WriteToFile(file, preString + "regionOfAttraction.");
        }
};


/**
 * @brief Parameter for the online planner.
 */
class ParameterOnlinePlanner {
    public:
        bool predictInitialStateOnReset;          // True if the initial state should be predicted by the expected computation time after a reset of the online planner (e.g. the first solve).
        double maxComputationTimePathOnReset;     // Maximum computation time in seconds for path planning after a reset.
        double maxComputationTimeMotionOnReset;   // Maximum computation time in seconds for motion planning after a reset.
        double maxComputationTimePath;            // Maximum computation time in seconds for path planning.
        double maxComputationTimeMotion;          // Maximum computation time in seconds for motion planning.
        double additionalAheadPlanningTime;       // Additional time added to the estimated computation time in seconds to obtain the future timepoint from where to start the next planning problem. This value must be greater than zero.
        double additionalTrajectoryDuration;      // Additional time added to the ahead planning time (= estimated computation time (no reset) + additional ahead planning time) to obtain the minimum time duration of a trajectory to make sure that the next solve operation can access a future point of the trajectory. This value must be greater than zero.
        double timeKeepPastTrajectory;            // Time in seconds to keep from a past trajectory. The past data of the previous trajectory is inserted at the beginning of a new solution. Inserting past data helps to handle imperfect time synchronization between this trajectory generator and the user of the trajectory data.

        /**
         * @brief Construct a new online planner parameter object and set default values.
         */
        ParameterOnlinePlanner() noexcept { Clear(); }

        /**
         * @brief Clear the online planner parameters and set default values.
         */
        void Clear(void) noexcept {
            predictInitialStateOnReset      = true;
            maxComputationTimePathOnReset   = 0.1;
            maxComputationTimeMotionOnReset = 0.2;
            maxComputationTimePath          = 0.3;
            maxComputationTimeMotion        = 0.6;
            additionalAheadPlanningTime     = 0.2;
            additionalTrajectoryDuration    = 1.0;
            timeKeepPastTrajectory          = 1.0;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        error_code IsValid(void) const noexcept {
            if(!(std::isfinite(maxComputationTimePathOnReset) && (maxComputationTimePathOnReset > 0.0)))
                return error_code::ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH_RESET;
            if(!(std::isfinite(maxComputationTimeMotionOnReset) && (maxComputationTimeMotionOnReset > 0.0)))
                return error_code::ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION_RESET;
            if(!(std::isfinite(maxComputationTimePath) && (maxComputationTimePath > 0.0)))
                return error_code::ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH;
            if(!(std::isfinite(maxComputationTimeMotion) && (maxComputationTimeMotion > 0.0)))
                return error_code::ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION;
            if(!(std::isfinite(additionalAheadPlanningTime) && (additionalAheadPlanningTime > 0.0)))
                return error_code::ONLINEPLANNER_ADDITIONAL_AHEAD_PLANNING_TIME;
            if(!(std::isfinite(additionalTrajectoryDuration) && (additionalTrajectoryDuration > 0.0)))
                return error_code::ONLINEPLANNER_ADDITIONAL_TRAJECTORY_DURATION;
            if(!(std::isfinite(timeKeepPastTrajectory) && (timeKeepPastTrajectory >= 0.0)))
                return error_code::ONLINEPLANNER_TIME_KEEP_PAST_TRAJECTORY;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("bool", preString + "predictInitialStateOnReset", {1}, &predictInitialStateOnReset, sizeof(predictInitialStateOnReset));
            file.WriteField("double", preString + "maxComputationTimePathOnReset", {1}, &maxComputationTimePathOnReset, sizeof(maxComputationTimePathOnReset));
            file.WriteField("double", preString + "maxComputationTimeMotionOnReset", {1}, &maxComputationTimeMotionOnReset, sizeof(maxComputationTimeMotionOnReset));
            file.WriteField("double", preString + "maxComputationTimePath", {1}, &maxComputationTimePath, sizeof(maxComputationTimePath));
            file.WriteField("double", preString + "maxComputationTimeMotion", {1}, &maxComputationTimeMotion, sizeof(maxComputationTimeMotion));
            file.WriteField("double", preString + "additionalAheadPlanningTime", {1}, &additionalAheadPlanningTime, sizeof(additionalAheadPlanningTime));
            file.WriteField("double", preString + "additionalTrajectoryDuration", {1}, &additionalTrajectoryDuration, sizeof(additionalTrajectoryDuration));
            file.WriteField("double", preString + "timeKeepPastTrajectory", {1}, &timeKeepPastTrajectory, sizeof(timeKeepPastTrajectory));
        }
};


} /* namespace: planner */


} /* namespace: mpsv */

