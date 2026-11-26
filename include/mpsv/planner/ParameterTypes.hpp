#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/logging/File.hpp>
#include <mpsv/geometry/VehicleShape.hpp>
#include <mpsv/math/Additional.hpp>
#include <mpsv/common/ErrorCode.hpp>


namespace mpsv::planner {


/**
 * @brief Parameter for the nonlinear dynamical motion model of the vehicle.
 */
class ParameterModel {
    public:
        std::array<double, 36U> matF;                   // 3-by-12 coefficient matrix (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double, 9U> matB;                    // 3-by-3 input matrix B (row-major order) of model nu_dot = F*n(nu) + B*tau.
        std::array<double, 3U> vecTimeconstantsXYN;     // Timeconstants {TX, TY, TN} for input force dynamics.
        std::array<double, 3U> vecTimeconstantsInput;   // Timeconstants {Tf1, Tf2, Tf3} for input filter dynamics.
        std::array<double, 3U> lowerLimitXYN;           // Lower saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.
        std::array<double, 3U> upperLimitXYN;           // Upper saturation value for input vector u of model nu_dot = F*n(nu) + B*tau.

        /**
         * @brief Construct a new model parameter object and set default values.
         */
        ParameterModel() noexcept {
            Clear();
        }

        /**
         * @brief Clear the model parameter and set default values.
         */
        void Clear() noexcept {
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
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(std::isfinite(matF[0U]) &&
                  std::isfinite(matF[1U]) &&
                  std::isfinite(matF[2U]) &&
                  std::isfinite(matF[3U]) &&
                  std::isfinite(matF[4U]) &&
                  std::isfinite(matF[5U]) &&
                  std::isfinite(matF[6U]) &&
                  std::isfinite(matF[7U]) &&
                  std::isfinite(matF[8U]) &&
                  std::isfinite(matF[9U]) &&
                  std::isfinite(matF[10U]) &&
                  std::isfinite(matF[11U]) &&
                  std::isfinite(matF[12U]) &&
                  std::isfinite(matF[13U]) &&
                  std::isfinite(matF[14U]) &&
                  std::isfinite(matF[15U]) &&
                  std::isfinite(matF[16U]) &&
                  std::isfinite(matF[17U]) &&
                  std::isfinite(matF[18U]) &&
                  std::isfinite(matF[19U]) &&
                  std::isfinite(matF[20U]) &&
                  std::isfinite(matF[21U]) &&
                  std::isfinite(matF[22U]) &&
                  std::isfinite(matF[23U]) &&
                  std::isfinite(matF[24U]) &&
                  std::isfinite(matF[25U]) &&
                  std::isfinite(matF[26U]) &&
                  std::isfinite(matF[27U]) &&
                  std::isfinite(matF[28U]) &&
                  std::isfinite(matF[29U]) &&
                  std::isfinite(matF[30U]) &&
                  std::isfinite(matF[31U]) &&
                  std::isfinite(matF[32U]) &&
                  std::isfinite(matF[33U]) &&
                  std::isfinite(matF[34U]) &&
                  std::isfinite(matF[35U])))
                return error_code::PARAM_MODEL_MATF;
            if (!(std::isfinite(matB[0U]) &&
                  std::isfinite(matB[1U]) &&
                  std::isfinite(matB[2U]) &&
                  std::isfinite(matB[3U]) &&
                  std::isfinite(matB[4U]) &&
                  std::isfinite(matB[5U]) &&
                  std::isfinite(matB[6U]) &&
                  std::isfinite(matB[7U]) &&
                  std::isfinite(matB[8U])))
                return error_code::PARAM_MODEL_MATB;
            if (!(std::isfinite(vecTimeconstantsXYN[0U]) &&
                  std::isfinite(vecTimeconstantsXYN[1U]) &&
                  std::isfinite(vecTimeconstantsXYN[2U]) &&
                  (vecTimeconstantsXYN[0U] > 0.0) &&
                  (vecTimeconstantsXYN[1U] > 0.0) &&
                  (vecTimeconstantsXYN[2U] > 0.0)))
                return error_code::PARAM_MODEL_VEC_TIMECONSTANTS_XYN;
            if (!(std::isfinite(vecTimeconstantsInput[0U]) &&
                  std::isfinite(vecTimeconstantsInput[1U]) &&
                  std::isfinite(vecTimeconstantsInput[2U]) &&
                  (vecTimeconstantsInput[0U] > 0.0) &&
                  (vecTimeconstantsInput[1U] > 0.0) &&
                  (vecTimeconstantsInput[2U] > 0.0)))
                return error_code::PARAM_MODEL_VEC_TIMECONSTANTS_INPUT;
            if (!(std::isfinite(lowerLimitXYN[0U]) &&
                  std::isfinite(lowerLimitXYN[1U]) &&
                  std::isfinite(lowerLimitXYN[2U])))
                return error_code::PARAM_MODEL_LOWER_LIMIT_XYN;
            if (!(std::isfinite(upperLimitXYN[0U]) &&
                  std::isfinite(upperLimitXYN[1U]) &&
                  std::isfinite(upperLimitXYN[2U])))
                return error_code::PARAM_MODEL_UPPER_LIMIT_XYN;
            if (!((upperLimitXYN[0U] >= lowerLimitXYN[0U]) &&
                  (upperLimitXYN[1U] >= lowerLimitXYN[1U]) &&
                  (upperLimitXYN[2U] >= lowerLimitXYN[2U])))
                return error_code::PARAM_MODEL_LOWER_GREATER_UPPER_LIMIT;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         * @note Matrix data is written in column-major order to the log file, as used by MATLAB.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            std::array<double, 36U> tmp;
            tmp[0U]  = matF[0U];  tmp[1U]  = matF[12U]; tmp[2U]  = matF[24U];
            tmp[3U]  = matF[1U];  tmp[4U]  = matF[13U]; tmp[5U]  = matF[25U];
            tmp[6U]  = matF[2U];  tmp[7U]  = matF[14U]; tmp[8U]  = matF[26U];
            tmp[9U]  = matF[3U];  tmp[10U] = matF[15U]; tmp[11U] = matF[27U];
            tmp[12U] = matF[4U];  tmp[13U] = matF[16U]; tmp[14U] = matF[28U];
            tmp[15U] = matF[5U];  tmp[16U] = matF[17U]; tmp[17U] = matF[29U];
            tmp[18U] = matF[6U];  tmp[19U] = matF[18U]; tmp[20U] = matF[30U];
            tmp[21U] = matF[7U];  tmp[22U] = matF[19U]; tmp[23U] = matF[31U];
            tmp[24U] = matF[8U];  tmp[25U] = matF[20U]; tmp[26U] = matF[32U];
            tmp[27U] = matF[9U];  tmp[28U] = matF[21U]; tmp[29U] = matF[33U];
            tmp[30U] = matF[10U]; tmp[31U] = matF[22U]; tmp[32U] = matF[34U];
            tmp[33U] = matF[11U]; tmp[34U] = matF[23U]; tmp[35U] = matF[35U];
            file.WriteField("double", preString + "matF", {3U, 12U}, &tmp[0U], sizeof(matF));
            file.WriteField("double", preString + "vecTimeconstantsXYN", {3U, 1U}, &vecTimeconstantsXYN[0U], sizeof(vecTimeconstantsXYN));
            file.WriteField("double", preString + "vecTimeconstantsInput", {3U, 1U}, &vecTimeconstantsInput[0U], sizeof(vecTimeconstantsInput));
            tmp[0U] = matB[0U]; tmp[1U] = matB[3U]; tmp[2U] = matB[6U];
            tmp[3U] = matB[1U]; tmp[4U] = matB[4U]; tmp[5U] = matB[7U];
            tmp[6U] = matB[2U]; tmp[7U] = matB[5U]; tmp[8U] = matB[8U];
            file.WriteField("double", preString + "matB", {3U, 3U}, &tmp[0U], sizeof(matB));
            file.WriteField("double", preString + "lowerLimitXYN", {3U, 1U}, &lowerLimitXYN[0U], sizeof(lowerLimitXYN));
            file.WriteField("double", preString + "upperLimitXYN", {3U, 1U}, &upperLimitXYN[0U], sizeof(upperLimitXYN));
        }
};


/**
 * @brief Parameter for the cost map (2D look-up table with additional cost values).
 */
class ParameterCostMap {
    public:
        double resolution{0.1};       // Resolution (> 1.0e-3) of the grid map (dimension of one cell).
        double distanceScale{5.0};    // Scale factor (>= 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).
        double distanceDecay{0.02};   // Decay factor (> 0) for obstacle distance (d) cost in cost map: scale*exp(-decay*d^2).

        /**
         * @brief Clear the costmap parameter and set default values.
         */
        void Clear() noexcept {
            resolution     = 0.1;
            distanceScale  = 5.0;
            distanceDecay  = 0.02;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(std::isfinite(resolution) && (resolution > 1.0e-3))) {
                return error_code::PARAM_COSTMAP_RESOLUTION;
            }
            if (!(std::isfinite(distanceScale) && (distanceScale >= 0.0))) {
                return error_code::PARAM_COSTMAP_DISTANCE_SCALE;
            }
            if (!(std::isfinite(distanceDecay) && (distanceDecay > 0.0))) {
                return error_code::PARAM_COSTMAP_DISTANCE_DECAY;
            }
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("double", preString + "resolution", {1U}, &resolution, sizeof(resolution));
            file.WriteField("double", preString + "distanceScale", {1U}, &distanceScale, sizeof(distanceScale));
            file.WriteField("double", preString + "distanceDecay", {1U}, &distanceDecay, sizeof(distanceDecay));
        }
};


/**
 * @brief Additional parameter for the distance metric function.
 */
class ParameterMetric {
    public:
        double weightPsi{3.0};            // Weighting (> 0) for heading angle (psi) in distance metric function.
        double weightSway{2.0};           // Weighting (>= 0) for sway movement (heading angle with respect to perpendicular direction of movement).
        double weightReverseScale{0.0};   // Weighting (>= 0) for sway and reverse movement (heading angle with respect to line angle).
        double weightReverseDecay{1.0};   // Decay factor (> 0) for the weighting function that weights sway and reverse movement.

        /**
         * @brief Clear the metric parameter and set default values.
         */
        void Clear() noexcept {
            weightPsi          = 3.0;
            weightSway         = 2.0;
            weightReverseScale = 0.0;
            weightReverseDecay = 1.0;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(std::isfinite(weightPsi) && (weightPsi > 0.0))) {
                return error_code::PARAM_METRIC_WEIGHT_PSI;
            }
            if (!(std::isfinite(weightSway) && (weightSway >= 0.0))) {
                return error_code::PARAM_METRIC_WEIGHT_SWAY;
            }
            if (!(std::isfinite(weightReverseScale) && (weightReverseScale >= 0.0))) {
                return error_code::PARAM_METRIC_WEIGHT_REVERSE_SCALE;
            }
            if (!(std::isfinite(weightReverseDecay) && (weightReverseDecay > 0.0))) {
                return error_code::PARAM_METRIC_WEIGHT_REVERSE_DECAY;
            }
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("double", preString + "weightPsi", {1U}, &weightPsi, sizeof(weightPsi));
            file.WriteField("double", preString + "weightSway", {1U}, &weightSway, sizeof(weightSway));
            file.WriteField("double", preString + "weightReverseScale", {1U}, &weightReverseScale, sizeof(weightReverseScale));
            file.WriteField("double", preString + "weightReverseDecay", {1U}, &weightReverseDecay, sizeof(weightReverseDecay));
        }
};


/**
 * @brief Geometry parameter.
 */
class ParameterGeometry {
    public:
        double collisionCheckMaxPositionDeviation{0.1};                     // Maximum position deviation for path subdivision during collision checking. Must be at least 0.01 meters.
        double collisionCheckMaxAngleDeviation{mpsv::math::deg2rad(5.0)};   // Maximum angle deviation for path subdivision during collision checking. Must be at least 1 degree (0.0174532925199433 radians).
        mpsv::geometry::VehicleShape vehicleShape;                          // Static shape of the vehicle to be used for path planning described by several convex polyons.
        std::vector<std::array<double, 2U>> skeletalPoints;                 // Skeletal points (b-frame) at which the cost map is to be evaluated.

        /**
         * @brief Clear the geometry parameter and set default values.
         */
        void Clear() noexcept {
            constexpr double defaultAngle{mpsv::math::deg2rad(5.0)};
            collisionCheckMaxPositionDeviation = 0.1;
            collisionCheckMaxAngleDeviation    = defaultAngle;
            vehicleShape.Clear();
            skeletalPoints.clear();
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code IsValid() const noexcept {
            constexpr double minPositionDeviation{0.01}; // Minimum position deviation for path subdivision during collision checking.
            constexpr double minAngleDeviation{mpsv::math::deg2rad(1.0)}; // Minimum angle deviation for path subdivision during collision checking.
            if (!(std::isfinite(collisionCheckMaxPositionDeviation) && (collisionCheckMaxPositionDeviation >= minPositionDeviation))) {
                return error_code::PARAM_GEOMETRY_MAX_POSITION_DEVIATION;
            }
            if (!(std::isfinite(collisionCheckMaxAngleDeviation) && (collisionCheckMaxAngleDeviation >= minAngleDeviation))) {
                return error_code::PARAM_GEOMETRY_MAX_ANGLE_DEVIATION;
            }
            if (!vehicleShape.IsValid()) {
                return error_code::PARAM_GEOMETRY_VEHICLE_SHAPE;
            }
            if (skeletalPoints.empty()) {
                return error_code::PARAM_GEOMETRY_SKELETAL_POINTS;
            }
            for (auto&& p : skeletalPoints) {
                if (!(std::isfinite(p[0U]) && std::isfinite(p[1U]))) {
                    return error_code::PARAM_GEOMETRY_SKELETAL_POINTS;
                }
            }
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            std::vector<mpsv::geometry::ConvexVehicleShape> vehicleShapes = vehicleShape.GetConvexShapes();
            file.WriteField("double", preString + "collisionCheckMaxPositionDeviation", {1U}, &collisionCheckMaxPositionDeviation, sizeof(collisionCheckMaxPositionDeviation));
            file.WriteField("double", preString + "collisionCheckMaxAngleDeviation", {1U}, &collisionCheckMaxAngleDeviation, sizeof(collisionCheckMaxAngleDeviation));
            file.WriteConvexPolygonsField(preString + "vehicleShape", vehicleShapes);
            file.WriteVectorField(preString + "skeletalPoints", skeletalPoints);
        }
};


/**
 * @brief Parameter for the path planner.
 */
class ParameterPathPlanner {
    public:
        std::uint32_t periodGoalSampling{50U};   // Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.

        /**
         * @brief Clear the path planner parameters and set default values.
         */
        void Clear() noexcept {
            periodGoalSampling = 50U;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(periodGoalSampling > 0U)) {
                return error_code::PARAM_PATHPLANNER_PERIOD_GOAL_SAMPLING;
            }
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("uint32_t", preString + "periodGoalSampling", {1U}, &periodGoalSampling, sizeof(periodGoalSampling));
        }
};


/**
 * @brief Parameter for the controller inside the motion planner.
 */
class ParameterController {
    public:
        double maxRadiusX;              // Maximum look-ahead distance for longitudinal distance during pose control.
        double maxRadiusY;              // Maximum look-ahead distance for lateral distance during pose control.
        double maxRadiusPsi;            // Maximum look-ahead distance for angular distance during pose control.
        double minRadiusPosition;       // Minimum look-ahead distance for position during pose control. The radius is limited by the guidance law according to nearby obstacles but is never lower than this value.
        std::array<double, 36U> matK;   // 3-by-12 control gain matrix (row-major order) for pose control.

        /**
         * @brief Construct a new controller parameter object and set default values.
         */
        ParameterController() noexcept {
            Clear();
        }

        /**
         * @brief Clear the controller parameters and set default values.
         */
        void Clear() noexcept {
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
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(std::isfinite(matK[0U]) &&
                  std::isfinite(matK[1U]) &&
                  std::isfinite(matK[2U]) &&
                  std::isfinite(matK[3U]) &&
                  std::isfinite(matK[4U]) &&
                  std::isfinite(matK[5U]) &&
                  std::isfinite(matK[6U]) &&
                  std::isfinite(matK[7U]) &&
                  std::isfinite(matK[8U]) &&
                  std::isfinite(matK[9U]) &&
                  std::isfinite(matK[10U]) &&
                  std::isfinite(matK[11U]) &&
                  std::isfinite(matK[12U]) &&
                  std::isfinite(matK[13U]) &&
                  std::isfinite(matK[14U]) &&
                  std::isfinite(matK[15U]) &&
                  std::isfinite(matK[16U]) &&
                  std::isfinite(matK[17U]) &&
                  std::isfinite(matK[18U]) &&
                  std::isfinite(matK[19U]) &&
                  std::isfinite(matK[20U]) &&
                  std::isfinite(matK[21U]) &&
                  std::isfinite(matK[22U]) &&
                  std::isfinite(matK[23U]) &&
                  std::isfinite(matK[24U]) &&
                  std::isfinite(matK[25U]) &&
                  std::isfinite(matK[26U]) &&
                  std::isfinite(matK[27U]) &&
                  std::isfinite(matK[28U]) &&
                  std::isfinite(matK[29U]) &&
                  std::isfinite(matK[30U]) &&
                  std::isfinite(matK[31U]) &&
                  std::isfinite(matK[32U]) &&
                  std::isfinite(matK[33U]) &&
                  std::isfinite(matK[34U]) &&
                  std::isfinite(matK[35U])))
                return error_code::PARAM_CONTROLLER_MAT_K;
            if (!(std::isfinite(maxRadiusX) && (maxRadiusX > 0.0))) {
                return error_code::PARAM_CONTROLLER_MAX_RADIUS_X;
            }
            if (!(std::isfinite(maxRadiusY) && (maxRadiusY > 0.0))) {
                return error_code::PARAM_CONTROLLER_MAX_RADIUS_Y;
            }
            if (!(std::isfinite(maxRadiusPsi) && (maxRadiusPsi > 0.0))) {
                return error_code::PARAM_CONTROLLER_MAX_RADIUS_PSI;
            }
            if (!(std::isfinite(minRadiusPosition) && (minRadiusPosition > 0.0))) {
                return error_code::PARAM_CONTROLLER_MIN_RADIUS_POSITION;
            }
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            std::array<double, 36U> tmp;
            tmp[0U]  = matK[0U];  tmp[1U]  = matK[12U]; tmp[2U]  = matK[24U];
            tmp[3U]  = matK[1U];  tmp[4U]  = matK[13U]; tmp[5U]  = matK[25U];
            tmp[6U]  = matK[2U];  tmp[7U]  = matK[14U]; tmp[8U]  = matK[26U];
            tmp[9U]  = matK[3U];  tmp[10U] = matK[15U]; tmp[11U] = matK[27U];
            tmp[12U] = matK[4U];  tmp[13U] = matK[16U]; tmp[14U] = matK[28U];
            tmp[15U] = matK[5U];  tmp[16U] = matK[17U]; tmp[17U] = matK[29U];
            tmp[18U] = matK[6U];  tmp[19U] = matK[18U]; tmp[20U] = matK[30U];
            tmp[21U] = matK[7U];  tmp[22U] = matK[19U]; tmp[23U] = matK[31U];
            tmp[24U] = matK[8U];  tmp[25U] = matK[20U]; tmp[26U] = matK[32U];
            tmp[27U] = matK[9U];  tmp[28U] = matK[21U]; tmp[29U] = matK[33U];
            tmp[30U] = matK[10U]; tmp[31U] = matK[22U]; tmp[32U] = matK[34U];
            tmp[33U] = matK[11U]; tmp[34U] = matK[23U]; tmp[35U] = matK[35U];
            file.WriteField("double", preString + "matK", {3U, 12U}, &tmp[0U], sizeof(matK));
            file.WriteField("double", preString + "maxRadiusX", {1U}, &maxRadiusX, sizeof(maxRadiusX));
            file.WriteField("double", preString + "maxRadiusY", {1U}, &maxRadiusY, sizeof(maxRadiusY));
            file.WriteField("double", preString + "maxRadiusPsi", {1U}, &maxRadiusPsi, sizeof(maxRadiusPsi));
            file.WriteField("double", preString + "minRadiusPosition", {1U}, &minRadiusPosition, sizeof(minRadiusPosition));
        }
};


/**
 * @brief Parameter for the region of attraction inside the motion planner.
 */
class ParameterRegionOfAttraction {
    public:
        std::array<double, 3U> rangePose{0.25, 0.25, 0.15};   // Pose box constraints for the region of attraction.
        std::array<double, 3U> rangeUVR{0.1, 0.1, 0.01};      // Velocity box constraints for the region of attraction.
        std::array<double, 3U> rangeXYN{10.0, 10.0, 10.0};    // The force range {dX,dY,dN}. A given force must be in this range {[-dX,dX],[-dY,dY],[-dN,dN]} to be in the region of attraction.

        /**
         * @brief Clear the region of attraction parameters and set default values.
         */
        void Clear() noexcept {
            rangePose = {0.25, 0.25, 0.15};
            rangeUVR  = {0.1, 0.1, 0.01};
            rangeXYN  = {10.0, 10.0, 10.0};
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(std::isfinite(rangePose[0U]) &&
                  std::isfinite(rangePose[1U]) &&
                  std::isfinite(rangePose[2U]) &&
                  (rangePose[0U] > 0.0) &&
                  (rangePose[1U] > 0.0) &&
                  (rangePose[2U] > 0.0)))
                return error_code::PARAM_REGION_OF_ATTRACTION_RANGE_POSE;
            if (!(std::isfinite(rangeUVR[0U]) &&
                  std::isfinite(rangeUVR[1U]) &&
                  std::isfinite(rangeUVR[2U]) &&
                  (rangeUVR[0U] > 0.0) &&
                  (rangeUVR[1U] > 0.0) &&
                  (rangeUVR[2U] > 0.0)))
                return error_code::PARAM_REGION_OF_ATTRACTION_RANGE_UVR;
            if (!(std::isfinite(rangeXYN[0U]) &&
                  std::isfinite(rangeXYN[1U]) &&
                  std::isfinite(rangeXYN[2U]) &&
                  (rangeXYN[0U] > 0.0) &&
                  (rangeXYN[1U] > 0.0) &&
                  (rangeXYN[2U] > 0.0)))
                return error_code::PARAM_REGION_OF_ATTRACTION_RANGE_XYN;
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("double", preString + "rangePose", {3U, 1U}, &rangePose[0U], sizeof(rangePose));
            file.WriteField("double", preString + "rangeUVR", {3U, 1U}, &rangeUVR[0U], sizeof(rangeUVR));
            file.WriteField("double", preString + "rangeXYN", {3U, 1U}, &rangeXYN[0U], sizeof(rangeXYN));
        }
};


/**
 * @brief Parameter for the motion planner.
 */
class ParameterMotionPlanner {
    public:
        double samplingRangePosition{10.0};               // Range in meters for sampling the position around a given path.
        double samplingRangeAngle{1.0};                   // Range in radians for sampling the angle around a given path.
        std::uint32_t periodGoalSampling{50U};            // Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
        double sampletime{0.05};                          // Sampletime to be used for fixed-step trajectory simulation.
        double maxInputPathLength{50.0};                  // Maximum length (> 0) of the input path (x,y only). The input path is trimmed to ensure this maximum length. The trimmed pose may be interpolated.
        ParameterController controller;                   // Controller parameters for vehicle simulation.
        ParameterRegionOfAttraction regionOfAttraction;   // Region of attraction parameters for vehicle simulation.

        /**
         * @brief Clear the motion planner parameters and set default values.
         */
        void Clear() noexcept {
            samplingRangePosition = 10.0;
            samplingRangeAngle    = 1.0;
            periodGoalSampling    = 50U;
            sampletime            = 0.05;
            maxInputPathLength    = 50.0;
            controller.Clear();
            regionOfAttraction.Clear();
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return mpsv::error_code::NONE if all attributes are valid, a non-zero error code otherwise.
         */
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(std::isfinite(samplingRangePosition) && (samplingRangePosition > 0.0))) {
                return error_code::PARAM_MOTIONPLANNER_SAMPLING_RANGE_POSITION;
            }
            if (!(std::isfinite(samplingRangeAngle) && (samplingRangeAngle > 0.0))) {
                return error_code::PARAM_MOTIONPLANNER_SAMPLING_RANGE_ANGLE;
            }
            if (!((periodGoalSampling > 0U))) {
                return error_code::PARAM_MOTIONPLANNER_PERIOD_GOAL_SAMPLING;
            }
            if (!(std::isfinite(sampletime) && (sampletime > 0.0))) {
                return error_code::PARAM_MOTIONPLANNER_SAMPLETIME;
            }
            if (!(std::isfinite(maxInputPathLength) && (maxInputPathLength > 0.0))) {
                return error_code::PARAM_MOTIONPLANNER_MAX_INPUT_PATH_LENGTH;
            }
            error_code e = controller.IsValid();
            if (error_code::NONE != e) {
                return e;
            }
            return regionOfAttraction.IsValid();
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("double", preString + "samplingRangePosition", {1U}, &samplingRangePosition, sizeof(samplingRangePosition));
            file.WriteField("double", preString + "samplingRangeAngle", {1U}, &samplingRangeAngle, sizeof(samplingRangeAngle));
            file.WriteField("uint32_t", preString + "periodGoalSampling", {1U}, &periodGoalSampling, sizeof(periodGoalSampling));
            file.WriteField("double", preString + "sampletime", {1U}, &sampletime, sizeof(sampletime));
            file.WriteField("double", preString + "maxInputPathLength", {1U}, &maxInputPathLength, sizeof(maxInputPathLength));
            controller.WriteToFile(file, preString + "controller.");
            regionOfAttraction.WriteToFile(file, preString + "regionOfAttraction.");
        }
};


/**
 * @brief Parameter for the online planner.
 */
class ParameterOnlinePlanner {
    public:
        double maxComputationTimePathOnReset{0.1};     // Maximum computation time in seconds for path planning after a reset.
        double maxComputationTimeMotionOnReset{0.2};   // Maximum computation time in seconds for motion planning after a reset.
        double maxComputationTimePath{0.3};            // Maximum computation time in seconds for path planning.
        double maxComputationTimeMotion{0.6};          // Maximum computation time in seconds for motion planning.
        double additionalAheadPlanningTime{0.2};       // Additional time added to the estimated computation time in seconds to obtain the future timepoint from where to start the next planning problem. This value must be greater than zero.
        double additionalTrajectoryDuration{1.0};      // Additional time added to the ahead planning time (= estimated computation time (no reset) + additional ahead planning time) to obtain the minimum time duration of a trajectory to make sure that the next solve operation can access a future point of the trajectory. This value must be greater than zero.
        double timeKeepPastTrajectory{1.0};            // Time in seconds to keep from a past trajectory. The past data of the previous trajectory is inserted at the beginning of a new solution. Inserting past data helps to handle imperfect time synchronization between this trajectory generator and the user of the trajectory data.

        /**
         * @brief Clear the online planner parameters and set default values.
         */
        void Clear() noexcept {
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
        [[nodiscard]] error_code IsValid() const noexcept {
            if (!(std::isfinite(maxComputationTimePathOnReset) && (maxComputationTimePathOnReset > 0.0))) {
                return error_code::PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH_RESET;
            }
            if (!(std::isfinite(maxComputationTimeMotionOnReset) && (maxComputationTimeMotionOnReset > 0.0))) {
                return error_code::PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION_RESET;
            }
            if (!(std::isfinite(maxComputationTimePath) && (maxComputationTimePath > 0.0))) {
                return error_code::PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_PATH;
            }
            if (!(std::isfinite(maxComputationTimeMotion) && (maxComputationTimeMotion > 0.0))) {
                return error_code::PARAM_ONLINEPLANNER_MAX_COMPUTATION_TIME_MOTION;
            }
            if (!(std::isfinite(additionalAheadPlanningTime) && (additionalAheadPlanningTime > 0.0))) {
                return error_code::PARAM_ONLINEPLANNER_ADDITIONAL_AHEAD_PLANNING_TIME;
            }
            if (!(std::isfinite(additionalTrajectoryDuration) && (additionalTrajectoryDuration > 0.0))) {
                return error_code::PARAM_ONLINEPLANNER_ADDITIONAL_TRAJECTORY_DURATION;
            }
            if (!(std::isfinite(timeKeepPastTrajectory) && (timeKeepPastTrajectory >= 0.0))) {
                return error_code::PARAM_ONLINEPLANNER_TIME_KEEP_PAST_TRAJECTORY;
            }
            return error_code::NONE;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept {
            file.WriteField("double", preString + "maxComputationTimePathOnReset", {1U}, &maxComputationTimePathOnReset, sizeof(maxComputationTimePathOnReset));
            file.WriteField("double", preString + "maxComputationTimeMotionOnReset", {1U}, &maxComputationTimeMotionOnReset, sizeof(maxComputationTimeMotionOnReset));
            file.WriteField("double", preString + "maxComputationTimePath", {1U}, &maxComputationTimePath, sizeof(maxComputationTimePath));
            file.WriteField("double", preString + "maxComputationTimeMotion", {1U}, &maxComputationTimeMotion, sizeof(maxComputationTimeMotion));
            file.WriteField("double", preString + "additionalAheadPlanningTime", {1U}, &additionalAheadPlanningTime, sizeof(additionalAheadPlanningTime));
            file.WriteField("double", preString + "additionalTrajectoryDuration", {1U}, &additionalTrajectoryDuration, sizeof(additionalTrajectoryDuration));
            file.WriteField("double", preString + "timeKeepPastTrajectory", {1U}, &timeKeepPastTrajectory, sizeof(timeKeepPastTrajectory));
        }
};


} /* namespace: mpsv::planner */
