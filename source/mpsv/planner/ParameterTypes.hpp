#pragma once


#include <mpsv/core/MPSVCommon.hpp>
#include <mpsv/core/DataLogFile.hpp>
#include <mpsv/geometry/VehicleShape.hpp>
#include <mpsv/math/Additional.hpp>


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
         * @return True if parameter is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validModel = std::isfinite(matF[0]) && std::isfinite(matF[1]) && std::isfinite(matF[2]) && std::isfinite(matF[3]) && std::isfinite(matF[4]) && std::isfinite(matF[5]) && std::isfinite(matF[6]) && std::isfinite(matF[7]) && std::isfinite(matF[8]) && std::isfinite(matF[9]) && std::isfinite(matF[10]) && std::isfinite(matF[11]);
            validModel &= std::isfinite(matF[12]) && std::isfinite(matF[13]) && std::isfinite(matF[14]) && std::isfinite(matF[15]) && std::isfinite(matF[16]) && std::isfinite(matF[17]) && std::isfinite(matF[18]) && std::isfinite(matF[19]) && std::isfinite(matF[20]) && std::isfinite(matF[21]) && std::isfinite(matF[22]) && std::isfinite(matF[23]);
            validModel &= std::isfinite(matF[24]) && std::isfinite(matF[25]) && std::isfinite(matF[26]) && std::isfinite(matF[27]) && std::isfinite(matF[28]) && std::isfinite(matF[29]) && std::isfinite(matF[30]) && std::isfinite(matF[31]) && std::isfinite(matF[32]) && std::isfinite(matF[33]) && std::isfinite(matF[34]) && std::isfinite(matF[35]);
            validModel &= std::isfinite(matB[0]) && std::isfinite(matB[1]) && std::isfinite(matB[2]);
            validModel &= std::isfinite(matB[3]) && std::isfinite(matB[4]) && std::isfinite(matB[5]);
            validModel &= std::isfinite(matB[6]) && std::isfinite(matB[7]) && std::isfinite(matB[8]);
            validModel &= std::isfinite(vecTimeconstantsXYN[0]) && std::isfinite(vecTimeconstantsXYN[1]) && std::isfinite(vecTimeconstantsXYN[2]) && (vecTimeconstantsXYN[0] > 0.0) && (vecTimeconstantsXYN[1] > 0.0) && (vecTimeconstantsXYN[2] > 0.0);
            validModel &= std::isfinite(vecTimeconstantsInput[0]) && std::isfinite(vecTimeconstantsInput[1]) && std::isfinite(vecTimeconstantsInput[2]) && (vecTimeconstantsInput[0] > 0.0) && (vecTimeconstantsInput[1] > 0.0) && (vecTimeconstantsInput[2] > 0.0);
            validModel &= std::isfinite(lowerLimitXYN[0]) && std::isfinite(lowerLimitXYN[1]) && std::isfinite(lowerLimitXYN[2]);
            validModel &= std::isfinite(upperLimitXYN[0]) && std::isfinite(upperLimitXYN[1]) && std::isfinite(upperLimitXYN[2]);
            validModel &= (upperLimitXYN[0] >= lowerLimitXYN[0]) && (upperLimitXYN[1] >= lowerLimitXYN[1]) && (upperLimitXYN[2] >= lowerLimitXYN[2]);
            return validModel;
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
         * @return True if parameter is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validCostMap = (modBreakpoints > 0);
            validCostMap &= std::isfinite(resolution) && (resolution > 1e-3);
            validCostMap &= std::isfinite(distanceScale) && (distanceScale >= 0.0);
            validCostMap &= std::isfinite(distanceDecay) && (distanceDecay > 0.0);
            return validCostMap;
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
        double weightPsi;            // Weighting for heading angle (psi) in distance metric function.
        double weightSway;           // Weighting for sway movement (heading angle with respect to perpenticular direction of movement).
        double weightReverseScale;   // Weighting for sway and reverse movement (heading angle with respect to line angle).
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
         * @return True if parameter is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validMetric = std::isfinite(weightPsi) && (weightPsi > 0.0);
            validMetric &= std::isfinite(weightSway) && (weightSway >= 0.0);
            validMetric &= std::isfinite(weightReverseScale) && (weightReverseScale >= 0.0);
            validMetric &= std::isfinite(weightReverseDecay) && (weightReverseDecay > 0.0);
            return validMetric;
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
        std::vector<std::array<double,2>> skeletalPoints;   // Skeletal points (b-frame) at which the cost map is to be evaluated. All points must be inside the vehicle shape.

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
         * @brief Check whether this parameter is valid.
         * @return True if parameter is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            constexpr double minPositionDeviation = 0.01; // Minimum position deviation for path subdivision during collision checking.
            constexpr double minAngleDeviation = 0.0174532925199433; // Minimum angle deviation for path subdivision during collision checking (= 1 deg).
            bool validGeometry = std::isfinite(collisionCheckMaxPositionDeviation) && (collisionCheckMaxPositionDeviation >= minPositionDeviation);
            validGeometry &= std::isfinite(collisionCheckMaxAngleDeviation) && (collisionCheckMaxAngleDeviation >= minAngleDeviation);
            validGeometry &= vehicleShape.InternalPolygonsAreConvex() && vehicleShape.IsFinite();
            validGeometry &= !skeletalPoints.empty();
            for(auto&& p : skeletalPoints){
                validGeometry &= std::isfinite(p[0]) && std::isfinite(p[1]);
                validGeometry &= vehicleShape.IsInside(p);
            }
            return validGeometry;
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
            periodGoalSampling        = 50;
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return True if parameter is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validPathPlanner = (periodGoalSampling > 0);
            return validPathPlanner;
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
 * @brief Parameter for the motion planner.
 */
class ParameterMotionPlanner {
    public:
        double samplingRangePosition;                          // Range in meters for sampling the position around a given path.
        double samplingRangeAngle;                             // Range in radians for sampling the angle around a given path.
        uint32_t periodGoalSampling;                           // Iteration period for goal sampling. Specifies how often the goal value should be used for sampling.
        double sampletime;                                     // Sampletime to be used for fixed-step trajectory simulation.
        double maxPositionOvershoot;                           // Maximum position overshoot due to dynamic motion between two states. This value is used to remove obstacles outside the area of interest.
        double maxInputPathLength;                             // Maximum length (> 0) of the input path (x,y only). The input path is trimmed to ensure this maximum length. The trimmed pose may be interpolated.
        struct {
            double maxRadiusX;                                 // [Controller] Maximum look-ahead distance for longitudinal distance during pose control.
            double maxRadiusY;                                 // [Controller] Maximum look-ahead distance for lateral distance during pose control.
            double maxRadiusPsi;                               // [Controller] Maximum look-ahead distance for angular distance during pose control.
            double minRadiusPosition;                          // [Controller] Minimum look-ahead distance for position during pose control. The radius is limited by the guidance law according to nearby obstacles but is never lower than this value.
            std::array<double,36> matK;                        // [Controller] 3-by-12 control gain matrix (row-major order) for pose control.
        } controller;
        struct {
            std::array<double,3> rangePose;                    // [RegionOfAttraction] Pose box constraints for the region of attraction.
            std::array<double,3> rangeUVR;                     // [RegionOfAttraction] Velocity box constraints for the region of attraction.
            std::array<double,3> rangeXYN;                     // [RegionOfAttraction] The force range {dX,dY,dN}. A given force must be in this range {[-dX,dX],[-dY,dY],[-dN,dN]} to be in the region of attraction.
        } regionOfAttraction;

        /**
         * @brief Construct a new motion planner parameter object and set default values.
         */
        ParameterMotionPlanner() noexcept { Clear(); }

        /**
         * @brief Clear the motion planner parameters and set default values.
         */
        void Clear(void) noexcept {
            samplingRangePosition                   = 10.0;
            samplingRangeAngle                      = 1.0;
            periodGoalSampling                      = 50;
            sampletime                              = 0.05;
            maxPositionOvershoot                    = 10.0;
            maxInputPathLength                      = 50.0;
            controller.matK                         = {22.6634683021076,0.41083997617316,-5.03181108460105,200.152378865113,3.50131969868455,-45.4715931055404,1.5119695156125,4.11059401574811e-05,6.69158720018265e-05,-2.34079198895084,-7.54067967761855e-05,-0.000123856162071262,1.12434267795184,25.3138181465665,14.4246388918514,9.92963030390279,215.90956455243,144.18277442949,-5.88351152250358e-05,1.5148295247012,-0.00151675229642403,0.000107595199040545,-2.34604350301895,0.00279300696770468,1.23673721286666,1.74532076137872,126.252817415723,10.9222424328901,14.9545285554908,1147.21712615239,0.000108596006441483,0.000320946378040582,1.50969338333892,-0.000200797181833612,-0.000590994129882,-2.33658555400436};
            controller.maxRadiusX                   = 10.0;
            controller.maxRadiusY                   = 6.0;
            controller.maxRadiusPsi                 = 1.0;
            controller.minRadiusPosition            = 2.0;
            regionOfAttraction.rangePose            = {0.25, 0.25, 0.15};
            regionOfAttraction.rangeUVR             = {0.1, 0.1, 0.01};
            regionOfAttraction.rangeXYN             = {10.0, 10.0, 10.0};
        }

        /**
         * @brief Check whether this parameter is valid.
         * @return True if parameter is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validMotionPlanner = std::isfinite(samplingRangePosition) && (samplingRangePosition > 0.0);
            validMotionPlanner &= std::isfinite(samplingRangeAngle) && (samplingRangeAngle > 0.0);
            validMotionPlanner &= (periodGoalSampling > 0);
            validMotionPlanner &= std::isfinite(sampletime) && (sampletime > 0.0);
            validMotionPlanner &= std::isfinite(maxPositionOvershoot) && (maxPositionOvershoot > 0.0);
            validMotionPlanner &= std::isfinite(maxInputPathLength) && (maxInputPathLength > 0.0);
            validMotionPlanner &= std::isfinite(controller.matK[0]) && std::isfinite(controller.matK[1]) && std::isfinite(controller.matK[2]);
            validMotionPlanner &= std::isfinite(controller.matK[3]) && std::isfinite(controller.matK[4]) && std::isfinite(controller.matK[5]);
            validMotionPlanner &= std::isfinite(controller.matK[6]) && std::isfinite(controller.matK[7]) && std::isfinite(controller.matK[8]);
            validMotionPlanner &= std::isfinite(controller.matK[9]) && std::isfinite(controller.matK[10]) && std::isfinite(controller.matK[11]);
            validMotionPlanner &= std::isfinite(controller.matK[12]) && std::isfinite(controller.matK[13]) && std::isfinite(controller.matK[14]);
            validMotionPlanner &= std::isfinite(controller.matK[15]) && std::isfinite(controller.matK[16]) && std::isfinite(controller.matK[17]);
            validMotionPlanner &= std::isfinite(controller.matK[18]) && std::isfinite(controller.matK[19]) && std::isfinite(controller.matK[20]);
            validMotionPlanner &= std::isfinite(controller.matK[21]) && std::isfinite(controller.matK[22]) && std::isfinite(controller.matK[23]);
            validMotionPlanner &= std::isfinite(controller.matK[24]) && std::isfinite(controller.matK[25]) && std::isfinite(controller.matK[26]);
            validMotionPlanner &= std::isfinite(controller.matK[27]) && std::isfinite(controller.matK[28]) && std::isfinite(controller.matK[29]);
            validMotionPlanner &= std::isfinite(controller.matK[30]) && std::isfinite(controller.matK[31]) && std::isfinite(controller.matK[32]);
            validMotionPlanner &= std::isfinite(controller.matK[33]) && std::isfinite(controller.matK[34]) && std::isfinite(controller.matK[35]);
            validMotionPlanner &= std::isfinite(controller.maxRadiusX) && (controller.maxRadiusX > 0.0);
            validMotionPlanner &= std::isfinite(controller.maxRadiusY) && (controller.maxRadiusY > 0.0);
            validMotionPlanner &= std::isfinite(controller.maxRadiusPsi) && (controller.maxRadiusPsi > 0.0);
            validMotionPlanner &= std::isfinite(controller.minRadiusPosition) && (controller.minRadiusPosition > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangePose[0]) && (regionOfAttraction.rangePose[0] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangePose[1]) && (regionOfAttraction.rangePose[1] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangePose[2]) && (regionOfAttraction.rangePose[2] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangeUVR[0]) && (regionOfAttraction.rangeUVR[0] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangeUVR[1]) && (regionOfAttraction.rangeUVR[1] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangeUVR[2]) && (regionOfAttraction.rangeUVR[2] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangeXYN[0]) && (regionOfAttraction.rangeXYN[0] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangeXYN[1]) && (regionOfAttraction.rangeXYN[1] > 0.0);
            validMotionPlanner &= std::isfinite(regionOfAttraction.rangeXYN[2]) && (regionOfAttraction.rangeXYN[2] > 0.0);
            return validMotionPlanner;
        }

        /**
         * @brief Write the whole parameter set to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            std::array<double,36> tmp;
            file.WriteField("double", preString + "samplingRangePosition", {1}, &samplingRangePosition, sizeof(samplingRangePosition));
            file.WriteField("double", preString + "samplingRangeAngle", {1}, &samplingRangeAngle, sizeof(samplingRangeAngle));
            file.WriteField("uint32_t", preString + "periodGoalSampling", {1}, &periodGoalSampling, sizeof(periodGoalSampling));
            file.WriteField("double", preString + "sampletime", {1}, &sampletime, sizeof(sampletime));
            file.WriteField("double", preString + "maxPositionOvershoot", {1}, &maxPositionOvershoot, sizeof(maxPositionOvershoot));
            file.WriteField("double", preString + "maxInputPathLength", {1}, &maxInputPathLength, sizeof(maxInputPathLength));
            tmp[0]  = controller.matK[0];  tmp[1]  = controller.matK[12]; tmp[2]  = controller.matK[24];
            tmp[3]  = controller.matK[1];  tmp[4]  = controller.matK[13]; tmp[5]  = controller.matK[25];
            tmp[6]  = controller.matK[2];  tmp[7]  = controller.matK[14]; tmp[8]  = controller.matK[26];
            tmp[9]  = controller.matK[3];  tmp[10] = controller.matK[15]; tmp[11] = controller.matK[27];
            tmp[12] = controller.matK[4];  tmp[13] = controller.matK[16]; tmp[14] = controller.matK[28];
            tmp[15] = controller.matK[5];  tmp[16] = controller.matK[17]; tmp[17] = controller.matK[29];
            tmp[18] = controller.matK[6];  tmp[19] = controller.matK[18]; tmp[20] = controller.matK[30];
            tmp[21] = controller.matK[7];  tmp[22] = controller.matK[19]; tmp[23] = controller.matK[31];
            tmp[24] = controller.matK[8];  tmp[25] = controller.matK[20]; tmp[26] = controller.matK[32];
            tmp[27] = controller.matK[9];  tmp[28] = controller.matK[21]; tmp[29] = controller.matK[33];
            tmp[30] = controller.matK[10]; tmp[31] = controller.matK[22]; tmp[32] = controller.matK[34];
            tmp[33] = controller.matK[11]; tmp[34] = controller.matK[23]; tmp[35] = controller.matK[35];
            file.WriteField("double", preString + "controller.matK", {3,12}, &tmp[0], sizeof(controller.matK));
            file.WriteField("double", preString + "controller.maxRadiusX", {1}, &controller.maxRadiusX, sizeof(controller.maxRadiusX));
            file.WriteField("double", preString + "controller.maxRadiusY", {1}, &controller.maxRadiusY, sizeof(controller.maxRadiusY));
            file.WriteField("double", preString + "controller.maxRadiusPsi", {1}, &controller.maxRadiusPsi, sizeof(controller.maxRadiusPsi));
            file.WriteField("double", preString + "controller.minRadiusPosition", {1}, &controller.minRadiusPosition, sizeof(controller.minRadiusPosition));
            file.WriteField("double", preString + "regionOfAttraction.rangePose", {3,1}, &regionOfAttraction.rangePose[0], sizeof(regionOfAttraction.rangePose));
            file.WriteField("double", preString + "regionOfAttraction.rangeUVR", {3,1}, &regionOfAttraction.rangeUVR[0], sizeof(regionOfAttraction.rangeUVR));
            file.WriteField("double", preString + "regionOfAttraction.rangeXYN", {3,1}, &regionOfAttraction.rangeXYN[0], sizeof(regionOfAttraction.rangeXYN));
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
         * @return True if parameter is valid, false otherwise.
         */
        bool IsValid(void) const noexcept {
            bool validOnlinePlanner = std::isfinite(maxComputationTimePathOnReset) && (maxComputationTimePathOnReset > 0.0);
            validOnlinePlanner &= std::isfinite(maxComputationTimeMotionOnReset) && (maxComputationTimeMotionOnReset > 0.0);
            validOnlinePlanner &= std::isfinite(maxComputationTimePath) && (maxComputationTimePath > 0.0);
            validOnlinePlanner &= std::isfinite(maxComputationTimeMotion) && (maxComputationTimeMotion > 0.0);
            validOnlinePlanner &= std::isfinite(additionalAheadPlanningTime) && (additionalAheadPlanningTime > 0.0);
            validOnlinePlanner &= std::isfinite(additionalTrajectoryDuration) && (additionalTrajectoryDuration > 0.0);
            validOnlinePlanner &= std::isfinite(timeKeepPastTrajectory) && (timeKeepPastTrajectory >= 0.0);
            return validOnlinePlanner;
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

