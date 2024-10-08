#pragma once


#include <mpsv_core_MPSVCommon.hpp>
#include <mpsv_core_LookUpTable2D.hpp>
#include <mpsv_core_DataLogFile.hpp>


namespace mpsv {


namespace core {


/**
 * @brief This class represents a 2D look-up table of precalculated scalar values.
 * @tparam Targ A user-defined argument that is passed to the callback function for calculating look-up table data.
 */
template <class Targ> class LookUpTable2DScalar: public mpsv::core::LookUpTable2D<Targ, double> {
    public:
        /**
         * @brief A more exact solution of the line integral than @ref LineIntegral. However, the computational effort is greater. This function does not assume
         * that the cell has a constant value. The four corners of the cell are used to generate a linear interpolated surface. The line integral is then
         * evaluated over that linear interpolated surface.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @tparam U Template parameter must be of type std::array<double,N> with N >= 2.
         * @param p0 Start point of the line from where to start the line integral, given in earth-fixed coordinates (meters).
         * @param p1 End point of the line where to end the line integral, given in earth-fixed coordinates (meters).
         * @return Final value of the line integral along a linear line over the look-up table data.
         */
        template <class T, class U> double LineIntegralLinearInterpolated(const T& p0, const U& p1) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {p0} must be of type std::array<double,N> with N >= 2!");
            static_assert(mpsv::is_vec2<U>::value,"Argument {p1} must be of type std::array<double,N> with N >= 2!");

            // Convert to index position ({0,0} indicates origin of lower-left corner of LUT data cell (lut[0]), {0.5,0.5} would be the center of the lower-left cell, all data corresponds to the center of a cell)
            std::array<double,2> iPos0 = this->ConvertToLUTPosition(p0) / this->res;
            std::array<double,2> iPos1 = this->ConvertToLUTPosition(p1) / this->res;

            // The center of the cell indicates the data, transform index position to the center such that all corners of the new grid cell correspond to true data points
            iPos0[0] -= 0.5;
            iPos0[1] -= 0.5;
            iPos1[0] -= 0.5;
            iPos1[1] -= 0.5;

            // Flooring the fractional index position (iPos) gives the true index
            int32_t ix0 = static_cast<int32_t>(std::floor(iPos0[0]));
            int32_t iy0 = static_cast<int32_t>(std::floor(iPos0[1]));
            int32_t ix1 = static_cast<int32_t>(std::floor(iPos1[0]));
            int32_t iy1 = static_cast<int32_t>(std::floor(iPos1[1]));

            // Vector from start to end {vx,vy} (in index position coordinates, 1 cell = 1.0)
            double vx = iPos1[0] - iPos0[0];
            double vy = iPos1[1] - iPos0[1];

            // Find direction for raytracing
            int32_t sx = 0;    // direction for raytracing in x direction, either -1,0,+1
            int32_t sy = 0;    // direction for raytracing in y direction, either -1,0,+1
            double dtx = 0.0;  // amount of path variable t to go one more cell in x direction (ray starts at t=0 and stops at t=1)
            double dty = 0.0;  // amount of path variable t to go one more cell in y direction (ray starts at t=0 and stops at t=1)
            double tx = std::numeric_limits<double>::infinity(); // path variable t where the ray intersects with the next cell border in x direction (ray starts at t=0 and stops at t=1)
            double ty = std::numeric_limits<double>::infinity(); // path variable t where the ray intersects with the next cell border in y direction (ray starts at t=0 and stops at t=1)
            double x0 = iPos0[0] - static_cast<double>(ix0);     // start position (x) of line segment in a cell
            double y0 = iPos0[1] - static_cast<double>(iy0);     // start position (y) of line segment in a cell
            double xe = 0.0;   // end position (x) of line segment in a cell
            double ye = 0.0;   // end position (y) of line segment in a cell
            double xs = 0.0;   // initial x position for next cell in x direction during ray tracing
            double ys = 0.0;   // initial y position for next cell in x direction during ray tracing
            if(std::fabs(vx) > 0.0){ // ray makes any progress in x direction
                if(vx > 0.0){
                    sx = 1;
                    tx = std::fabs((1.0 - (iPos0[0] - static_cast<double>(ix0))) / vx);
                    xe = 1.0;
                    xs = 0.0;
                }
                else{
                    sx = -1;
                    tx = std::fabs((static_cast<double>(ix0) - iPos0[0]) / vx);
                    xe = 0.0;
                    xs = 1.0;
                }
                dtx = std::fabs(1.0 / vx);
            }
            if(std::fabs(vy) > 0.0){ // ray makes any progress in y direction
                if(vy > 0.0){
                    sy = 1;
                    ty = std::fabs((1.0 - (iPos0[1] - static_cast<double>(iy0))) / vy);
                    ye = 1.0;
                    ys = 0.0;
                }
                else{
                    sy = -1;
                    ty = std::fabs((static_cast<double>(iy0) - iPos0[1]) / vy);
                    ye = 0.0;
                    ys = 1.0;
                }
                dty = std::fabs(1.0 / vy);
            }

            // Raytracing, start at t=0. The next cell to go is defined by tx,ty (the next intersection cell border is either in x or y direction)
            double t = 0;
            double sum = 0.0;
            int32_t i0 = std::clamp(ix0, 0, this->numX - 1) * this->numY;   // safe lower border x-index for clamped to valid range, all indices (i0,i1) that are outside the range are clamped to the zero border
            int32_t j0 = std::clamp(iy0, 0, this->numY - 1);                // safe lower border y-index clamped to valid range, all indices (j0,j1) that are outside the range are clamped to the zero border
            int32_t i1 = std::clamp(ix0+1, 0, this->numX - 1) * this->numY; // safe lower border x-index for clamped to valid range, all indices (i0,i1) that are outside the range are clamped to the zero border
            int32_t j1 = std::clamp(iy0+1, 0, this->numY - 1);              // safe lower border y-index clamped to valid range, all indices (j0,j1) that are outside the range are clamped to the zero border
            while(t <= 1.0){
                if((ix0 == ix1) && (iy0 == iy1)){ // goal cell reached
                    double x1 = x0 + vx * (1.0 - t);
                    double y1 = y0 + vy * (1.0 - t);
                    double L00 = this->lut[i0 + j0];
                    double L01 = this->lut[i0 + j1];
                    double L10 = this->lut[i1 + j0];
                    double L11 = this->lut[i1 + j1];
                    double dx = x1 - x0;
                    double dy = y1 - y0;
                    double tmp = (x0*y0 + x1*y1);
                    sum += std::sqrt(dx*dx + dy*dy)*(3.0*((L00+L00) + ((L10 - L00)*(x0 + x1) + (L01 - L00)*(y0 + y1))) + (L00 + L11 - L01 - L10)*((tmp+tmp) + (x0*y1 + x1*y0)));
                    break;
                }
                if(tx < ty){ // next cell border in x direction
                    double x1 = xe;
                    double y1 = y0 + vy * (tx - t);
                    double L00 = this->lut[i0 + j0];
                    double L01 = this->lut[i0 + j1];
                    double L10 = this->lut[i1 + j0];
                    double L11 = this->lut[i1 + j1];
                    double dx = x1 - x0;
                    double dy = y1 - y0;
                    double tmp = (x0*y0 + x1*y1);
                    sum += std::sqrt(dx*dx + dy*dy)*(3.0*((L00+L00) + ((L10 - L00)*(x0 + x1) + (L01 - L00)*(y0 + y1))) + (L00 + L11 - L01 - L10)*((tmp+tmp) + (x0*y1 + x1*y0)));
                    x0 = xs;
                    y0 = y1;
                    t = tx;
                    tx += dtx;
                    ix0 += sx;
                    i0 = std::clamp(ix0, 0, this->numX - 1) * this->numY;
                    i1 = std::clamp(ix0+1, 0, this->numX - 1) * this->numY;
                }
                else{ // next cell border in y direction
                    double x1 = x0 + vx * (ty - t);
                    double y1 = ye;
                    double L00 = this->lut[i0 + j0];
                    double L01 = this->lut[i0 + j1];
                    double L10 = this->lut[i1 + j0];
                    double L11 = this->lut[i1 + j1];
                    double dx = x1 - x0;
                    double dy = y1 - y0;
                    double tmp = (x0*y0 + x1*y1);
                    sum += std::sqrt(dx*dx + dy*dy)*(3.0*((L00+L00) + ((L10 - L00)*(x0 + x1) + (L01 - L00)*(y0 + y1))) + (L00 + L11 - L01 - L10)*((tmp+tmp) + (x0*y1 + x1*y0)));
                    x0 = x1;
                    y0 = ys;
                    t = ty;
                    ty += dty;
                    iy0 += sy;
                    j0 = std::clamp(iy0, 0, this->numY - 1);
                    j1 = std::clamp(iy0+1, 0, this->numY - 1);
                }
            }
            return sum * this->res / 6.0; // sum is calculated for cell units, transform to meters by taking cell resolution into account, div by 6 comes from integral (constant that is moved to outside the sum)
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        void WriteToFile(mpsv::core::DataLogFile& file, std::string preString) noexcept {
            file.WriteField("double", preString + "data", {static_cast<uint32_t>(this->numY), static_cast<uint32_t>(this->numX)}, &this->lut[0], sizeof(double) * this->lut.size());
            file.WriteField("double", preString + "resolution", {1}, &this->res, sizeof(double));
            file.WriteField("double", preString + "position", {2,1}, &this->position[0], sizeof(double) * this->position.size());
            file.WriteField("double", preString + "normal", {2,1}, &this->normal[0], sizeof(double) * this->normal.size());
        }
};


} /* namespace: core */


} /* namespace: mpsv */

