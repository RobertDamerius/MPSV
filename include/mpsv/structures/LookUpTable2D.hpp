#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/logging/File.hpp>


namespace mpsv {


namespace structures {


/**
 * @brief This class represents a 2D look-up table of precalculated values.
 * @tparam Targ A user-defined argument that is passed to the callback function for calculating look-up table data.
 * @tparam Tdata The datatype of the value to be stored in the 2D look-up table.
 */
template <class Targ, class Tdata> class LookUpTable2D {
    static_assert((mpsv::is_vec2<Tdata>::value) || (std::is_floating_point<Tdata>::value), "Data type must be a floating point or an N-dimensional array of type double with N >= 2!");

    public:
        /**
         * @brief Construct a new 2-dimensional look-up table object.
         */
        LookUpTable2D() noexcept {
            Resize(9);
        }

        /**
         * @brief Destroy the 2-dimensional look-up table object.
         */
        virtual ~LookUpTable2D() noexcept {}

        /**
         * @brief Build the 2D look-up table by applying a new size and calculating new multi-dimensional table values.
         * @param[in] orientedBox The oriented box that represents the area of the LUT.
         * @param[in] resolution Resolution of the grid map (dimension of one cell). This value must be at least 1.0e-3 and is automatically increased if the dimension is too large.
         * @param[in] modInterpolate A modulo factor that indicates when to calculate the data using the LUT function and when to do linear interpolation.
         * @param[in] args User-defined arguments passed to the LUT data callback function.
         */
        void Build(const mpsv::geometry::OrientedBox& orientedBox, double resolution, int32_t modInterpolate, const Targ& args) noexcept {
            // Ensure correct values
            modInterpolate = std::max(static_cast<int32_t>(1),modInterpolate);
            std::array<double,2> dimension = orientedBox.GetDimension();

            // Adjust lower/upper limit of resolution based on maximum number of cells
            double a = 4.0 - static_cast<double>(m_lut.size()); // maximum value is -5
            double b = 2.0 * (dimension[0] + dimension[1]);
            double c = dimension[0] * dimension[1];
            double r = (b + std::sqrt(b * b - 4.0 * a * c)) / (-2.0 * a);
            m_res = std::max(1.0e-3, std::max(resolution, r));

            // Get box parameters and calculate the LUT dimension based on size, resolution and modulo factor
            m_position = orientedBox.GetPosition();
            m_normal = orientedBox.GetNormal();
            int32_t ux, uy, mx, my;
            while(true){
                ux = static_cast<int32_t>(std::ceil(dimension[0] / m_res));
                uy = static_cast<int32_t>(std::ceil(dimension[1] / m_res));
                mx = (ux - 1) % modInterpolate;
                my = (uy - 1) % modInterpolate;
                ux += mx ? (modInterpolate - mx) : 0;
                uy += my ? (modInterpolate - my) : 0;
                m_numX = 2 + std::max(static_cast<int32_t>(1), ux);
                m_numY = 2 + std::max(static_cast<int32_t>(1), uy);
                size_t numCells = static_cast<size_t>(m_numX) * static_cast<size_t>(m_numY);
                if(numCells <= m_lut.size()){
                    break;
                }
                m_res *= std::sqrt(static_cast<double>(numCells) / static_cast<double>(m_lut.size()));
            }

            // Initialize the active LUT region and set edges to zero
            int32_t i0 = (m_numX - 1) * m_numY;
            for(int32_t i = 0; i < m_numY; ++i){
                m_lut[i] = m_lut[i0 + i] = {0.0};
            }
            for(int32_t i = 0, i0 = 0; i < m_numX; ++i, i0 += m_numY){
                m_lut[i0] = m_lut[i0 + m_numY - 1] = {0.0};
            }

            // Center position (x0,y0) of corner grid data cell [0,0] == center position of lut[1,1]
            double d = 0.5 * m_res;
            double x0 = m_position[0] + d * (m_normal[0] - m_normal[1]);
            double y0 = m_position[1] + d * (m_normal[1] + m_normal[0]);

            // Calculate cost on specific grid points (modulo grid points)
            // A 2-by-3 grid with a modulo factor of 3 looks like this (note that there is a border of zeros around the actual data):
            // 
            //  0 0 0 0 0 0 0 0 0 0 0 0
            //  0 x . . x . . x . . x 0
            //  0 . . . . . . . . . . 0
            //  0 . . . . . . . . . . 0
            //  0 x . . x . . x . . x 0
            //  0 . . . . . . . . . . 0
            //  0 . . . . . . . . . . 0
            //  0 x . . x . . x . . x 0
            //  0 0 0 0 0 0 0 0 0 0 0 0
            // 
            // The following loop only calculates the cost for marked cells (x).
            int32_t numBreakpointsX = ((m_numX - 3) / modInterpolate) + 1;
            int32_t numBreakpointsY = ((m_numY - 3) / modInterpolate) + 1;
            int32_t numTotal = numBreakpointsX * numBreakpointsY;
            double dX = m_res * m_normal[0];
            double dY = m_res * m_normal[1];
            #pragma omp parallel for shared(m_lut)
            for(int32_t i = 0; i < numTotal; ++i){
                int32_t bi = (i / numBreakpointsY) * modInterpolate;
                int32_t bj = (i % numBreakpointsY) * modInterpolate;
                int32_t index = (bi + 1) * m_numY + (bj + 1);
                double x = x0 + static_cast<double>(bi)*dX - static_cast<double>(bj)*dY;
                double y = y0 + static_cast<double>(bi)*dY + static_cast<double>(bj)*dX;
                m_lut[index] = CallbackTableData(x, y, args);
            }

            // Linear interpolation of "non-modul" grid points
            // Going on with the actual LUT, x indicates the grids for which cost values have been calculated.
            // 
            //  0 0 0 0 0 0 0 0 0 0 0 0
            //  0 x . . x . . x . . x 0
            //  0 + + + - - - + + + . 0
            //  0 + + + - - - + + + . 0
            //  0 x + + x - - x + + x 0
            //  0 - - - + + + - - - . 0
            //  0 - - - + + + - - - . 0
            //  0 x - - x + + x - - x 0
            //  0 0 0 0 0 0 0 0 0 0 0 0
            // 
            // The following loop interpolates the grid data (+,-) for all blocks. One block is indicated by a square of 'x'. The last row, column before the zero border is handled later.
            int32_t numBlocksX = ux / modInterpolate;
            int32_t numBlocksY = uy / modInterpolate;
            double invMod = 1.0 / static_cast<double>(modInterpolate);
            double invMod2 = 1.0 / static_cast<double>(modInterpolate * modInterpolate);
            numTotal = numBlocksX * numBlocksY;
            #pragma omp parallel for shared(m_lut)
            for(int32_t i = 0; i < numTotal; ++i){
                int32_t i00 = (i / numBlocksY) * modInterpolate + 1;
                int32_t j00 = (i % numBlocksY) * modInterpolate + 1;
                int32_t index = i00 * m_numY + j00;
                int32_t mn = modInterpolate * m_numY;
                Tdata c00 = m_lut[index];
                Tdata c10 = m_lut[index + mn];
                Tdata c01 = m_lut[index + modInterpolate];
                Tdata c11 = m_lut[index + mn + modInterpolate];
                Tdata d1 = (c10 - c00) * invMod;
                Tdata d2 = (c01 - c00) * invMod;
                Tdata d3 = (c00 + c11 - c01 - c10) * invMod2;
                Tdata sd1 = {0.0};
                Tdata sd3 = {0.0};
                for(int32_t ix = 0, w = 0; ix < modInterpolate; ++ix, w += m_numY){
                    Tdata sd2 = {0.0};
                    Tdata ssd3 = {0.0};
                    for(int32_t iy = 0; iy < modInterpolate; ++iy){
                        m_lut[index + w + iy] = c00 + sd1 + sd2 + ssd3;
                        sd2 += d2;
                        ssd3 += sd3;
                    }
                    sd1 += d1;
                    sd3 += d3;
                }
            }

            // Go on with the remaining edges. In the following two loops we assign the values for A and B which gives us the following LUT:
            // 
            //  0 0 0 0 0 0 0 0 0 0 0 0
            //  0 x B B x B B x B B x 0
            //  0 + + + - - - + + + A 0
            //  0 + + + - - - + + + A 0
            //  0 x + + x - - x + + x 0
            //  0 - - - + + + - - - A 0
            //  0 - - - + + + - - - A 0
            //  0 x - - x + + x - - x 0
            //  0 0 0 0 0 0 0 0 0 0 0 0
            int32_t ix0, ix1, iy0, iy1;
            Tdata c10, c01, c11, sd1;
            mx = modInterpolate * m_numY;
            Tdata vd;
            ix0 = m_numY;
            iy1 = m_numY - 2;
            for(int32_t bx = 0; bx < numBlocksX; ++bx){
                ix1 = ix0 + mx;
                c01 = m_lut[ix0 + iy1];
                c11 = m_lut[ix1 + iy1];
                vd = (c11 - c01) * invMod;
                sd1 = vd;
                for(int32_t ix = m_numY; ix < mx; ix += m_numY){
                    m_lut[ix0 + ix + iy1] = c01 + sd1;
                    sd1 += vd;
                }
                ix0 += mx;
            }
            iy0 = 1;
            ix1 = (m_numX - 2) * m_numY;
            for(int32_t by = 0; by < numBlocksY; ++by){
                iy1 = iy0 + modInterpolate;
                c10 = m_lut[ix1 + iy0];
                c11 = m_lut[ix1 + iy1];
                vd = (c11 - c10) * invMod;
                sd1 = vd;
                for(int32_t iy = 1; iy < modInterpolate; ++iy){
                    m_lut[ix1 + iy0 + iy] = c10 + sd1;
                    sd1 += vd;
                }
                iy0 += modInterpolate;
            }
        }

        /**
         * @brief Resize the look-up table storage and reset it to its default state.
         * @param[in] maxNumCells The maximum number of cells the look-up table should contain.
         */
        void Resize(size_t maxNumCells) noexcept {
            m_position = {0.0, 0.0};
            m_normal = {1.0, 0.0};
            m_res = 1.0;
            m_numX = 0;
            m_numY = 0;
            m_lut.resize(std::max(size_t{9}, maxNumCells));
            m_lut.shrink_to_fit();
        }

        /**
         * @brief Get the nearest  value at a specific position.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] position Position for which to obtain the  value.
         * @return Value at the given position. If the specified position is outside the look-up table area, the internal zero-border value is returned.
         */
        template <class T> [[nodiscard]] Tdata GetValueAtPosition(const T& position) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {position} must be of type std::array<double,N> with N >= 2!");

            // Convert to index position ({0,0} indicates origin of lower-left corner of LUT data cell (lut[0]), {0.5,0.5} would be the center of the lower-left cell, all data corresponds to the center of a cell)
            std::array<double,2> indexPosition = ConvertToLUTPosition(position) / m_res;

            // Floor down to lower-left position of a cell to obtain indices and ensure them to be in a valid range: ix in [0 ... numX), iy in [0 ... numY)
            int32_t ix = std::clamp(static_cast<int32_t>(std::floor(indexPosition[0])), 0, m_numX - 1);
            int32_t iy = std::clamp(static_cast<int32_t>(std::floor(indexPosition[1])), 0, m_numY - 1);
            return m_lut[ix*m_numY + iy];
        }

        /**
         * @brief Get the linear interpolated value at a specific position.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param position Position for which to obtain the value.
         * @return Linear interpolated value at the given position. If the specified position is outside the look-up table area, the internal zero-border value is returned.
         */
        template <class T> [[nodiscard]] Tdata GetLinearInterpolatedValueAtPosition(const T& position) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {position} must be of type std::array<double,N> with N >= 2!");

            // Convert to index position ({0,0} indicates origin of lower-left corner of LUT data cell (lut[0]), {0.5,0.5} would be the center of the lower-left cell, all data corresponds to the center of a cell)
            std::array<double,2> indexPosition = ConvertToLUTPosition(position);
            indexPosition[0] /= m_res;
            indexPosition[1] /= m_res;

            // The center of the cell indicates the data, transform index position to the center such that all corners of the new grid cell correspond to true data points
            indexPosition[0] -= 0.5;
            indexPosition[1] -= 0.5;

            // Get lower left corner index (ix0,iy0) and upper right corner index (ix1,iy1)
            // Clamp those indices to the valid range of the lut: ix* in [0 ... numX), iy* in [0 ... numY)
            int32_t ix0 = static_cast<int32_t>(std::floor(indexPosition[0]));
            int32_t iy0 = static_cast<int32_t>(std::floor(indexPosition[1]));
            double dx = indexPosition[0] - static_cast<double>(ix0);
            double dy = indexPosition[1] - static_cast<double>(iy0);
            int32_t ix1 = std::clamp(ix0 + 1, 0, m_numX - 1);
            int32_t iy1 = std::clamp(iy0 + 1, 0, m_numY - 1);
            ix0 = std::clamp(ix0, 0, m_numX - 1);
            iy0 = std::clamp(iy0, 0, m_numY - 1);

            // Get table data for all four corners
            int32_t ix0_numY = ix0 * m_numY;
            int32_t ix1_numY = ix1 * m_numY;
            const Tdata& lut00 = m_lut[ix0_numY + iy0];
            const Tdata& lut10 = m_lut[ix1_numY + iy0];
            const Tdata& lut01 = m_lut[ix0_numY + iy1];
            const Tdata& lut11 = m_lut[ix1_numY + iy1];

            // Linear interpolation along lower and upper x border followed by linear interpolation in y direction
            Tdata y0 = lut00 + dx * (lut10 - lut00);
            Tdata y1 = lut01 + dx * (lut11 - lut01);
            return y0 + dy * (y1 - y0);
        }

        /**
         * @brief Calculate the line integral along a linear line between two points for all dimensions of the multi-dimensional table data.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @tparam U Template parameter must be of type std::array<double,N> with N >= 2.
         * @param p0 Start point of the line from where to start the line integral, given in earth-fixed coordinates (meters).
         * @param p1 End point of the line where to end the line integral, given in earth-fixed coordinates (meters).
         * @return Final value of the line integral along a linear line over the look-up table data.
         */
        template <class T, class U> [[nodiscard]] Tdata LineIntegral(const T& p0, const U& p1) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {p0} must be of type std::array<double,N> with N >= 2!");
            static_assert(mpsv::is_vec2<U>::value,"Argument {p1} must be of type std::array<double,N> with N >= 2!");

            // Convert to index position ({0,0} indicates origin of lower-left corner of LUT data cell (lut[0]), {0.5,0.5} would be the center of the lower-left cell, all data corresponds to the center of a cell)
            std::array<double,2> iPos0 = ConvertToLUTPosition(p0);
            iPos0[0] /= m_res;
            iPos0[1] /= m_res;
            std::array<double,2> iPos1 = ConvertToLUTPosition(p1);
            iPos1[0] /= m_res;
            iPos1[1] /= m_res;

            // Flooring the fractional index position (iPos) gives the true index
            int32_t ix0 = static_cast<int32_t>(std::floor(iPos0[0]));
            int32_t iy0 = static_cast<int32_t>(std::floor(iPos0[1]));
            int32_t ix1 = static_cast<int32_t>(std::floor(iPos1[0]));
            int32_t iy1 = static_cast<int32_t>(std::floor(iPos1[1]));

            // Vector from start to end {vx,vy} and the length (in index position coordinates, 1 cell = 1.0)
            double vx = iPos1[0] - iPos0[0];
            double vy = iPos1[1] - iPos0[1];
            double L = std::sqrt(vx*vx + vy*vy);

            // Find direction for raytracing
            int32_t sx = 0;    // direction for raytracing in x direction, either -1,0,+1
            int32_t sy = 0;    // direction for raytracing in y direction, either -1,0,+1
            double dtx = 0.0;  // amount of path variable t to go one more cell in x direction (ray starts at t=0 and stops at t=1)
            double dty = 0.0;  // amount of path variable t to go one more cell in y direction (ray starts at t=0 and stops at t=1)
            double tx = std::numeric_limits<double>::infinity(); // path variable t where the ray intersects with the next cell border in x direction (ray starts at t=0 and stops at t=1)
            double ty = std::numeric_limits<double>::infinity(); // path variable t where the ray intersects with the next cell border in y direction (ray starts at t=0 and stops at t=1)
            if(std::fabs(vx) > 0.0){ // ray makes any progress in x direction
                if(vx > 0.0){
                    sx = 1;
                    tx = std::fabs((1.0 - (iPos0[0] - static_cast<double>(ix0))) / vx);
                }
                else{
                    sx = -1;
                    tx = std::fabs((static_cast<double>(ix0) - iPos0[0]) / vx);
                }
                dtx = std::fabs(1.0 / vx);
            }
            if(std::fabs(vy) > 0.0){ // ray makes any progress in y direction
                if(vy > 0.0){
                    sy = 1;
                    ty = std::fabs((1.0 - (iPos0[1] - static_cast<double>(iy0))) / vy);
                }
                else{
                    sy = -1;
                    ty = std::fabs((static_cast<double>(iy0) - iPos0[1]) / vy);
                }
                dty = std::fabs(1.0 / vy);
            }

            // Raytracing, start at t=0. The next cell to go is defined by tx,ty (the next intersection cell border is either in x or y direction)
            double ds;
            double t = 0;
            Tdata sum = 0.0;
            int32_t i = std::clamp(ix0, 0, m_numX - 1) * m_numY; // safe index clamped to valid range, all indices (ix0,ix1) that are outside the range are clamped to the zero border
            int32_t j = std::clamp(iy0, 0, m_numY - 1); // safe index clamped to valid range, all indices (iy0,iy1) that are outside the range are clamped to the zero border
            while(t <= 1.0){
                if((ix0 == ix1) && (iy0 == iy1)){ // goal cell reached
                    ds = (1.0 - t) * L;
                    sum += m_lut[i + j] * ds;
                    break;
                }
                if(tx < ty){ // next cell border in x direction
                    ds = (tx - t) * L;
                    sum += m_lut[i + j] * ds;
                    t = tx;
                    tx += dtx;
                    ix0 += sx;
                    i = std::clamp(ix0, 0, m_numX - 1) * m_numY;
                }
                else{ // next cell border in y direction
                    ds = (ty - t) * L;
                    sum += m_lut[i + j] * ds;
                    t = ty;
                    ty += dty;
                    iy0 += sy;
                    j = std::clamp(iy0, 0, m_numY - 1);
                }
            }
            return sum * m_res; // sum is calculated for cell units, transform to meters by taking cell resolution into account
        }

        /**
         * @brief Write data to the log file.
         * @param[in] file The log file to which to write the data to.
         * @param[in] preString A pre-string to be inserted at the beginning of variable names.
         */
        virtual void WriteToFile(mpsv::logging::File& file, std::string preString) noexcept = 0;

    protected:
        std::array<double,2> m_position{0.0, 0.0};   // Corner position (lower left corner) of the actual LUT (without zero-border: corner of data cell [0,0] == corner of lut[1,1]).
        std::array<double,2> m_normal{1.0, 0.0};     // Normal vector in longitudinal direction of the LUT.
        double m_res{1.0};                           // Resolution of the grid map (dimension of one cell).
        int32_t m_numX{0};                           // Number of grid cells in X (longitudinal) direction.
        int32_t m_numY{0};                           // Number of grid cells in Y (lateral) direction.
        std::vector<Tdata> m_lut{Tdata{0.0}};        // Table data that also contains a zero-border (index = ix*numY + iy). ix and iy indicate the lower-left index position of the cell, however, the actual data of that cell has been evaluated for the center of the cell.

        /**
         * @brief Callback function for evaluating the look-up table data for a specific 2D position.
         * @param[in] x X position for which to evaluate the table data.
         * @param[in] y Y position for which to evaluate the table data.
         * @param[in] args User-defined arguments to be used for evaluating the table data.
         * @return Final multi-dimensional table data value to be stored in the LUT.
         * @note This callback may be invoked concurrently from multiple OpenMP threads. Overrides must be thread-safe.
         */
        virtual Tdata CallbackTableData(double x, double y, const Targ& args) noexcept = 0;

        /**
         * @brief Convert a given point in meters to the corresponding look-up table position in meters.
         * @tparam T Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] p The point in meters to be converted, given in the earth-fixed reference frame.
         * @return Resulting 2-dimensional position in meters according to the LUT data with respect to the lower-left corner of the zero-border.
         * @details The input position is rotated into the coordinate system of the LUT and is translated such that the lower-left of the LUT, which is the zero-border data, indicates the origin {0,0}.
         * The lut position devided by the resolution (@ref m_res) gives the true index position {ix,iy} for the LUT data (@ref m_lut).
         */
        template <class T> std::array<double,2> ConvertToLUTPosition(const T& p) const noexcept {
            static_assert(mpsv::is_vec2<T>::value,"Argument {p} must be of type std::array<double,N> with N >= 2!");
            double ex = p[0] - m_position[0];
            double ey = p[1] - m_position[1];
            return {m_res + m_normal[0]*ex + m_normal[1]*ey, m_res - m_normal[1]*ex + m_normal[0]*ey}; // add one cell of [res,res] to account for the zero-border of the LUT
        }
};


} /* namespace: structures */


} /* namespace: mpsv */
