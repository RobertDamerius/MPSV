#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/OrientedBox.hpp>
#include <mpsv/logging/File.hpp>


namespace mpsv::structures {


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
            Resize(9U);
        }

        /**
         * @brief Destroy the 2-dimensional look-up table object.
         */
        virtual ~LookUpTable2D() noexcept {}

        /**
         * @brief Build the 2D look-up table by applying a new size and calculating new multi-dimensional table values.
         * @param[in] orientedBox The oriented box that represents the area of the LUT.
         * @param[in] resolution Resolution of the grid map (dimension of one cell). This value must be at least 1.0e-3 and is automatically increased if the dimension is too large.
         * @param[in] args User-defined arguments passed to the LUT data callback function.
         */
        void Build(const mpsv::geometry::OrientedBox& orientedBox, double resolution, const Targ& args) noexcept {
            std::array<double, 2U> dimension = orientedBox.GetDimension();

            // Adjust lower/upper limit of resolution based on maximum number of cells
            double a{4.0 - static_cast<double>(m_lut.size())}; // maximum value is -5
            double b{2.0 * (dimension[0U] + dimension[1U])};
            double c{dimension[0U] * dimension[1U]};
            double r{(b + std::sqrt(b * b - 4.0 * a * c)) / (-2.0 * a)};
            m_res = std::max(1.0e-3, std::max(resolution, r));

            // Get box parameters and calculate the LUT dimension based on size and resolution
            m_position = orientedBox.GetPosition();
            m_normal = orientedBox.GetNormal();
            std::int32_t ux, uy;
            while (true) {
                ux = std::max(std::int32_t{1}, static_cast<std::int32_t>(std::ceil(dimension[0U] / m_res)));
                uy = std::max(std::int32_t{1}, static_cast<std::int32_t>(std::ceil(dimension[1U] / m_res)));
                m_numX = 2 + ux;
                m_numY = 2 + uy;
                std::size_t numCells{static_cast<std::size_t>(m_numX) * static_cast<std::size_t>(m_numY)};
                if (numCells <= m_lut.size()) {
                    break;
                }
                m_res *= std::sqrt(static_cast<double>(numCells) / static_cast<double>(m_lut.size()));
            }

            // Initialize the active LUT region and set edges to zero
            std::int32_t i0{(m_numX - 1) * m_numY};
            for (std::int32_t i{0}; i < m_numY; ++i) {
                m_lut[i] = m_lut[i0 + i] = {0.0};
            }
            for (std::int32_t i{0}, i0{0}; i < m_numX; ++i, i0 += m_numY) {
                m_lut[i0] = m_lut[i0 + m_numY - 1] = {0.0};
            }

            // Center position (x0,y0) of corner grid data cell [0,0] == center position of lut[1,1]
            double d{0.5 * m_res};
            double x0{m_position[0U] + d * (m_normal[0U] - m_normal[1U])};
            double y0{m_position[1U] + d * (m_normal[1U] + m_normal[0U])};

            // Calculate the data for every cell in the active LUT region
            std::int32_t numTotal{ux * uy};
            double dX{m_res * m_normal[0U]};
            double dY{m_res * m_normal[1U]};
            #pragma omp parallel for shared(m_lut)
            for (std::int32_t i = 0; i < numTotal; ++i) {
                std::int32_t bi{i / uy};
                std::int32_t bj{i % uy};
                std::int32_t index{(bi + 1) * m_numY + (bj + 1)};
                double x{x0 + static_cast<double>(bi)*dX - static_cast<double>(bj)*dY};
                double y{y0 + static_cast<double>(bi)*dY + static_cast<double>(bj)*dX};
                m_lut[index] = CallbackTableData(x, y, args);
            }
        }

        /**
         * @brief Resize the look-up table storage and reset it to its default state.
         * @param[in] maxNumCells The maximum number of cells the look-up table should contain.
         */
        void Resize(std::uint32_t maxNumCells) noexcept {
            m_position = {0.0, 0.0};
            m_normal = {1.0, 0.0};
            m_res = 1.0;
            m_numX = 0;
            m_numY = 0;
            m_lut.resize(std::max(std::uint32_t{9U}, maxNumCells));
            m_lut.shrink_to_fit();
        }

        /**
         * @brief Get the nearest  value at a specific position.
         * @tparam Tposition Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] position Position for which to obtain the  value.
         * @return Value at the given position. If the specified position is outside the look-up table area, the internal zero-border value is returned.
         */
        template <class Tposition> [[nodiscard]] Tdata GetValueAtPosition(const Tposition& position) const noexcept {
            static_assert(mpsv::is_vec2<Tposition>::value, "Argument {position} must be of type std::array<double,N> with N >= 2!");

            // Convert to index position ({0,0} indicates origin of lower-left corner of LUT data cell (lut[0]), {0.5,0.5} would be the center of the lower-left cell, all data corresponds to the center of a cell)
            std::array<double, 2U> indexPosition = ConvertToLUTPosition(position) / m_res;

            // Floor down to lower-left position of a cell to obtain indices and ensure them to be in a valid range: ix in [0 ... numX), iy in [0 ... numY)
            std::int32_t ix{std::clamp(static_cast<std::int32_t>(std::floor(indexPosition[0U])), 0, m_numX - 1)};
            std::int32_t iy{std::clamp(static_cast<std::int32_t>(std::floor(indexPosition[1U])), 0, m_numY - 1)};
            return m_lut[ix*m_numY + iy];
        }

        /**
         * @brief Get the linear interpolated value at a specific position.
         * @tparam Tposition Template parameter must be of type std::array<double,N> with N >= 2.
         * @param position Position for which to obtain the value.
         * @return Linear interpolated value at the given position. If the specified position is outside the look-up table area, the internal zero-border value is returned.
         */
        template <class Tposition> [[nodiscard]] Tdata GetLinearInterpolatedValueAtPosition(const Tposition& position) const noexcept {
            static_assert(mpsv::is_vec2<Tposition>::value, "Argument {position} must be of type std::array<double,N> with N >= 2!");

            // Convert to index position ({0,0} indicates origin of lower-left corner of LUT data cell (lut[0]), {0.5,0.5} would be the center of the lower-left cell, all data corresponds to the center of a cell)
            std::array<double, 2U> indexPosition = ConvertToLUTPosition(position);
            indexPosition[0U] /= m_res;
            indexPosition[1U] /= m_res;

            // The center of the cell indicates the data, transform index position to the center such that all corners of the new grid cell correspond to true data points
            indexPosition[0U] -= 0.5;
            indexPosition[1U] -= 0.5;

            // Get lower left corner index (ix0,iy0) and upper right corner index (ix1,iy1)
            // Clamp those indices to the valid range of the lut: ix* in [0 ... numX), iy* in [0 ... numY)
            std::int32_t ix0{static_cast<std::int32_t>(std::floor(indexPosition[0U]))};
            std::int32_t iy0{static_cast<std::int32_t>(std::floor(indexPosition[1U]))};
            double dx{indexPosition[0U] - static_cast<double>(ix0)};
            double dy{indexPosition[1U] - static_cast<double>(iy0)};
            std::int32_t ix1{std::clamp(ix0 + 1, 0, m_numX - 1)};
            std::int32_t iy1{std::clamp(iy0 + 1, 0, m_numY - 1)};
            ix0 = std::clamp(ix0, 0, m_numX - 1);
            iy0 = std::clamp(iy0, 0, m_numY - 1);

            // Get table data for all four corners
            std::int32_t ix0_numY{ix0 * m_numY};
            std::int32_t ix1_numY{ix1 * m_numY};
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
         * @tparam Tpoint0 Template parameter must be of type std::array<double,N> with N >= 2.
         * @tparam Tpoint1 Template parameter must be of type std::array<double,N> with N >= 2.
         * @param p0 Start point of the line from where to start the line integral, given in earth-fixed coordinates (meters).
         * @param p1 End point of the line where to end the line integral, given in earth-fixed coordinates (meters).
         * @return Final value of the line integral along a linear line over the look-up table data.
         */
        template <class Tpoint0, class Tpoint1> [[nodiscard]] Tdata LineIntegral(const Tpoint0& p0, const Tpoint1& p1) const noexcept {
            static_assert(mpsv::is_vec2<Tpoint0>::value, "Argument {p0} must be of type std::array<double,N> with N >= 2!");
            static_assert(mpsv::is_vec2<Tpoint1>::value, "Argument {p1} must be of type std::array<double,N> with N >= 2!");

            // Convert to index position ({0,0} indicates origin of lower-left corner of LUT data cell (lut[0]), {0.5,0.5} would be the center of the lower-left cell, all data corresponds to the center of a cell)
            std::array<double, 2U> iPos0 = ConvertToLUTPosition(p0);
            iPos0[0U] /= m_res;
            iPos0[1U] /= m_res;
            std::array<double, 2U> iPos1 = ConvertToLUTPosition(p1);
            iPos1[0U] /= m_res;
            iPos1[1U] /= m_res;

            // Flooring the fractional index position (iPos) gives the true index
            std::int32_t ix0{static_cast<std::int32_t>(std::floor(iPos0[0U]))};
            std::int32_t iy0{static_cast<std::int32_t>(std::floor(iPos0[1U]))};
            std::int32_t ix1{static_cast<std::int32_t>(std::floor(iPos1[0U]))};
            std::int32_t iy1{static_cast<std::int32_t>(std::floor(iPos1[1U]))};

            // Vector from start to end {vx,vy} and the length (in index position coordinates, 1 cell = 1.0)
            double vx{iPos1[0U] - iPos0[0U]};
            double vy{iPos1[1U] - iPos0[1U]};
            double L{std::sqrt(vx*vx + vy*vy)};

            // Find direction for raytracing
            std::int32_t sx{0}; // direction for raytracing in x direction, either -1,0,+1
            std::int32_t sy{0}; // direction for raytracing in y direction, either -1,0,+1
            double dtx{0.0}; // amount of path variable t to go one more cell in x direction (ray starts at t=0 and stops at t=1)
            double dty{0.0}; // amount of path variable t to go one more cell in y direction (ray starts at t=0 and stops at t=1)
            double tx{std::numeric_limits<double>::infinity()}; // path variable t where the ray intersects with the next cell border in x direction (ray starts at t=0 and stops at t=1)
            double ty{std::numeric_limits<double>::infinity()}; // path variable t where the ray intersects with the next cell border in y direction (ray starts at t=0 and stops at t=1)
            if (std::fabs(vx) > 0.0) { // ray makes any progress in x direction
                if (vx > 0.0) {
                    sx = 1;
                    tx = std::fabs((1.0 - (iPos0[0U] - static_cast<double>(ix0))) / vx);
                } else {
                    sx = -1;
                    tx = std::fabs((static_cast<double>(ix0) - iPos0[0U]) / vx);
                }
                dtx = std::fabs(1.0 / vx);
            }
            if (std::fabs(vy) > 0.0) { // ray makes any progress in y direction
                if (vy > 0.0) {
                    sy = 1;
                    ty = std::fabs((1.0 - (iPos0[1U] - static_cast<double>(iy0))) / vy);
                } else {
                    sy = -1;
                    ty = std::fabs((static_cast<double>(iy0) - iPos0[1U]) / vy);
                }
                dty = std::fabs(1.0 / vy);
            }

            // Raytracing, start at t=0. The next cell to go is defined by tx,ty (the next intersection cell border is either in x or y direction)
            double ds;
            double t{0};
            Tdata sum = 0.0;
            std::int32_t i{std::clamp(ix0, 0, m_numX - 1) * m_numY}; // safe index clamped to valid range, all indices (ix0,ix1) that are outside the range are clamped to the zero border
            std::int32_t j{std::clamp(iy0, 0, m_numY - 1)}; // safe index clamped to valid range, all indices (iy0,iy1) that are outside the range are clamped to the zero border
            while (t <= 1.0) {
                if ((ix0 == ix1) && (iy0 == iy1)) { // goal cell reached
                    ds = (1.0 - t) * L;
                    sum += m_lut[i + j] * ds;
                    break;
                }
                if (tx < ty) { // next cell border in x direction
                    ds = (tx - t) * L;
                    sum += m_lut[i + j] * ds;
                    t = tx;
                    tx += dtx;
                    ix0 += sx;
                    i = std::clamp(ix0, 0, m_numX - 1) * m_numY;
                } else { // next cell border in y direction
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
        std::array<double, 2U> m_position{0.0, 0.0};   // Corner position (lower left corner) of the actual LUT (without zero-border: corner of data cell [0,0] == corner of lut[1,1]).
        std::array<double, 2U> m_normal{1.0, 0.0};     // Normal vector in longitudinal direction of the LUT.
        double m_res{1.0};                             // Resolution of the grid map (dimension of one cell).
        std::int32_t m_numX{0};                        // Number of grid cells in X (longitudinal) direction.
        std::int32_t m_numY{0};                        // Number of grid cells in Y (lateral) direction.
        std::vector<Tdata> m_lut{Tdata{0.0}};          // Table data that also contains a zero-border (index = ix*numY + iy). ix and iy indicate the lower-left index position of the cell, however, the actual data of that cell has been evaluated for the center of the cell.

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
         * @tparam Tposition Template parameter must be of type std::array<double,N> with N >= 2.
         * @param[in] p The point in meters to be converted, given in the earth-fixed reference frame.
         * @return Resulting 2-dimensional position in meters according to the LUT data with respect to the lower-left corner of the zero-border.
         * @details The input position is rotated into the coordinate system of the LUT and is translated such that the lower-left of the LUT, which is the zero-border data, indicates the origin {0,0}.
         * The lut position devided by the resolution (@ref m_res) gives the true index position {ix,iy} for the LUT data (@ref m_lut).
         */
        template <class Tposition> std::array<double, 2U> ConvertToLUTPosition(const Tposition& p) const noexcept {
            static_assert(mpsv::is_vec2<Tposition>::value, "Argument {p} must be of type std::array<double,N> with N >= 2!");
            double ex{p[0U] - m_position[0U]};
            double ey{p[1U] - m_position[1U]};
            return {m_res + m_normal[0U]*ex + m_normal[1U]*ey, m_res - m_normal[1U]*ex + m_normal[0U]*ey}; // add one cell of [res,res] to account for the zero-border of the LUT
        }
};


} /* namespace: mpsv::structures */
