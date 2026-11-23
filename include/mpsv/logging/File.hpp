#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>


namespace mpsv {


namespace logging {


/**
 * @brief This class represents a data log file to be used to store data to a binary file.
 * By default, opening an existing file replaces its contents.
 */
class File {
    public:
        /**
         * @brief Construct a new data log file object via default construction.
         */
        File() noexcept = default;

        /**
         * @brief Destroy the data log file object.
         * @details This will also close the data log file.
         */
        ~File() noexcept {
            Close();
        }

        /* Make this class non-copyable */
        File(const File&) = delete;
        File& operator=(const File&) = delete;
        File(File&&) = delete;
        File& operator=(File&&) = delete;

        /**
         * @brief Open the data log file and write header data.
         * @param[in] fileName The name of the file to be opened.
         * @param[in] append True if file should be opened in appending mode, defaults to false.
         * @return True if success, false otherwise.
         */
        [[nodiscard]] bool Open(const std::string& fileName, bool append = false) noexcept {
            m_ofs.open(fileName, (append ? (std::ofstream::binary | std::ofstream::ate | std::ofstream::app) : (std::ofstream::binary | std::ofstream::ate)));
            if (!m_ofs.is_open()) {
                return false;
            }
            WriteHeaderData();
            return true;
        }

        /**
         * @brief Check whether the file is open and the stream has no errors.
         * @return True if the stream is open and good, false otherwise.
         */
        [[nodiscard]] bool IsGood() const noexcept {
            return m_ofs.is_open() && m_ofs.good();
        }

        /**
         * @brief Close the data log file.
         */
        void Close() noexcept {
            m_ofs.close();
        }

        /**
         * @brief Write one field to the data log file.
         * @param[in] type String indicating the data type of the value.
         * @param[in] name String indicating the name of the value.
         * @param[in] dimension Vector of numeric values indicating the dimension of the value.
         * @param[in] value Pointer to the value buffer. For matrices, column-major order is used.
         * @param[in] valueSize Binary size of the value buffer.
         */
        void WriteField(std::string type, std::string name, std::vector<std::uint32_t> dimension, void* value, std::uint64_t valueSize) noexcept {
            // Remove zero-terminators from strings
            type.erase(std::find(type.begin(), type.end(), '\0'), type.end());
            name.erase(std::find(name.begin(), name.end(), '\0'), name.end());

            // Validate inputs
            constexpr std::uint64_t maxUint32{std::numeric_limits<std::uint32_t>::max()};
            constexpr std::uint64_t maxUint64{std::numeric_limits<std::uint64_t>::max()};
            if ((type.size() > maxUint32) || (name.size() > maxUint32) || (dimension.size() > maxUint32) || (valueSize > static_cast<std::uint64_t>(std::numeric_limits<std::streamsize>::max())) || ((valueSize > 0U) && (value == nullptr)) || !m_ofs.is_open() || !m_ofs.good()) {
                return;
            }
            std::uint64_t numElements{1U};
            for (std::uint32_t d : dimension) {
                if ((d != 0U) && (numElements > (maxUint64 / d))) {
                    return;
                }
                numElements *= d;
            }

            // Get lengths and compute field size
            std::uint32_t lengthOfType{static_cast<std::uint32_t>(type.size())};
            std::uint32_t lengthOfName{static_cast<std::uint32_t>(name.size())};
            std::uint32_t numberOfDimensions{static_cast<std::uint32_t>(dimension.size())};
            constexpr std::uint64_t fieldMetadataSize{sizeof(lengthOfType) + sizeof(lengthOfName) + sizeof(numberOfDimensions) + sizeof(std::uint64_t)};
            std::uint64_t fieldSize{fieldMetadataSize + type.size() + name.size() + sizeof(std::uint32_t) * dimension.size()};
            if (valueSize > (maxUint64 - fieldSize)) {
                return;
            }
            fieldSize += valueSize;

            // Write field start
            constexpr char fieldStart[] = {'#', '#'};
            m_ofs.write(&fieldStart[0U], sizeof(fieldStart));

            // Write field size
            m_ofs.write(reinterpret_cast<const char*>(&fieldSize), sizeof(fieldSize));

            // Write type
            m_ofs.write(reinterpret_cast<const char*>(&lengthOfType), sizeof(lengthOfType));
            m_ofs.write(type.c_str(), type.size());

            // Write name
            m_ofs.write(reinterpret_cast<const char*>(&lengthOfName), sizeof(lengthOfName));
            m_ofs.write(name.c_str(), name.size());

            // Write dimension
            m_ofs.write(reinterpret_cast<const char*>(&numberOfDimensions), sizeof(numberOfDimensions));
            for (auto&& d : dimension) {
                m_ofs.write(reinterpret_cast<const char*>(&d), sizeof(d));
            }

            // Write value
            m_ofs.write(reinterpret_cast<const char*>(&valueSize), sizeof(valueSize));
            m_ofs.write(static_cast<const char*>(value), valueSize);
        }

        /**
         * @brief Write a vector of array<double,Tdim> to the data log file, with Tdim being the number of values per array.
         * @tparam Tdim The number of values per array.
         * @param[in] name String indicating the name of the value.
         * @param[in] values Vector containing the array values to be written to the data log file.
         */
        template<std::size_t Tdim> void WriteVectorField(std::string name, const std::vector<std::array<double, Tdim>>& values) noexcept {
            std::uint32_t N{static_cast<std::uint32_t>(values.size())};
            std::vector<double> vecData;
            vecData.reserve(N * Tdim);
            for (std::uint32_t n{0U}; n != N; ++n) {
                for (std::size_t i{0U}; i < Tdim; ++i) {
                    vecData.push_back(values[n][i]);
                }
            }
            if (N) {
                WriteField("double", name, {Tdim, N}, &vecData[0U], sizeof(double) * vecData.size());
            }
        }

        /**
         * @brief Write convex polygons to the data log file.
         * @tparam TPolygon The class representing the convex polygon data type. This class must be derived from mpsv::geometry::ConvexPolygon.
         * @param[in] name String indicating the name of the value.
         * @param[in] convexPolygons Vector containing the convex polygons to be written to the data log file.
         */
        template<class TPolygon> void WriteConvexPolygonsField(std::string name, const std::vector<TPolygon>& convexPolygons) noexcept {
            static_assert(std::is_base_of<mpsv::geometry::ConvexPolygon, TPolygon>::value, "Template class TPolygon must be derived from mpsv::geometry::ConvexPolygon!");
            std::uint32_t numConvexPolygons{static_cast<std::uint32_t>(convexPolygons.size())};
            std::vector<double> vecVert;
            for (std::uint32_t i{0U}; i != numConvexPolygons; ++i) {
                std::vector<std::array<double, 2U>> vertices = convexPolygons[i].GetVertices();
                std::uint32_t N{static_cast<std::uint32_t>(vertices.size())};
                vecVert.clear();
                vecVert.reserve(N * 2U);
                for (std::uint32_t n{0U}; n != N; ++n) {
                    vecVert.push_back(vertices[n][0U]);
                    vecVert.push_back(vertices[n][1U]);
                }
                if (N > 0U) {
                    WriteField("double", name + "{" + std::to_string(i + 1U) + "}", {2U, N}, &vecVert[0U], sizeof(double) * vecVert.size());
                }
            }
        }

    protected:
        std::ofstream m_ofs;   // The internal output file stream object for writing data to a file.

        /**
         * @brief Write header data to the output file.
         */
        void WriteHeaderData() noexcept {
            // Create header data
            namespace chr = std::chrono;
            const chr::system_clock::time_point now{chr::system_clock::now()};
            const chr::sys_days today{chr::floor<chr::days>(now)};
            const chr::hh_mm_ss<chr::nanoseconds> timeOfDay{chr::duration_cast<chr::nanoseconds>(now - today)};
            const chr::year_month_day ymd{today};
            const std::int32_t nanosecond{static_cast<std::int32_t>(timeOfDay.subseconds().count())};
            const std::int32_t second{static_cast<std::int32_t>(timeOfDay.seconds().count())};
            const std::int32_t minute{static_cast<std::int32_t>(timeOfDay.minutes().count())};
            const std::int32_t hour{static_cast<std::int32_t>(timeOfDay.hours().count())};
            const std::uint32_t day{static_cast<std::uint32_t>(static_cast<unsigned int>(ymd.day()))};
            const std::uint32_t month{static_cast<std::uint32_t>(static_cast<unsigned int>(ymd.month()))};
            const std::uint32_t year{static_cast<std::uint32_t>(static_cast<int>(ymd.year()))};
            const double timestampUTC{3600.0 * static_cast<double>(hour) + 60.0 * static_cast<double>(minute) + static_cast<double>(second) + 1.0e-9 * static_cast<double>(nanosecond)};
            constexpr char logStart[] = {'$', '$'};
            const std::uint32_t headerSize{sizeof(year) + sizeof(month) + sizeof(day) + sizeof(timestampUTC)};

            // Write header data
            m_ofs.write(&logStart[0U], sizeof(logStart));
            m_ofs.write(reinterpret_cast<const char*>(&headerSize), sizeof(headerSize));
            m_ofs.write(reinterpret_cast<const char*>(&year), sizeof(year));
            m_ofs.write(reinterpret_cast<const char*>(&month), sizeof(month));
            m_ofs.write(reinterpret_cast<const char*>(&day), sizeof(day));
            m_ofs.write(reinterpret_cast<const char*>(&timestampUTC), sizeof(timestampUTC));
        }
};


} /* namespace: core */


} /* namespace: mpsv */
