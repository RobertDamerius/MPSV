#pragma once


#include <mpsv/common/Common.hpp>
#include <mpsv/geometry/ConvexPolygon.hpp>


namespace mpsv {


namespace logging {


/**
 * @brief This class represents a data log file to be used to store data to a binary file.
 * The file is always opened in appending mode. That is, an existing data log file is not deleted, but new data is appended.
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
        ~File() noexcept { Close(); }

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
            if(!m_ofs.is_open()){
                return false;
            }
            WriteHeaderData();
            return true;
        }

        /**
         * @brief Close the data log file.
         */
        void Close(void) noexcept { m_ofs.close(); }

        /**
         * @brief Write one field to the data log file.
         * @param[in] type String indicating the data type of the value.
         * @param[in] name String indicating the name of the value.
         * @param[in] dimension Vector of numeric values indicating the dimension of the value.
         * @param[in] value Pointer to the value buffer. For matrices, column-major order is used.
         * @param[in] valueSize Binary size of the value buffer.
         */
        void WriteField(std::string type, std::string name, std::vector<uint32_t> dimension, void* value, uint64_t valueSize) noexcept {
            // Remove zero-terminators from strings
            type.erase(std::find(type.begin(), type.end(), '\0'), type.end());
            name.erase(std::find(name.begin(), name.end(), '\0'), name.end());

            // Get lengths and compute field size
            uint32_t lengthOfType = static_cast<uint32_t>(type.size());
            uint32_t lengthOfName = static_cast<uint32_t>(name.size());
            uint32_t numberOfDimensions = static_cast<uint32_t>(dimension.size());
            uint64_t fieldSize = sizeof(lengthOfType) + type.size() + sizeof(lengthOfName) + name.size() + sizeof(numberOfDimensions) + sizeof(uint32_t) * dimension.size() + sizeof(valueSize) + valueSize;

            // Write field start
            constexpr char fieldStart[] = {'#','#'};
            m_ofs.write(&fieldStart[0], sizeof(fieldStart));

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
            for(auto&& d : dimension)
                m_ofs.write(reinterpret_cast<const char*>(&d), sizeof(d));

            // Write value
            m_ofs.write(reinterpret_cast<const char*>(&valueSize), sizeof(valueSize));
            m_ofs.write(static_cast<const char*>(value), valueSize);
        }

        /**
         * @brief Write a vector of array<double,T> to the data log file, with T being the number of values per array.
         * @tparam T The number of values per array.
         * @param[in] name String indicating the name of the value.
         * @param[in] values Vector containing the array values to be written to the data log file.
         */
        template<size_t T> void WriteVectorField(std::string name, const std::vector<std::array<double,T>>& values) noexcept {
            uint32_t N = static_cast<uint32_t>(values.size());
            std::vector<double> vecData;
            vecData.reserve(N * T);
            for(uint32_t n = 0; n != N; ++n){
                for(size_t i = 0; i < T; ++i){
                    vecData.push_back(values[n][i]);
                }
            }
            if(N){
                WriteField("double", name, {T, N}, &vecData[0], sizeof(double) * vecData.size());
            }
        }

        /**
         * @brief Write convex polygons to the data log file.
         * @tparam T The class representing the convex polygon data type. This class must be derived from mpsv::geometry::ConvexPolygon.
         * @param[in] name String indicating the name of the value.
         * @param[in] convexPolygons Vector containing the convex polygons to be written to the data log file.
         */
        template<class T> void WriteConvexPolygonsField(std::string name, const std::vector<T>& convexPolygons) noexcept {
            static_assert(std::is_base_of<mpsv::geometry::ConvexPolygon,T>::value, "Template class T must be derived from mpsv::geometry::ConvexPolygon!");
            uint32_t numConvexPolygons = static_cast<uint32_t>(convexPolygons.size());
            std::vector<double> vecVert;
            for(uint32_t i = 0; i != numConvexPolygons; ++i){
                std::vector<std::array<double,2>> vertices = convexPolygons[i].GetVertices();
                uint32_t N = static_cast<uint32_t>(vertices.size());
                vecVert.clear();
                vecVert.reserve(N * 2);
                for(uint32_t n = 0; n != N; ++n){
                    vecVert.push_back(vertices[n][0]);
                    vecVert.push_back(vertices[n][1]);
                }
                if(N){
                    WriteField("double", name + "{" + std::to_string(i + 1) + "}", {2, N}, &vecVert[0], sizeof(double) * vecVert.size());
                }
            }
        }

    protected:
        std::ofstream m_ofs;   // The internal output file stream object for writing data to a file.

        /**
         * @brief Write header data to the output file.
         */
        void WriteHeaderData(void) noexcept {
            // Create header data
            const auto timePoint = std::chrono::system_clock::now();
            const auto secondTimePoint = std::chrono::floor<std::chrono::seconds>(timePoint);
            const std::time_t time = std::chrono::system_clock::to_time_t(secondTimePoint);
            const std::tm* gmTime = std::gmtime(&time); //#TODO: use thread-safe alternatives gmtime_s or gmtime_r or use manual locks
            uint32_t year{};
            uint8_t month{};
            uint8_t day{};
            double timestampUTC{};
            if(gmTime){
                year = static_cast<uint32_t>(gmTime->tm_year) + 1900;
                month = static_cast<uint8_t>(gmTime->tm_mon) + 1;
                day = static_cast<uint8_t>(gmTime->tm_mday);
                const double fractionalSeconds = 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(timePoint - secondTimePoint).count());
                timestampUTC = 3600.0 * static_cast<double>(gmTime->tm_hour) + 60.0 * static_cast<double>(gmTime->tm_min) + static_cast<double>(gmTime->tm_sec) + fractionalSeconds;
            }
            constexpr char logStart[] = {'$','$'};
            const uint32_t headerSize = sizeof(year) + sizeof(month) + sizeof(day) + sizeof(timestampUTC);

            // Write header data
            m_ofs.write(&logStart[0], sizeof(logStart));
            m_ofs.write(reinterpret_cast<const char*>(&headerSize), sizeof(headerSize));
            m_ofs.write(reinterpret_cast<const char*>(&year), sizeof(year));
            m_ofs.write(reinterpret_cast<const char*>(&month), sizeof(month));
            m_ofs.write(reinterpret_cast<const char*>(&day), sizeof(day));
            m_ofs.write(reinterpret_cast<const char*>(&timestampUTC), sizeof(timestampUTC));
        }
};


} /* namespace: core */


} /* namespace: mpsv */
