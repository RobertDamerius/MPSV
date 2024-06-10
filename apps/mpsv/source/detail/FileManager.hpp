#pragma once


#include <Common.hpp>


namespace FileManager {


/**
 * @brief Get the absolute path to the application directory.
 * @return Absolute path to the application directory.
 */
inline std::filesystem::path GetApplicationPath(void){
    #ifdef _WIN32
    char* buffer = new char[65536];
    DWORD len = GetModuleFileNameA(NULL, &buffer[0], 65536);
    std::string str(buffer, len);
    delete[] buffer;
    #else
    std::string str("/proc/self/exe");
    #endif
    std::filesystem::path applicationPath;
    try {
        applicationPath = std::filesystem::canonical(str);
        applicationPath.remove_filename();
    }
    catch(...){ }
    return applicationPath;
}


/**
 * @brief Generate the filename of the protocol file based on the current system time.
 * @return The filename of the format "YYYYMMDD_hhmmssmmm.txt".
 */
inline std::string GenerateProtocolFileName(void){
    auto timePoint = std::chrono::system_clock::now();
    std::time_t systemTime = std::chrono::system_clock::to_time_t(timePoint);
    std::tm* gmTime = std::gmtime(&systemTime);
    uint32_t utcYear = static_cast<uint32_t>(gmTime->tm_year);
    uint32_t utcMonth = static_cast<uint32_t>(gmTime->tm_mon);
    uint32_t utcMDay = static_cast<uint32_t>(gmTime->tm_mday);
    uint32_t utcHour = static_cast<uint32_t>(gmTime->tm_hour);
    uint32_t utcMinute = static_cast<uint32_t>(gmTime->tm_min);
    uint32_t utcSecond = static_cast<uint32_t>(gmTime->tm_sec);
    auto duration = timePoint.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    duration -= seconds;
    uint32_t utcNanoseconds = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(duration).count());
    char buffer[64];
    sprintf(buffer, "%u%02u%02u_%02u%02u%02u%03u", 1900 + utcYear, 1 + utcMonth, utcMDay, utcHour, utcMinute, utcSecond, utcNanoseconds / 1000000);
    return std::string(buffer) + ".txt";
}


} /* namespace: FileManager */


enum EnumFileName {
    FILENAME_MPSV_PROTOCOL,
    FILENAME_MPSV_CONFIGURATION,
    FILENAME_DIRECTORY_PROTOCOL
};


/**
 * @brief Get the absolute filename for a specific file.
 * @param[in] enumFileName Enumeration file that specifies the file for which to obtain the absolute filename.
 * @return Absolute filename.
 */
inline std::string FileName(EnumFileName enumFileName){
    std::filesystem::path applicationPath = FileManager::GetApplicationPath();
    std::filesystem::path protocolPath = applicationPath / "protocol";
    std::filesystem::path result;
    switch(enumFileName){
        case FILENAME_MPSV_PROTOCOL:        result = protocolPath / FileManager::GenerateProtocolFileName();   break;
        case FILENAME_MPSV_CONFIGURATION:   result = applicationPath / "MPSV.json";                            break;
        case FILENAME_DIRECTORY_PROTOCOL:   result = protocolPath;                                             break;
    }
    return result.string();
}

