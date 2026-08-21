#pragma once


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// External Libraries
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* Default C/C++ headers */
#include <iostream>
#include <chrono>
#include <array>
#include <vector>
#include <string>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <atomic>
#include <thread>
#include <cstdarg>
#include <csignal>
#include <cstring>
#include <cmath>


/* OS depending */
// Windows System (MinGW)
#ifdef _WIN32
#include <winsock2.h>
#include <Ws2tcpip.h>
#include <Iphlpapi.h>
#include <sys/stat.h>
// Unix System
#elif __linux__
#include <execinfo.h>
#include <sys/utsname.h>
#include <sys/timerfd.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <sys/stat.h>
#else
// Other
#endif


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Version Settings
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#if __linux__
inline constexpr std::string_view strOS{"Linux"};
#elif __FreeBSD__
inline constexpr std::string_view strOS{"FreeBSD"};
#elif __ANDROID__
inline constexpr std::string_view strOS{"Android"};
#elif __APPLE__
inline constexpr std::string_view strOS{"macOS"};
#elif _WIN32
inline constexpr std::string_view strOS{"Windows"};
#else
inline constexpr std::string_view strOS{"unknown"};
#endif
inline constexpr std::string_view strVersion{"1.0.0"};
inline constexpr std::string_view strCompilerVersion{__VERSION__};
inline constexpr std::string_view strBuilt{__DATE__ " " __TIME__};


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Macros For Thread-safe Console Prints
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
extern void __mpsv_print(const char* format, ...);
extern void __mpsv_print_time(const char* format, ...);
extern void __mpsv_print_verbose(const char prefix, const char* file, const int line, const char* func, const char* format, ...);


#define Print(...) __mpsv_print(__VA_ARGS__)
#define PrintT(...) __mpsv_print_time(__VA_ARGS__)
#define PrintW(...) __mpsv_print_verbose('W', __FILE__, __LINE__, __func__, __VA_ARGS__)
#define PrintE(...) __mpsv_print_verbose('E', __FILE__, __LINE__, __func__, __VA_ARGS__)

