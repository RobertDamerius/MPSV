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
extern const std::string strOS;
extern const std::string strVersion;
extern const std::string strCompilerVersion;
extern const std::string strBuilt;


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Macros For Thread-safe Console Prints
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
extern std::mutex __mpsv_mtx_print;


extern void __mpsv_print(const char* format, ...);
extern void __mpsv_print_time(const char* format, ...);
extern void __mpsv_print_verbose(const char prefix, const char* file, const int line, const char* func, const char* format, ...);


#define Print(...) __mpsv_print(__VA_ARGS__)
#define PrintT(...) __mpsv_print_time(__VA_ARGS__)
#define PrintW(...) __mpsv_print_verbose('W', __FILE__, __LINE__, __func__, __VA_ARGS__)
#define PrintE(...) __mpsv_print_verbose('E', __FILE__, __LINE__, __func__, __VA_ARGS__)

