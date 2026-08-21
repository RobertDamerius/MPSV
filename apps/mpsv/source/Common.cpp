#include <Common.hpp>


std::mutex __mpsv_mtx_print;
static std::chrono::time_point<std::chrono::steady_clock> __mpsv_steady_clock = std::chrono::steady_clock::now();


void __mpsv_print(const char* format, ...){
    const std::lock_guard<std::mutex> lk(__mpsv_mtx_print);
    va_list argptr;
    va_start(argptr, format);
    vfprintf(stderr, format, argptr);
    va_end(argptr);
}

void __mpsv_print_time(const char* format, ...){
    double t = 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - __mpsv_steady_clock).count());
    {
        const std::lock_guard<std::mutex> lk(__mpsv_mtx_print);
        fprintf(stderr,"  [t=%lf] ", t);
        va_list argptr;
        va_start(argptr, format);
        vfprintf(stderr, format, argptr);
        va_end(argptr);
    }
}

void __mpsv_print_verbose(const char prefix, const char* file, const int line, const char* func, const char* format, ...){
    double t = 1e-9 * static_cast<double>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - __mpsv_steady_clock).count());
    {
        const std::lock_guard<std::mutex> lk(__mpsv_mtx_print);
        fprintf(stderr,"%c [t=%lf] %s:%d in %s(): ", prefix, t, file, line, func);
        va_list argptr;
        va_start(argptr, format);
        vfprintf(stderr, format, argptr);
        va_end(argptr);
    }
}

