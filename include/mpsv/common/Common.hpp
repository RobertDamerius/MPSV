#pragma once


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// External Libraries
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
#include <iostream>
#include <cstdint>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <functional>
#include <ranges>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <random>
#include <cmath>
#include <limits>
#include <type_traits>
#include <tuple>
#include <thread>
#include <pthread.h>
#include <string_view>
#if defined(_WIN32)
#include <mutex>
#include <condition_variable>
#elif defined(__linux__)
#include <semaphore.h>
#endif


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MPSV Globals
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* Default namespace for MPSV */
namespace mpsv {


// MPSV version number
inline constexpr std::string_view version{"2.1.0"};


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// Stuff for data type checking
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* Types that must be at least 2-dimensional arrays */
template <typename C> struct is_vec2 : std::false_type {};
template <size_t N> struct is_vec2<std::array<double,N>> : std::true_type { static_assert(N > 1, "Array must contain at least 2 elements!"); };

/* Types that must be at least 3-dimensional arrays */
template <typename C> struct is_vec3 : std::false_type {};
template <size_t N> struct is_vec3<std::array<double,N>> : std::true_type { static_assert(N > 2, "Array must contain at least 3 elements!"); };

/* Types that must be at least 6-dimensional arrays */
template <typename C> struct is_vec6 : std::false_type {};
template <size_t N> struct is_vec6<std::array<double,N>> : std::true_type { static_assert(N > 5, "Array must contain at least 6 elements!"); };

/* Types that must be at least 9-dimensional arrays */
template <typename C> struct is_vec9 : std::false_type {};
template <size_t N> struct is_vec9<std::array<double,N>> : std::true_type { static_assert(N > 8, "Array must contain at least 9 elements!"); };


} /* namespace: mpsv */
