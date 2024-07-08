#pragma once


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// External Libraries
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/* Default C/C++ headers */
#include <iostream>
#include <cstdint>
#include <fstream>
#include <string>
#include <array>
#include <vector>
#include <mutex>
#include <functional>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <random>
#include <cmath>
#include <limits>
#include <type_traits>
#include <tuple>
#include <thread>
#include <condition_variable>


// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// MPSV Globals
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
constexpr uint32_t MPSV_VERSION = 20240708;   // Version number of the MPSV library.


/* Default namespace for MPSV */
namespace mpsv {


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

