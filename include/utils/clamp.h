#ifndef CLAMP_UTILS_H_
#define CLAMP_UTILS_H_

template <class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi);

template <class T, class Compare>
constexpr const T& clamp(const T& v, const T& lo, const T& hi, Compare comp)

#include "clamp.hpp"

#endif  // CLAMP_UTILS_H_