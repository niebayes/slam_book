#ifndef CLAMP_UTILS_H_
#define CLAMP_UTILS_H_

namespace std {

template <class T>
constexpr const T& clamp(const T& v, const T& lo, const T& hi);

template <class T, class Compare>
constexpr const T& clamp(const T& v, const T& lo, const T& hi, Compare comp);

#include "clamp.hpp"

}  // namespace std

#endif  // CLAMP_UTILS_H_