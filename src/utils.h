#ifndef UTILS_H_
#define UTILS_H_

#define MATH_MIN(a, b) ((a) > (b) ? (b) : (a))
#define MATH_MAX(a, b) ((a) > (b) ? (a) : (b))
#define MATH_CLAMP(v, low, high) MATH_MIN(MATH_MAX(v, low), high)

#endif // UTILS_H_
