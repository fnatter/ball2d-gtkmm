#ifndef SRC_MATHS_H
#define SRC_MATHS_H 1

#include <type_traits>

typedef double number;

inline number pow2(number x)
{
    return x*x;
}

const number MINX = -100.0;
const number MAXX = 100.0;
const number MINY = -100.0;
const number MAXY = 100.0;
//#define ANGLES_ALL (NULL)

/* was: MAX_RADIUS = 6.0, MAX_MASS=2.0, MIN_MASS=0.1 */
const double MIN_RADIUS = 1.0, MAX_RADIUS = 10.0; 
const double MIN_VELOCITY = 0.0, MAX_VELOCITY = 2.0, MIN_MASS = 0.5, MAX_MASS = 2.0,
    SINGLE_TIME_STEP = 0.7, SINGLE_STEP = 0.05, EPS=1.0e-4;

const double MAX_MOMENTUM = MAX_VELOCITY * MAX_MASS;

// https://stackoverflow.com/questions/11421432/how-can-i-output-the-value-of-an-enum-class-in-c11:
template <typename Enumeration>
auto as_integer(Enumeration const value)
    -> typename std::underlying_type<Enumeration>::type
{
    return static_cast<typename std::underlying_type<Enumeration>::type>(value);
}

#endif /* SRC_MATHS_H */
