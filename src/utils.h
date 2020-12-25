#ifndef SRC_MATHS_H
#define SRC_MATHS_H 1

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

#endif /* SRC_MATHS_H */
