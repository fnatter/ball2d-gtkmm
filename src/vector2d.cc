#include "vector2d.h"

#include <cmath>

using namespace std;

number new_number_random (number min, number max) {
    return ( (max-min)*((number)random()/(number)RAND_MAX) ) + min;
}

int new_number_random_int (int min, int max) {
    return (int)( (max-min+1)*((number)random()/(number)RAND_MAX) ) + min;
}

Vector2D new_vector_random (number min_x, number max_x,
                   number min_y, number max_y) {
    Vector2D ret;
    ret.x = new_number_random (min_x, max_x);
    ret.y = new_number_random (min_y, max_y);
    return ret;
}

number Vector2D_length(Vector2D* vec)
{
    return sqrt(vec->x*vec->x + vec->y*vec->y);
}

void Vector2D_normalize(Vector2D* vec)
{
    number len = Vector2D_length(vec);
    vec->x /= len;
    vec->y /= len;
}

/*
  static void
  Vector2D_negate(Vector2D* vec)
  {
  vec->x = -vec->x;
  vec->y = -vec->y;
  }
*/

Vector2D Vector2D_add(Vector2D* vec1, Vector2D* vec2)
{
    Vector2D sum;
    sum.x = vec1->x + vec2->x;
    sum.y = vec1->y + vec2->y;
    return sum;
}

Vector2D Vector2D_sub(Vector2D* vec1, Vector2D* vec2)
{
    Vector2D diff;
    diff.x = vec1->x - vec2->x;
    diff.y = vec1->y - vec2->y;
    return diff;
}

Vector2D Vector2D_multScalar(Vector2D* vec, number scalar)
{
    Vector2D ret;
    ret.x = vec->x * scalar;
    ret.y = vec->y * scalar;
    return ret;
}

number Vector2D_scalarProduct(Vector2D* vec1, Vector2D* vec2)
{
    return vec1->x * vec2->x + vec1->y * vec2->y;
}

number Vector2D_scalarProjectionOfVec1OntoVec2(Vector2D* vec1, Vector2D* vec2)
{
    return Vector2D_scalarProduct(vec1, vec2) / Vector2D_length(vec2);
}

void Vector2D_rotate(Vector2D* vec, number degrees, bool clockWise)
{
    number x, y, angle = degrees * M_PI/180.0,
        cosAngle, sinAngle;

    /* see: http://en.wikipedia.org/wiki/Cartesian_coordinate_system#Rotation 
       (CW/CCW is reversed in the article, because it assumes that top
       means high y value (which is not true for ball2d))
    */
    /* also: cos(angle) = cos(-angle) = sin(angle) = -sin(-angle) */
    cosAngle = cos(angle);
    sinAngle = sin(angle);
    if (clockWise)
    {
        x = vec->x*cosAngle - vec->y*sinAngle;
        y = vec->x*sinAngle + vec->y*cosAngle;
    }
    else
    {
        x = vec->x*cosAngle  + vec->y*sinAngle;
        y = -vec->x*sinAngle + vec->y*cosAngle;
    }

    vec->x = x;
    vec->y = y;
}

void Vector2D_rotateAroundPoint(Vector2D* vec, Vector2D* point,
                           number degrees, bool clockWise)
{
    *vec = Vector2D_sub(vec, point);
    Vector2D_rotate(vec, degrees, clockWise);
    *vec = Vector2D_add(vec, point);
}
