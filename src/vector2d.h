#ifndef SRC_VECTOR2D_H
#define SRC_VECTOR2D_H

#include "utils.h"

struct Vector2D
{
    number x;
    number y;
};

number new_number_random (number min, number max);
int new_number_random_int (int min, int max);

Vector2D new_vector_random (number min_x, number max_x, number min_y, number max_y);
number Vector2D_length(Vector2D* vec);
void Vector2D_normalize(Vector2D* vec);
Vector2D Vector2D_add(Vector2D* vec1, Vector2D* vec2);
Vector2D Vector2D_sub(Vector2D* vec1, Vector2D* vec2);
Vector2D Vector2D_multScalar(Vector2D* vec, number scalar);
number Vector2D_scalarProduct(Vector2D* vec1, Vector2D* vec2);
number Vector2D_scalarProjectionOfVec1OntoVec2(Vector2D* vec1, Vector2D* vec2);
void Vector2D_rotate(Vector2D* vec, number degrees, bool clockWise);
void Vector2D_rotateAroundPoint(Vector2D* vec, Vector2D* point, number degrees, bool clockWise);

#endif /* SRC_VECTOR2D_H */
