#ifndef SRC_EVENT_H
#define SRC_EVENT_H 1

#include <vector>

#include "vector2d.h"
#include "utils.h"

// fwd declaration
class Ball;

enum class EventType { NONE = 0,
    LEFT_WALL_HIT, RIGHT_WALL_HIT, TOP_WALL_HIT, BOTTOM_WALL_HIT,
    OBSTACLE_COLLISION, POLYGONAL_OBSTACLE_COLLISION, BALL_COLLISION
};

/* collision type for Ball vs axis-aligned rectangular obstacles */
enum class ObstacleCollisionType
{
    LEFT_EDGE = 0, RIGHT_EDGE, TOP_EDGE, BOTTOM_EDGE,
    TOPLEFT_CORNER, TOPRIGHT_CORNER, BOTTOMLEFT_CORNER, BOTTOMRIGHT_CORNER,
    CENTER_INSIDE /* this shouldn't happen */
};

struct Obstacle
{
    number x1, y1, x2, y2;
};

typedef struct
{
    std::vector<Vector2D> points;
} PolygonalObstacle;

// TODO: copy ctor!
class Event 
{
public:
	static Event find_closest(Event* events, int count);

public:
    EventType type;
       
    Obstacle* obstacle;
    ObstacleCollisionType obsCollType;
    /* 0..numberPoints-1: points, 
       numberpoints..2*numberpoints-1: edges */
    int polyObsCollType; 
    PolygonalObstacle* polyObstacle;

    Ball* ball;
    Ball* ball2;

    number delta_t;
};

#endif /* SRC_EVENT_H */

