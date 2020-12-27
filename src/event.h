#ifndef SRC_EVENT_H
#define SRC_EVENT_H 1

#include <vector>
#include <ostream>

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
	Obstacle(number _x1, number _y1, number _x2, number _y2)
	{
		x1 = _x1;
		y1 = _y1;
		x2 = _x2;
		y2 = _y2;
	}
	
	void print(std::ostream& out) const
	{
		out << "Obstacle[" << x1 << ", " << y1 << ", " << x2 << ", " << y2 << "]";
	}
	
    number x1, y1, x2, y2;
};

inline std::ostream& operator<<(std::ostream& out, const Obstacle& obstacle)
{
	obstacle.print(out);
	return out;
}

typedef struct
{
    std::vector<Vector2D> points;
} PolygonalObstacle;

// TODO: copy ctor!
class Event 
{
public:
	Event() { }

	static Event find_closest(Event* events, int count);
	
	void print(std::ostream& out) const;

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

inline std::ostream& operator<<(std::ostream& out, const Event& event)
{
	event.print(out);
	return out;
}

#endif /* SRC_EVENT_H */

