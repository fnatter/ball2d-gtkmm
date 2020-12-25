#ifndef SRC_BALL_H
#define SRC_BALL_H 1

#include "event.h"
#include "utils.h"
#include <gtkmm.h>

// TODO: copy ctor!
class Ball
{
public:

	Ball();
	
	void randomizePosition();
	
	/*
		find out at which time the ball will first touch (cx,cy)
		created with maxima using this command:
		solve([(px+t*vx - cx)^2 + (py+t*vy -cy)^2 = r^2], t);
	*/
	number findPathPointIntersection(number cx, number cy) const;
	bool Obstacle_collision_check(Obstacle* obstacle, ObstacleCollisionType* type) const;
	bool pointInside(number x, number y) const;
	
	void draw(const Cairo::RefPtr<Cairo::Context>& cr, const int width, const int height) const;

public:
    number x,y;
    number dx,dy;
    number mass;
    number radius;
    unsigned long color;
    double velocity;
	Event lastEvent;
};


#endif /* SRC_BALL_H */
