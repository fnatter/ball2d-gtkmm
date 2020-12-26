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
	bool Obstacle_collision_check(Obstacle* obstacle, ObstacleCollisionType* type) const;
	bool pointInside(number x, number y) const;
	void findPolygonalObstaclePathIntersection(PolygonalObstacle* polyObs, Event* ev) const;
	void findWallIntersection(Event* ev) const;
	number findPathPointIntersection(number cx, number cy) const;
	void findObstacleIntersection(Obstacle* ob, Event* ev) const;
	
	/*
		This was solved via maxima (GPL) with the following command:
		solve([(px2+t*vx2 - px1-t*vx1)^2 + (py2 + t*vy2 - py1-t*vy1)^2 =
		(r1+r2)^2], t);
	*/
	static number findPathIntersection(Ball* b1, Ball* b2);

	static bool collision_check(Ball* b1, Ball* b2);

	
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