#ifndef SRC_BALL_H
#define SRC_BALL_H 1

#include "event.h"
#include "utils.h"
#include <gtkmm.h>

#include <ostream>

// TODO: copy ctor!
class Ball
{
public:

	Ball(bool tinyMode, bool debianMode, bool cornersMode);
	
	void randomizePosition();
	void print(std::ostream& out) const;
	
	/*
		find out at which time the ball will first touch (cx,cy)
		created with maxima using this command:
		solve([(px+t*vx - cx)^2 + (py+t*vy -cy)^2 = r^2], t);
	*/
	bool Obstacle_collision_check(Obstacle* obstacle, ObstacleCollisionType* type) const;
	bool pointInside(number x, number y) const;
	void findPolygonalObstaclePathIntersection(PolygonalObstacle* polyObs, Event* ev);
	void findWallIntersection(Event* ev);
	number findPathPointIntersection(number cx, number cy) const;
	void findObstacleIntersection(Obstacle* ob, Event* ev);
	
	/*
		This was solved via maxima (GPL) with the following command:
		solve([(px2+t*vx2 - px1-t*vx1)^2 + (py2 + t*vy2 - py1-t*vy1)^2 =
		(r1+r2)^2], t);
	*/
	static number findPathIntersection(Ball* b1, Ball* b2);
	static bool collision_check(Ball* b1, Ball* b2);
	static bool pointInPolygon(number px, number py, PolygonalObstacle* polyObs);

	void setDirectionForTarget(number tx, number ty);
	bool boundedEdge_collision_check(Vector2D P1, Vector2D P2) const;
	bool polygonalObstacle_collision_check(PolygonalObstacle* polyObs) const;
	bool obstacle_collision_check(Obstacle* obstacle, ObstacleCollisionType* type) const;
	
	void rotate(number degrees, bool clockWise);
	
	static bool doCollision(Ball* b1, Ball* b2);
	bool doObstacleCollision(Obstacle* obstacle, ObstacleCollisionType obsCollType);
	bool doPolygonalObstacleCollision(PolygonalObstacle* polyObs, int polyObsCollType);
	
public:
    number x,y;
    number dx,dy;
    number mass;
    number radius;
    number r, g, b, a;
    double velocity;
	Event lastEvent;
};

inline std::ostream& operator<<(std::ostream& out, const Ball& ball)
{
	ball.print(out);
	return out;
}

#endif /* SRC_BALL_H */
