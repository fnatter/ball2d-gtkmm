#ifndef SRC_BALLSIM_H
#define SRC_BALLSIM_H 1

#include "event.h"
#include "ball.h"

#include <ctime>

struct BallSimulation
{
public:
	BallSimulation();
	
	bool collision_with_other(Ball* b);
	
public:
    long iteration_number;
    std::time_t startTime;
    std::time_t lastZombieTime;

    std::vector<Ball> balls;
    int nObstacles;
    std::vector<Obstacle> obstacles;
    int nPolyObstacles;
    std::vector<PolygonalObstacle> polyObstacles;
  
    bool dbuf;
    int delay;
    int count;
    int ncolors;
    bool showVelocityVectors;
    int* startAngles;
    int numStartAngles;
    bool tiny;
    bool startGrid;
    bool zombies;
    bool corners;
    bool debian;
    bool slowStart;
    number max_radius;
    bool showFutureCollisions;
    Event nextEvent;

};

#endif /* SRC_BALLSIM_H */
