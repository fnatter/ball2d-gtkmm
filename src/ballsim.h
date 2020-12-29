#ifndef SRC_BALLSIM_H
#define SRC_BALLSIM_H 1

#include "event.h"
#include "ball.h"

#include <ctime>

struct BallSimulation
{
public:
	BallSimulation();
	
	void init();
	
	void initObstacles();
	void initObstacles2();
	void initRandomPositions();
	
	bool move(number* delta_t);
	bool collision_with_other(Ball* b);
	
	void createStarPolygon(PolygonalObstacle* polyObs, int numberPoints,
                  number x, number y, number radius1, number radius2);
	
public:
    long iteration_number;
    std::time_t startTime;
    std::time_t lastZombieTime;

    std::vector<Ball> balls;
    std::vector<Obstacle> obstacles;
    std::vector<PolygonalObstacle> polyObstacles;

    int numberOfBalls;
    bool tinyMode;
    bool debianMode;
    bool cornersMode;
      
    bool dbuf;
    int delay;
    int ncolors;
    bool showVelocityVectors;
    int* startAngles;
    int numStartAngles;

    bool startGrid;
    bool zombies;
    bool slowStart;
    number max_radius;
    bool showFutureCollisions;
    Event nextEvent;

};

#endif /* SRC_BALLSIM_H */
