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

	void initDebianPositions();
	void initCornerPositions();
	void initRandomPositions();
	void initGridPositions();
	
	void initObstacles();
	void initObstacles2();
	
	void innerLoop();

	bool move(number* delta_t);
	bool collision_with_other(Ball* b);
	
	void createStarPolygon(PolygonalObstacle* polyObs, int numberPoints,
                  number x, number y, number radius1, number radius2);
                  
public:
	bool tryPlaceBallsInCorner(int* ballCounter, number cornerX, number cornerY, int numberBallsForThisCorner);
	void resizeBalls(int newNumberOfBalls);

    long iteration_number;
    std::time_t startTime;
    //std::time_t lastZombieTime;
    number max_radius;

    std::vector<Ball> balls;
    std::vector<Obstacle> obstacles;
    std::vector<PolygonalObstacle> polyObstacles;

    int numberOfBalls;
    bool tinyMode;
    bool debianMode;
    bool cornersMode;
    bool slowStartMode;
    bool startGridMode;
    bool showVelocityVectors;
    bool showFutureCollisions;

    int speed; // [0;10]
    Glib::ustring startAnglesOpt;
    std::vector<int> startAngles;

    // TODO: implement zombies mode
//    bool zombies;
    Event nextEvent;
};

#endif /* SRC_BALLSIM_H */
