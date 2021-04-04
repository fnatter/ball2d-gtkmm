#ifndef SRC_BALLSIM_H
#define SRC_BALLSIM_H 1

#include "event.h"
#include "ball.h"

#include <ctime>

struct FutureCollision : public Event
{
	static constexpr int durationMs = 1000;
	static constexpr double r = 0.0;
	static constexpr double g = 0.0;
	static constexpr double b = 1.0;

	long startTimeMs;

	FutureCollision(Event evt)
		: Event(evt), startTimeMs(time_microseconds() / 1000)
	{

	}

};

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

	void removeFadedOutFutureCollisionMarkers();

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

    std::vector<FutureCollision> futureCollisions;
};

#endif /* SRC_BALLSIM_H */
