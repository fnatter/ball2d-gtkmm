#include "ballsim.h"

BallSimulation::BallSimulation()
{
	unsigned int numberOfBalls = 20;
	for (size_t i = 0; i < numberOfBalls; i++)
	{
		balls.push_back(Ball());
	}
}
