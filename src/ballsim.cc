#include "ballsim.h"

BallSimulation::BallSimulation()
{
	unsigned int numberOfBalls = 20;
	for (size_t i = 0; i < numberOfBalls; i++)
	{
		balls.push_back(Ball());
	}
}

bool BallSimulation::collision_with_other(Ball* b)
{
    for (size_t i = 0; i < balls.size(); i++)
    {
        Ball* other = &balls[i];
        if (other == b)
            continue;
        if (Ball::collision_check(b, other))
            return true;
    }
    return false;
}
