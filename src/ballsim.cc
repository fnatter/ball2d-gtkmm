#include "ballsim.h"

#include <iostream>
#include <ctime>

BallSimulation::BallSimulation()
{
	// set some defaults
	numberOfBalls = 100;
	tinyMode = false;
	debianMode = false;
	cornersMode = false;
	slowStartMode = false;
	showVelocityVectors = true;
	//delay = 20000;

	// we cannot call init() here because the command line
	// options are not parsed yet!
}

void BallSimulation::init()
{
	if (cornersMode)
	{
		if (numberOfBalls < 8)
			numberOfBalls = 8;
		else if (numberOfBalls > 32)
			numberOfBalls = 32;
		else
			numberOfBalls -= numberOfBalls % 8;
	}

	
	if (debianMode || cornersMode)
	{
		slowStartMode = true;
	}
	startTime = time(nullptr);
	
	max_radius = -100.0;
	for (size_t i = 0; i < numberOfBalls; i++)
	{
		balls.emplace_back(tinyMode, debianMode, cornersMode);

		if (balls[i].radius > max_radius)
            max_radius = balls[i].radius;
	}
	
	nextEvent.type = EventType::NONE;
	nextEvent.delta_t = INFINITY;
	
	//initObstacles2();
	//polyObstacles.resize(1);
	//createStarPolygon(&polyObstacles[0], 24, 0, 0, 70, 10);
	
	if (debianMode)
	{
		initDebianPositions();
	}
	else if (cornersMode)
	{
		initCornerPositions();
	}
	else
	{
		initRandomPositions();
	}
}

void BallSimulation::initObstacles()
{
	obstacles.emplace_back(-50.0, -40.0, -40.0, 20.0);
	obstacles.emplace_back(30.0, -10.0, 50.0, 10.0);
}

void BallSimulation::initObstacles2()
{
	double obstacleSize = 10.0;
	int numCols = 5, numRows = 3;
	double xSpacing = (MAXX-MINX)/(numCols+1);
	double ySpacing = (MAXY-MINY)/(numRows+1);

	for (int i = 1; i <= numRows; i++)
	{
		for (int j = 1; j <= numCols; j++)
		{
			double x = MINX + j*xSpacing;
			double y = MINY + i*ySpacing;
			
			obstacles.emplace_back(x-obstacleSize/2, y-obstacleSize/2, x+obstacleSize/2, y+obstacleSize/2);
		}
	}
}

void BallSimulation::initCornerPositions()
{
    int numberBallsPerCorner = numberOfBalls / 8;
    int ballCounter = 0;

    // left
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MINX + SINGLE_STEP + max_radius,
                            MINY + (MAXY-MINY)/2.0,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }

    // top left
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MINX + SINGLE_STEP + max_radius,
                            MINY + SINGLE_STEP + max_radius,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }

    // top 
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MINX + (MAXX-MINX)/2.0,
                            MINY + SINGLE_STEP + max_radius,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }

    // top right
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MAXX - SINGLE_STEP - max_radius,
                            MINY + SINGLE_STEP + max_radius,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }

    // right
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MAXX - SINGLE_STEP - max_radius,
                            MINY + (MAXY-MINY)/2.0,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }

    // bottom right
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MAXX - SINGLE_STEP - max_radius,
                            MAXY - SINGLE_STEP - max_radius,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }
      
    // bottom
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MINX + (MAXX-MINX)/2.0,
                            MAXY - SINGLE_STEP - max_radius,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }

    // bottom left
    if (tryPlaceBallsInCorner(&ballCounter, 
                            MINX + SINGLE_STEP + max_radius,
                            MAXY - SINGLE_STEP - max_radius,
                            numberBallsPerCorner))
    {
        // collision - stop placing balls!
        resizeBalls(ballCounter);
        return;
    }
}

void BallSimulation::initRandomPositions()
{
    ObstacleCollisionType obsCollType;

	for (size_t i = 0; i < balls.size(); i++)
	{
		int numberConflicts = 0;

		bool conflict = true;
		while (conflict)
		{
			balls[i].randomizePosition();
			conflict = false;

			/* check for collision with obstacle(s) */
			for (size_t j = 0; j < obstacles.size(); j++)
			{
				if (balls[i].obstacle_collision_check(&obstacles[j], &obsCollType))
				{
					conflict = true;
					break;
				}
			}
			for (size_t j = 0; j <  polyObstacles.size(); j++)
			{
				if (balls[i].polygonalObstacle_collision_check(&polyObstacles[j]))
				{
					conflict = true;
					break;
				}
			}

			/* check for collision with other balls */
			for (size_t j = 0; j < i; j++)
			{
				if (Ball::collision_check(&balls[i], &balls[j])) {
					conflict = true;
					break;
				}
			}
			if (conflict && ++numberConflicts > 10000)
			{
				/* it is very likely that there's no space left for the
				   i'th ball */
				//balls.resize(i-1);
				resizeBalls(i-1);
				break;
			}
		} // while(conflict)
	} // for all balls
}

void BallSimulation::initDebianPositions()
{
	int ballCounter = 0;
	number numberOfRounds = 2.5;
	number angleIncrement = 15.0; // was: 20.0

	for (number angle = -45.0, radius = 0.0; angle <= numberOfRounds * 360.0;
		 angle += angleIncrement, radius += max_radius*0.8)
	{
		balls[ballCounter].x = cos(angle * M_PI/180.0) * radius;
		balls[ballCounter].y = sin(angle * M_PI/180.0) * radius;
		
		bool skip = false;
		for (int otherBallCounter = 0; otherBallCounter < ballCounter; otherBallCounter++)
		{
			if (Ball::collision_check(&balls[ballCounter], &balls[otherBallCounter]))
			{
				// skip this position, it would cause a collision
				skip = true;
				break;
			}
		}
		if (skip)
		{
			continue;
		}

		// no more space left?
		if (balls[ballCounter].x - balls[ballCounter].radius < MINX ||
			balls[ballCounter].x + balls[ballCounter].radius > MAXX ||
			balls[ballCounter].y - balls[ballCounter].radius < MINY ||
			balls[ballCounter].y + balls[ballCounter].radius > MAXY)
		{
			//std::cout << "Stopping after ballCounter=" << ballCounter << "\n";
			break;
		}
		
		ballCounter++;
		// all balls placed?
		if (ballCounter == numberOfBalls)
			break;
	}
	//std::cout << "Done with Debian: ballCounter=" << ballCounter << ", balls.size()=" << balls.size() << "\n";
	resizeBalls(ballCounter);
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

void BallSimulation::innerLoop()
{
	number delta_t_remaining = SINGLE_TIME_STEP;
    time_t timeDelta = time(nullptr) - startTime;
	
	if (slowStartMode && timeDelta <= 15)
	{
		delta_t_remaining *= 0.05;
	}
	
    while (move(&delta_t_remaining))
    {
		//std::cout << "in loop: delta_t_remaining=" << delta_t_remaining << std::endl;
    }
}

bool BallSimulation::move(number* delta_t)
{
    int i, j, nBalls;
    Ball* cball;
    number this_delta_t;
    time_t timeDelta, currentTime;

    /* return false;*/

#ifdef LOGGING
	std::cout << "\n\nBalls_move(" << *delta_t << ")" << std::endl;
#endif

    balls = balls;
    nBalls = balls.size();

    currentTime = time(NULL);
    timeDelta = currentTime - startTime;

    if (zombies && lastZombieTime != currentTime &&
        timeDelta > 0 && timeDelta % 10 == 0)
    {
        int angle = new_number_random_int(-1,7)*45;
        
        for (i = 0; i < nBalls; i++)
        {
            if (angle == -45)
				balls[i].setDirectionForTarget(0.0, 0.0);
            else
            {
                balls[i].dx = cos(angle/180.0*M_PI);
                balls[i].dy = sin(angle/180.0*M_PI);
            }
        }
        lastZombieTime = currentTime; 
        
        /* make sure that events are re-calculated */
        nextEvent.delta_t = INFINITY;
        nextEvent.type = EventType::NONE;

        /* make sure that all collisions are possible again */
        for (i = 0; i < nBalls; i++)
        {
            balls[i].lastEvent.type = EventType::NONE;
        }
    }

    if (nextEvent.delta_t < *delta_t)
    {
        this_delta_t = nextEvent.delta_t;
        nextEvent.delta_t = 0.0;
        /* set remaining delta_t */
        *delta_t -= this_delta_t;
    }
    else
    {
        this_delta_t = *delta_t;
        nextEvent.delta_t -= this_delta_t;
        /* set remaining delta_t */
        *delta_t = 0.0;
    }

#ifdef LOGGING
	std::cout << "this_delta_t=" << this_delta_t << ", *delta_t=" << *delta_t << std::endl;
#endif

    /* move balls for delta_t timesteps */
    for (i = 0; i < nBalls; i++)
    {
        cball = &balls[i];
        cball->x += cball->dx * cball->velocity * this_delta_t;
        cball->y += cball->dy * cball->velocity * this_delta_t;
    }

    /* execute nextEvent if it's time */
    if (nextEvent.delta_t == 0.0)
    {
        Ball* b = nextEvent.ball;
        Ball* b2 = nextEvent.ball2;
      
        switch (nextEvent.type)
        {
        case EventType::LEFT_WALL_HIT:
        case EventType::RIGHT_WALL_HIT:
            b->dx = -b->dx;
            b->lastEvent = nextEvent;
            break;
          
        case EventType::TOP_WALL_HIT:
        case EventType::BOTTOM_WALL_HIT:
            b->dy = -b->dy;
            b->lastEvent = nextEvent;
            break;
          
        case EventType::BALL_COLLISION:
            Ball::doCollision(b, b2);
            break;

        case EventType::OBSTACLE_COLLISION:
            b->doObstacleCollision(nextEvent.obstacle, nextEvent.obsCollType);
            break;

        case EventType::POLYGONAL_OBSTACLE_COLLISION:
            b->doPolygonalObstacleCollision(nextEvent.polyObstacle, nextEvent.polyObsCollType);
            break;

        case EventType::NONE:
			std::cout << "nextEvent.type == NONE!\n";
            abort();
            break;
        }
    }

    if (nextEvent.delta_t == 0.0 || nextEvent.type == EventType::NONE)
    {
        /* find the next closest event */
        nextEvent.delta_t = INFINITY;
        nextEvent.type = EventType::NONE;

        /* wall or obstacle collision */
        for (i = 0; i < nBalls; i++)
        {
            Event ev;
            cball = &balls[i];

            /* cball-wall collision? */
            cball->findWallIntersection(&ev);
       
            /* do not collide with wall twice with no event in between! */
            if (cball->lastEvent.type == ev.type)
                continue;
       
            if (ev.delta_t < nextEvent.delta_t)
            {
                nextEvent = ev;
            }

#ifdef EVENT_LOGGING
            std::cout << "Checking for path intersections with obstacles...";
#endif
            for (j = 0; j < obstacles.size(); j++)
            {
                /* do not re-collide with same obstacle with no event in
                   between! */
                if (cball->lastEvent.type == EventType::OBSTACLE_COLLISION &&
                    cball->lastEvent.obstacle == &obstacles[j])
                    continue;

                /* cball-obstacle_j collision? */
                cball->findObstacleIntersection(&obstacles[j], &ev);
                if (ev.delta_t < nextEvent.delta_t)
                {
                    nextEvent = ev;
                }
            }
            
#ifdef EVENT_LOGGING
            std::cout << "Checking for path intersections with poly obstacles...";
#endif
            for (j = 0; j < polyObstacles.size(); j++)
            {
                cball->findPolygonalObstaclePathIntersection(&polyObstacles[j], &ev);

                if (ev.delta_t < nextEvent.delta_t)
                {
                    nextEvent = ev;
                }
            }
        }

#ifdef EVENT_LOGGING
        std::cout << "found best wall intersection: " << nextEvent << std::endl;
#endif
  
        /* ball collision */
        for (i = 0; i < nBalls; i++)
        {
            Ball* b1 = &balls[i];

#ifdef EVENT_LOGGING
			std::cout << "Ball " << i << ": last Event:\n\t"
			          << b1->lastEvent << std::endl;
#endif

            for (j = 0; j < i; j++)
            {
                Ball* b2 = &balls[j];
                number delta_t;

                if (b1->lastEvent.type == EventType::BALL_COLLISION &&
                    b1->lastEvent.ball2 == b2 &&
                    b2->lastEvent.type == EventType::BALL_COLLISION &&
                    b2->lastEvent.ball2 == b1)
                {
#ifdef EVENT_LOGGING
                    std::cout << "skipping collision(" i << ", " << j << ")\n";
#endif
                    continue;
                }
          
                delta_t = Ball::findPathIntersection(b1, b2);
#ifdef EVENT_LOGGING
                /*
                  printf("found ball collision(%x,%x) @%f\n",
                  (unsigned int)b1, (unsigned int)b2, delta_t);
                */
#endif
          
                if (delta_t < nextEvent.delta_t)
                {
                    nextEvent.type = EventType::BALL_COLLISION;
                    nextEvent.delta_t = delta_t;
                    nextEvent.ball = b1;
                    nextEvent.ball2 = b2;
#ifdef EVENT_LOGGING
					std::cout << "found better ball collision:\n"
					          << nextEvent << std::endl;
#endif
                }
            }
        }
#ifdef EVENT_LOGGING
        std::cout << "best event:\n" << nextEvent << std::endl;
#endif
    }

    return *delta_t > 0.0;
}

void BallSimulation::createStarPolygon(PolygonalObstacle* polyObs, int numberPoints,
                  number x, number y, number radius1, number radius2)
{
    polyObs->points.resize(numberPoints);
	number radPerPoint = 2*M_PI / polyObs->points.size();
    number angle = new_number_random(0.0, 2*M_PI);
    for (size_t i = 0; i < polyObs->points.size(); i++)
    {
        polyObs->points[i].x = x +
            cos(angle) * (i % 2 == 0 ? radius1 : radius2);
        polyObs->points[i].y = y +
            sin(angle) * (i % 2 == 0 ? radius1 : radius2);
        angle += radPerPoint;
    }
}

void BallSimulation::resizeBalls(int newNumberOfBalls)
{
	if (balls.size() < newNumberOfBalls)
	{
		throw std::runtime_error("Cannot extend initial balls.size()!");
	}
	while (balls.size() > newNumberOfBalls)
	{
		balls.pop_back();
	}
	// this is redundant...
	numberOfBalls = balls.size();
}

// place up to numberBallsForThisCorner number balls in a corner 
bool BallSimulation::tryPlaceBallsInCorner(int* ballCounter, number cornerX, number cornerY, int numberBallsForThisCorner)
{
	Vector2D pos;
	// starting point in corner
	pos.x = cornerX;
	pos.y = cornerY;

	for (int i = 0; i < numberBallsForThisCorner; ++i)
	{
		balls[*ballCounter].x = pos.x;
		balls[*ballCounter].y = pos.y;
		balls[*ballCounter].setDirectionForTarget(0.0, 0.0);
		
		for (int j =0; j < *ballCounter; ++j)
		{
			if (Ball::collision_check(&balls[*ballCounter], &balls[j]))
			{
				return true;
			}
		}
		  
		pos.x += balls[*ballCounter].dx * 2*max_radius;
		pos.y += balls[*ballCounter].dy * 2*max_radius;
		(*ballCounter)++;
	}
	return false;
}
