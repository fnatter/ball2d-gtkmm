#include "ballsim.h"

#include <iostream>

BallSimulation::BallSimulation()
{
	max_radius = -100.0;
	unsigned int numberOfBalls = 20;
	for (size_t i = 0; i < numberOfBalls; i++)
	{
		balls.push_back(Ball());
		
		if (balls[i].radius > max_radius)
            max_radius = balls[i].radius;

	}
	
	nextEvent.type = EventType::NONE;
	nextEvent.delta_t = INFINITY;
	
	initRandomPosition();	
}

void BallSimulation::initRandomPosition()
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
				balls.resize(i-1);
				break;
			}
		} // while(conflict)
	} // for all balls
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
            //Ball_doObstacleCollision(st, b, nextEvent.obstacle, nextEvent.obsCollType);
            break;

        case EventType::POLYGONAL_OBSTACLE_COLLISION:
            //Ball_doPolygonalObstacleCollision(st, b, nextEvent.polyObstacle, nextEvent.polyObsCollType);
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
