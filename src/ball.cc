#include <cmath>

#include "ball.h"

using namespace std;

Ball::Ball()
{ 
    int angle;
    
    bool tinyMode = false;
    bool debianMode = false;
    bool cornersMode = false;

    if (tinyMode) /* tiny mode: 95% small balls */
    {
        int tinyProb = new_number_random_int(1, 100);
        if (tinyProb < 95)
            this->mass = new_number_random(MIN_MASS, MIN_MASS + 0.1 * (MAX_MASS-MIN_MASS));
        else /* insert a few large balls */
            this->mass = new_number_random(MIN_MASS + 0.9*(MAX_MASS-MIN_MASS), MAX_MASS);
    }
    else if (debianMode)
    {
        /* insert only small balls in "debian mode" */
        this->mass = new_number_random(MIN_MASS, MIN_MASS + 0.1 * (MAX_MASS-MIN_MASS));
    }
    else
    {
        this->mass = new_number_random(MIN_MASS, MAX_MASS);
    }

    this->radius = ((MAX_RADIUS - MIN_RADIUS) * (this->mass - MIN_MASS) / MAX_MASS) + MIN_RADIUS;
    /* TODO: cache mapped radius?
       map_length(wld_radius, &radius); */

    //if (st->startAngles == ANGLES_ALL)
    {
        double angle = new_number_random(0.0, 360.0)*M_PI/180.0;
        this->dx = cos(angle);
        this->dy = sin(angle);
    }
    /*
    else
    {
        int whichAngle;

        whichAngle = new_number_random_int(0, st->numStartAngles - 1);
        // printf("whichAngle=%d\n", whichAngle);
        angle = st->startAngles[whichAngle]; 
        this->dx = cos(M_PI/180.0 * angle);
        this->dy = sin(M_PI/180.0 * angle);
    }
    */

    this->velocity = new_number_random(MIN_VELOCITY, MAX_VELOCITY);
    if (debianMode || cornersMode)
    {
        this->velocity = MIN_VELOCITY + (MAX_VELOCITY-MIN_VELOCITY)/2.0;
    }

    this->lastEvent.type = EventType::NONE;
    this->lastEvent.obstacle = nullptr;
    this->lastEvent.ball = nullptr;
    this->lastEvent.ball2 = nullptr;
    
    randomizePosition();
}

void Ball::randomizePosition()
{
    Vector2D position;
    position = new_vector_random (MINX + this->radius, MAXX - this->radius,
                                  MINY + this->radius, MAXY - this->radius);
    this->x = position.x;
    this->y = position.y;
}

number Ball::findPathPointIntersection(number cx, number cy) const
{
    number r = this->radius, px = this->x, py = this->y,
         vx = this->dx * this->velocity, vy = this->dy * this->velocity;
    
    number sqrtArg, sqrtValue, t1, t2;

    sqrtArg = (pow2(r)-pow2(px)+2*cx*px-pow2(cx))*pow2(vy)+((2*px-2*cx)*py-2*cy*px+2*cx*cy)*vx*vy+(pow2(r)-pow2(py)+2*cy*py-pow2(cy))*pow2(vx);
  
    if (sqrtArg < 0)
        return INFINITY;
  
    sqrtValue = sqrt(sqrtArg);
  
    t1 =-(sqrtValue +(py-cy)*vy+(px-cx)*vx)/(pow2(vy)+pow2(vx));
  
    t2 =(sqrtValue  +(cy-py)*vy+(cx-px)*vx)/(pow2(vy)+pow2(vx));

    if (t1 < 0)
        return (t2 < 0) ? INFINITY : t2;
    else if (t2 < 0)
        return t1;
    else
        return fmin(t1, t2);
}

bool Ball::Obstacle_collision_check(Obstacle* obstacle, ObstacleCollisionType* type) const
{
    /* 1. center of ball is inside rectangle */
    if (this->x >= obstacle->x1 && this->x <= obstacle->x2 &&
        this->y >= obstacle->y1 && this->y <= obstacle->y2)
    {
        *type = ObstacleCollisionType::CENTER_INSIDE;
        return true;
    }

    /* 2. ball intersects with edge of rectangle (in simple way) */
    /* 2.1 left edge */
    if (this->x < obstacle->x1 &&
        this->y >= obstacle->y1 && this->y <= obstacle->y2 &&
        this->x + this->radius >= obstacle->x1)
    {
        *type = ObstacleCollisionType::LEFT_EDGE;
        return true;
    }
    /* 2.2 right edge */
    if (this->x > obstacle->x2 &&
        this->y >= obstacle->y1 && this->y <= obstacle->y2 &&
        this->x - this->radius <= obstacle->x2)
    {
        *type = ObstacleCollisionType::RIGHT_EDGE;
        return true;
    }
    /* 2.3 top edge */
    if (this->y < obstacle->y1 &&
        this->x >= obstacle->x1 && this->x <= obstacle->x2 &&
        this->y + this->radius >= obstacle->y1)
    {
        *type = ObstacleCollisionType::TOP_EDGE;
        return true;
    }
    /* 2.4 bottom edge */
    if (this->y > obstacle->y2 &&
        this->x >= obstacle->x1 && this->x <= obstacle->x2 &&
        this->y - this->radius <= obstacle->y2)
    {
        *type = ObstacleCollisionType::BOTTOM_EDGE;
        return true;
    }

    /* 3. ball intersects with a corner of rectangle 
       (these intersections are not found by 2.) */
    if (pointInside(obstacle->x1, obstacle->y1))
    {
        *type = ObstacleCollisionType::TOPLEFT_CORNER;
        return true;
    }
    if (pointInside(obstacle->x2, obstacle->y1))
    {
        *type = ObstacleCollisionType::TOPRIGHT_CORNER;
        return true;
    }
    if (pointInside(obstacle->x1, obstacle->y2))
    {
        *type = ObstacleCollisionType::BOTTOMLEFT_CORNER;
        return true;
    }
    if (pointInside(obstacle->x2, obstacle->y2))
    {
        *type = ObstacleCollisionType::BOTTOMRIGHT_CORNER;
        return true;
    }

    return false;
}

bool Ball::pointInside(number x, number y) const
{
    number dx, dy;

    dx = this->x - x;
    dy = this->y - y;
    if (dx*dx + dy*dy <= this->radius*this->radius)
        return true;
    else
        return false;
}

void Ball::draw(const Cairo::RefPtr<Cairo::Context>& cr,
	const int width, const int height) const
{
	int sx = ((this->x - MINX) / (MAXX - MINX)) * width;
	int sy = ((this->y - MINY) / (MAXY - MINY)) * height;
		
	cr->save();
	cr->arc(sx, sy, width / 6, 0.0, 2.0 * M_PI); // full circle
	cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
	cr->fill_preserve();
	cr->restore();  // back to opaque black
	cr->stroke();
}
