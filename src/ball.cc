#include <cmath>
#include <iostream>

#include "ball.h"

using namespace std;

Ball::Ball(bool tinyMode, bool debianMode, bool cornersMode, std::vector<int> startAngles)
{ 
    int angle;
    
    if (tinyMode) // tiny mode: 95% small balls
    {
        int tinyProb = new_number_random_int(1, 100);
        if (tinyProb < 95)
            this->mass = new_number_random(MIN_MASS, MIN_MASS + 0.1 * (MAX_MASS-MIN_MASS));
        else // insert a few large balls
            this->mass = new_number_random(MIN_MASS + 0.9*(MAX_MASS-MIN_MASS), MAX_MASS);
    }
    else if (debianMode)
    {
        // insert only small balls in "debian mode"
        this->mass = new_number_random(MIN_MASS, MIN_MASS + 0.1 * (MAX_MASS-MIN_MASS));
    }
    else
    {
        this->mass = new_number_random(MIN_MASS, MAX_MASS);
    }

    this->radius = ((MAX_RADIUS - MIN_RADIUS) * (this->mass - MIN_MASS) / MAX_MASS) + MIN_RADIUS;
    
    this->r = new_number_random(0.0, 1.0);
    this->g = new_number_random(0.0, 1.0);
    this->b = new_number_random(0.0, 1.0);
    this->a = 0.9;

    if (startAngles.empty()) // alll angles with the same probability
    {
        double angle = new_number_random(0.0, 360.0)*M_PI/180.0;
        this->dx = cos(angle);
        this->dy = sin(angle);
    }
    else
    {
        int whichAngle = new_number_random_int(0, startAngles.size() - 1);
        angle = startAngles[whichAngle];
        this->dx = cos(M_PI/180.0 * angle);
        this->dy = sin(M_PI/180.0 * angle);
    }

    this->velocity = new_number_random(MIN_VELOCITY, MAX_VELOCITY);
    if (debianMode || cornersMode)
    {
        this->velocity = MIN_VELOCITY + (MAX_VELOCITY-MIN_VELOCITY)/2.0;
    }

    this->lastEvent.type = EventType::NONE;
    this->lastEvent.obstacle = nullptr;
    this->lastEvent.ball = nullptr;
    this->lastEvent.ball2 = nullptr;

    this->x = -numeric_limits<number>::max();
    this->y = -numeric_limits<number>::max();
}

void Ball::randomizePosition()
{
    Vector2D position;
    position = new_vector_random (MINX + this->radius, MAXX - this->radius,
                                  MINY + this->radius, MAXY - this->radius);
    this->x = position.x;
    this->y = position.y;
}

void Ball::print(std::ostream& out) const
{
	out << "Ball@" << x << "," << y << ": r=" << radius;
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

void Ball::findPolygonalObstaclePathIntersection(PolygonalObstacle* polyObs, Event* ev)
{
    int numberPoints = polyObs->points.size();
    Event* cornersAndEdges = reinterpret_cast<Event*>(calloc(numberPoints*2, sizeof(Event)));
    int k;

    for (k = 0; k < numberPoints; k++)
    {
        int i = k;
        int j = (k == numberPoints - 1) ? 0 : k+1;
        Vector2D P1, P2, P1P2, v;
        number t;

        /* check for path intersection with corner k */
        cornersAndEdges[k].type = EventType::POLYGONAL_OBSTACLE_COLLISION;
        cornersAndEdges[k].polyObstacle = polyObs;
        cornersAndEdges[k].polyObsCollType = k;
        cornersAndEdges[k].ball = this;
        cornersAndEdges[k].ball2 = NULL;
        cornersAndEdges[k].obstacle = NULL;
        /* do not re-collide with same obstacle corner with no event in
           between! */
        if (this->lastEvent.type == EventType::POLYGONAL_OBSTACLE_COLLISION &&
            this->lastEvent.polyObsCollType == k)
            cornersAndEdges[k].delta_t = INFINITY;
        else
            cornersAndEdges[k].delta_t =
                findPathPointIntersection(polyObs->points[k].x,
                                          polyObs->points[k].y);

        /* check for path intersection with edge (i,j) */
        P1 = polyObs->points[i];
        P2 = polyObs->points[j];
        
        cornersAndEdges[numberPoints + k].type = EventType::POLYGONAL_OBSTACLE_COLLISION;
        cornersAndEdges[numberPoints + k].polyObstacle = polyObs;
        cornersAndEdges[numberPoints + k].polyObsCollType = numberPoints + k;
        cornersAndEdges[numberPoints + k].ball = this;
        cornersAndEdges[numberPoints + k].ball2 = NULL;
        cornersAndEdges[numberPoints + k].obstacle = NULL;

        /* do not re-collide with same obstacle edge with no event in
           between! */
        if (this->lastEvent.type == EventType::POLYGONAL_OBSTACLE_COLLISION &&
            this->lastEvent.polyObsCollType == numberPoints + k)
        {
            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
            continue;
        }

        /* 
           1. is b moving *towards* edge (i,j)?
           solve([bx+vx*t = p1x + u*(p2x-p1x), 
                  by+vy*t = p1y + u*(p2y-p1y)], [t,u]); 
        */
        t = -(this->x*(P2.y-P1.y)+P1.x*(this->y-P2.y)+(P1.y-this->y)*P2.x)/
            (this->dy*(-P2.x+P1.x) + (P2.y-P1.y)*this->dx);
        if (t < 0.0) /* the ball is moving away from edge (i,j) */
        {
            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
        }
        else /* the ball is moving towards edge (i,j) */
        {
            Vector2D yAxis, b;
            number cos_alpha, alpha;

            P1P2 = Vector2D_sub(&P2, &P1);
            v.x = this->dx;
            v.y = this->dy;

            b.x = this->x;
            b.y = this->y;

            if (P1P2.y < 0.0)
            {
                yAxis.x = 0.0;
                yAxis.y = -1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                if (P1P2.x >= 0.0)
                { /* Quadrant I */
                    Vector2D_rotate(&v, alpha, false);
                    Vector2D_rotate(&P1P2, alpha, false);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, false);
                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
                    
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + this->radius - b.x)/(v.x * this->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - this->radius - b.x)/(v.x * this->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * this->velocity * cornersAndEdges[numberPoints + k].delta_t;
                        if (y < fmin(P1.y, P2.y) || y > fmax(P1.y, P2.y))
                            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    }
                }
                else
                { /* Quadrant II */
                    Vector2D_rotate(&v, alpha, true);
                    Vector2D_rotate(&P1P2, alpha, true);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, true);

                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
           
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + this->radius - b.x)/(v.x * this->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - this->radius - b.x)/(v.x * this->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * this->velocity * cornersAndEdges[numberPoints + k].delta_t;
                        if (y < fmin(P1.y, P2.y) || y > fmax(P1.y, P2.y))
                            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    }
                }
            }
            else
            {
                yAxis.x = 0.0;
                yAxis.y = 1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                if (P1P2.x >= 0.0)
                { /* Quadrant IV */
                    Vector2D_rotate(&v, alpha, true);
                    Vector2D_rotate(&P1P2, alpha, true);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, true);
                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
                    
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + this->radius - b.x)/(v.x * this->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - this->radius - b.x)/(v.x * this->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * this->velocity * cornersAndEdges[numberPoints + k].delta_t;
                        if (y < fmin(P1.y, P2.y) || y > fmax(P1.y, P2.y))
                            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    }
                }
                else
                { /* Quadrant III */
                    Vector2D_rotate(&v, alpha, false);
                    Vector2D_rotate(&P1P2, alpha, false);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, false);

                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
                    
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + this->radius - b.x)/(v.x * this->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - this->radius - b.x)/(v.x * this->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * this->velocity * cornersAndEdges[numberPoints + k].delta_t;
                        if (y < fmin(P1.y, P2.y) || y > fmax(P1.y, P2.y))
                            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    }
                }
            }
        }
    }
        
    *ev = Event::find_closest(cornersAndEdges, numberPoints*2);
    free(cornersAndEdges);
}

void Ball::findWallIntersection(Event* ev)
{
    Event walls[4];
    number 
        bx = this->x + EPS * this->dx, 
        by = this->y + EPS * this->dy;

    /* future collision with left wall?
       (bx + this->dx*this->velocity*t = MINX + this->radius) */
    walls[0].type = EventType::LEFT_WALL_HIT;
    walls[0].ball = this;
    walls[0].ball2 = nullptr;
    walls[0].delta_t = (MINX - bx + this->radius)/(this->dx * this->velocity);
    if (walls[0].delta_t < 0)
        walls[0].delta_t = INFINITY;

    /* future collision with right wall?
       (bx + this->dx*this->velocity*t = MAXX - this->radius) */
    walls[1].type = EventType::RIGHT_WALL_HIT;
    walls[1].ball = this;
    walls[1].ball2 = nullptr;
    walls[1].delta_t = (MAXX - bx - this->radius)/(this->dx * this->velocity);
    if (walls[1].delta_t < 0)
        walls[1].delta_t = INFINITY;

    /* future collision with top wall?
       (by + this->dy*this->velocity*t = MINY + this->radius) */
    walls[2].type = EventType::TOP_WALL_HIT;
    walls[2].ball = this;
    walls[2].ball2 = nullptr;
    walls[2].delta_t = (MINY - by + this->radius)/(this->dy * this->velocity);
    if (walls[2].delta_t < 0)
        walls[2].delta_t = INFINITY;
  
    /* future collision with bottom wall?
       (by + this->dy*this->velocity*t = MAXY - this->radius) */
    walls[3].type = EventType::BOTTOM_WALL_HIT;
    walls[3].ball = this;
    walls[3].ball2 = nullptr;
    walls[3].delta_t = (MAXY - by - this->radius)/(this->dy * this->velocity);
    if (walls[3].delta_t < 0)
        walls[3].delta_t = INFINITY;

    /* 
       qsort(walls, 4, sizeof(Event), event_comp);
       *ev = walls[0]; 
       */

    *ev = Event::find_closest(walls, 4);
}

void Ball::findObstacleIntersection(Obstacle* ob, Event* ev)
{
    int i;
    Vector2D center;
    Event obsColls[8];
    number 
        bx = this->x + EPS * this->dx, 
        by = this->y + EPS * this->dy,
        r = this->radius;
    number x, y;
    
    for (i = 0; i < 8; i++)
    {
        obsColls[i].type = EventType::OBSTACLE_COLLISION;
        obsColls[i].ball = this;
        obsColls[i].ball2 = nullptr;
        obsColls[i].obstacle = ob;
    }

    /* future collision with left edge? */
    obsColls[0].obsCollType = ObstacleCollisionType::LEFT_EDGE;
    obsColls[0].delta_t = (ob->x1 - r - bx)/(this->dx * this->velocity);
    if (obsColls[0].delta_t < 0)
        obsColls[0].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the left edge? */
        y = by + this->dy * this->velocity * obsColls[0].delta_t;
        if (y < ob->y1 || y > ob->y2)
            obsColls[0].delta_t = INFINITY;
    }

    /* future collision with right edge? */
    obsColls[1].obsCollType = ObstacleCollisionType::RIGHT_EDGE;
    obsColls[1].delta_t = (ob->x2 + r - bx)/(this->dx * this->velocity);
    if (obsColls[1].delta_t < 0)
        obsColls[1].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the right edge? */
        y = by + this->dy * this->velocity * obsColls[1].delta_t;
        if (y < ob->y1 || y > ob->y2)
            obsColls[1].delta_t = INFINITY;
    }

    /* future collision with top edge? */
    obsColls[2].obsCollType = ObstacleCollisionType::TOP_EDGE;
    obsColls[2].delta_t = (ob->y1 - r - by)/(this->dy * this->velocity);
    if (obsColls[2].delta_t < 0)
        obsColls[2].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the top edge? */
        x = bx + this->dx * this->velocity * obsColls[2].delta_t;
        if (x < ob->x1 || x > ob->x2)
            obsColls[2].delta_t =INFINITY;
    }

    /* future collision with bottom edge? */
    obsColls[3].obsCollType = ObstacleCollisionType::BOTTOM_EDGE;
    obsColls[3].delta_t = (ob->y2 + r - by)/(this->dy * this->velocity);
    if (obsColls[3].delta_t < 0)
        obsColls[3].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the bottom edge? */
        x = bx + this->dx * this->velocity * obsColls[3].delta_t;
        if (x < ob->x1 || x > ob->x2)
            obsColls[3].delta_t =INFINITY;
    }

    center.x = (ob->x1 + ob->x2)/2.0;
    center.y = (ob->y1 + ob->y2)/2.0;

    /* future intersection with top left corner? */
    obsColls[4].obsCollType = ObstacleCollisionType::TOPLEFT_CORNER;
    if (bx < center.x && by < center.y)
        obsColls[4].delta_t = findPathPointIntersection(ob->x1, ob->y1);
    else
        obsColls[4].delta_t = INFINITY;
    
    /* future intersection with top right corner? */
    obsColls[5].obsCollType = ObstacleCollisionType::TOPRIGHT_CORNER;
    if (bx > center.x && by < center.y)
        obsColls[5].delta_t = findPathPointIntersection(ob->x2, ob->y1);
    else
        obsColls[5].delta_t = INFINITY;

    /* future intersection with bottom left corner? */
    obsColls[6].obsCollType = ObstacleCollisionType::BOTTOMLEFT_CORNER;
    if (bx < center.x && by > center.y)
        obsColls[6].delta_t = findPathPointIntersection(ob->x1, ob->y2);
    else
        obsColls[6].delta_t = INFINITY;

    /* future intersection with bottom right corner? */
    obsColls[7].obsCollType = ObstacleCollisionType::BOTTOMRIGHT_CORNER;
    if (bx > center.x && by > center.y)
        obsColls[7].delta_t = findPathPointIntersection(ob->x2, ob->y2);
    else
        obsColls[7].delta_t = INFINITY;

    /*
      qsort(obsColls, 8, sizeof(Event), event_comp);
      *ev = obsColls[0];
      */
    *ev = Event::find_closest(obsColls, 8);
}

number Ball::findPathIntersection(Ball* b1, Ball* b2)
{
    number 
        px1 = b1->x + EPS*b1->dx, 
        py1 = b1->y + EPS*b1->dy, 
        px2 = b2->x + EPS*b2->dx, 
        py2 = b2->y + EPS*b2->dy;
    number vx1 = b1->dx*b1->velocity, vy1 = b1->dy*b1->velocity,
        vx2 = b2->dx*b2->velocity, vy2 = b2->dy*b2->velocity;
    number r1 = b1->radius, r2 = b2->radius;
    number sqrtArg, sqrtValue;
    number t1, t2;

    sqrtArg = (pow2(r2)+2*r1*r2+pow2(r1)-pow2(px2)+2*px1*px2-pow2(px1))*pow2(vy2)+((-2*pow2(r2)-4*r1*r2-2*pow2(r1)+2*pow2(px2)-4*px1*px2+2*pow2(px1))*vy1+((2*px2-2*px1)*py2+(2*px1-2*px2)*py1)*vx2+((2*px1-2*px2)*py2+(2*px2-2*px1)*py1)*vx1)*vy2+(pow2(r2)+2*r1*r2+pow2(r1)-pow2(px2)+2*px1*px2-pow2(px1))*pow2(vy1)+(((2*px1-2*px2)*py2+(2*px2-2*px1)*py1)*vx2+((2*px2-2*px1)*py2+(2*px1-2*px2)*py1)*vx1)*vy1+(pow2(r2)+2*r1*r2+pow2(r1)-pow2(py2)+2*py1*py2-pow2(py1))*pow2(vx2)+(-2*pow2(r2)-4*r1*r2-2*pow2(r1)+2*pow2(py2)-4*py1*py2+2*pow2(py1))*vx1*vx2+(pow2(r2)+2*r1*r2+pow2(r1)-pow2(py2)+2*py1*py2-pow2(py1))*pow2(vx1);
  
    if (sqrtArg < 0)
        return INFINITY;

    sqrtValue = sqrt(sqrtArg);

    t1 = -(sqrtValue +(py2-py1)*vy2+(py1-py2)*vy1+
           (px2-px1)*vx2+(px1-px2)*vx1)/(pow2(vy2)-2*vy1*vy2+pow2(vy1)+pow2(vx2)-2*vx1*vx2+pow2(vx1));

    t2 = (sqrtValue +(py1-py2)*vy2+(py2-py1)*vy1+
          (px1-px2)*vx2+(px2-px1)*vx1)/(pow2(vy2)-2*vy1*vy2+pow2(vy1)+pow2(vx2)-2*vx1*vx2+pow2(vx1));

    if (t1 < 0)
        return (t2 < 0) ? INFINITY : t2;
    else if (t2 < 0)
        return t1;
    else
        return fmin(t1, t2);
}

bool Ball::collision_check(Ball* b1, Ball* b2)
{
    double r1pr2, dx, dy;
    r1pr2 = b1->radius + b2->radius;
    dx = b2->x - b1->x;
    dy = b2->y - b1->y;
    if (dx*dx + dy*dy <= r1pr2*r1pr2)
        return true;
    else
        return false;
}

void Ball::setDirectionForTarget(number tx, number ty)
{
    Vector2D dir;
    dir.x = tx - this->x;
    dir.y = ty - this->y;
    Vector2D_normalize(&dir);
    this->dx = dir.x;
    this->dy = dir.y;
}

/*
  check for overlap between ball (bx,by,r) and (bounded) edge p1 +
  u*(p2-p1) by checking whether the shortest distance between the (bounded)
  edge and the ball is <= r:
  
  could be solved by:
  (P2-P1)*(b-H)=0, H=P1 + u*(P2-P1) <=>
  (P2-P1)*(b - P1 -u*P2+u*P1) <=>
  solve([(p2x-p1x)*(bx-p1x-u*p2x+u*p1x) + (p2y-p1y)*(by-p1y-u*p2y+u*p1y)], u);  
    number bx = ball->x, by = ball->y, r = ball->radius;
    number p1x = P1.x, p1y = P1.y, p2x = P2.x, p2y = P2.y;

    number u = -((p1y-by)*p2y+(p1x-bx)*p2x-pow2(p1y)+by*p1y-pow2(p1x)+bx*p1x) /
        (pow2(p2y)-2*p1y*p2y+pow2(p2x)-2*p1x*p2x+pow2(p1y)+pow2(p1x));
  
   but the vector representation is simpler:
   (b-H)*(P2-P1) = 0, H = P1 + u*(P2-P1) <=>
   (b-P1-u*(P2-P1))*(P2-P1) = 0 <=>
   b*(P2-P1) - P1*(P2-P1) - u*(P2-P1)^2 = 0 <=>
   (b-P1)*(P2-P1) = u*(P2-P1)^2 <=>
   u = (b-P1)*(P2-P1)/(P2-P1)^2
*/
bool Ball::boundedEdge_collision_check(Vector2D P1, Vector2D P2) const
{
    Vector2D H, shortestPath, diffP2P1, diffbP1;
    number u;

    diffP2P1.x = P2.x - P1.x;
    diffP2P1.y = P2.y - P1.y;
    diffbP1.x = this->x-P1.x;
    diffbP1.y = this->y-P1.y;
    u = Vector2D_scalarProduct(&diffbP1, &diffP2P1) /
        Vector2D_scalarProduct(&diffP2P1, &diffP2P1);

    if (u < 0.0)
    { /* the shortest path between the P1-P2 line and b=(bx,by) lies outside
         of the line *segment* (defined by P1 and P2)
         => measure distance(b, P1) instead
      */
        u = 0.0;
    }
    if (u > 1.0)
    { /* the shortest path between the P1-P2 line and b lies outside
         of the line *segment* (defined by P1 and P2)
         => measure distance(b, P2) instead
      */
        u = 1.0;
    }
    H.x = P1.x + u*(P2.x-P1.x);
    H.y = P1.y + u*(P2.y-P1.y);
    
    shortestPath.x = H.x - this->x;
    shortestPath.y = H.y - this->y;
    return Vector2D_length(&shortestPath) <= this->radius;
}

/*
  Use the ray casting algorithm:
    http://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
  see how many polygon edges a horizontal line starting from p=(px,py) crosses.
  if and only if this is odd, the point lies inside the polygon.
 */
bool Ball::pointInPolygon(number px, number py, PolygonalObstacle* polyObs)
{
    int k, numberIntersections = 0;
    int numberPoints = polyObs->points.size();

    for (k = 0; k < numberPoints; k++)
    {
        Vector2D pi = polyObs->points[k];
        Vector2D pj = (k == numberPoints - 1) ? polyObs->points[0] : polyObs->points[k+1];
        number pix = pi.x, piy = pi.y, pjx = pj.x, pjy = pj.y, u, v;

        /* special case: edge is horizontal */
        if (piy == pjy)
        {
            /* ignore this */
            continue;
        }

        /* 
           horizontal line: h = p + u*(1,0)
           polygon edge: e = pi + v*(pj-pi)
           h = e <=> 
           solve([px + u = pix + v*(pjx - pix),py = piy + v*(pjy-piy)], [u,v]); 
        */
        u = -(pix*(py-pjy)+pjx*(piy-py)+(pjy-piy)*px)/(pjy-piy); /* position on horizontal line */
        v = (py-piy)/(pjy-piy); /* position on edge */

        if (u < 0.0)
        {
            /* intersection is behind p */
            continue;
        }
        if (v >= 0 && v <= 1.0) /* is intersection inside bounds of edge?*/
        {
            numberIntersections++;
        }
    }
    return numberIntersections % 2 == 1;
}

/* TODO: what if the ball is totally contained in the polygon?? */
bool Ball::polygonalObstacle_collision_check(PolygonalObstacle* polyObs) const
{
    /* 1. is the ball's center contained in the polygon? */
    if (pointInPolygon(this->x, this->y, polyObs))
    {
        return true;
    }
    
    /* 2. does the ball intersect any edge? */
    for (int k = 0; k < polyObs->points.size(); k++)
    {
        int i = k;
        int j = (k == polyObs->points.size() - 1) ? 0 : k+1;
        if (boundedEdge_collision_check(polyObs->points[i],
                                        polyObs->points[j]))
        {
            return true;
        }
    }
    return false;
}

bool Ball::obstacle_collision_check(Obstacle* obstacle, ObstacleCollisionType* type) const
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

void Ball::rotate(number degrees, bool clockWise)
{
    Vector2D dirVec;
  
    dirVec.x = this->dx;
    dirVec.y = this->dy;
    
    Vector2D_rotate(&dirVec, degrees, clockWise);
    
    this->dx = dirVec.x;
    this->dy = dirVec.y;
}

bool Ball::doCollision(Ball* b1, Ball* b2)
{
    Vector2D position_b1, position_b2, line_of_sight, b1_velocity_before, b2_velocity_before, b1_velocity_after, b2_velocity_after;
    double b1_velocity_in_line_of_sight, b2_velocity_in_line_of_sight, b1_velocity_in_line_of_sight_before, b2_velocity_in_line_of_sight_before, b1_change_in_line_of_sight_velocity, b2_change_in_line_of_sight_velocity;

#ifdef LOGGING
    std::cout << "Ball_doCollision()\n";
#endif
    /*
      if (b1->lastEvent.type == BALL_COLLISION &&
      b1->lastEvent.otherBall == b2 &&
      b2->lastEvent.type == BALL_COLLISION &&
      b2->lastEvent.otherBall == b1)
      return false;
    */

    position_b1.x = b1->x; position_b1.y = b1->y;
    position_b2.x = b2->x; position_b2.y = b2->y;
    line_of_sight = Vector2D_sub(&position_b2, &position_b1);

    Vector2D_normalize(&line_of_sight);

    b1_velocity_before.x = b1->dx * b1->velocity; b1_velocity_before.y = b1->dy * b1->velocity;
    b2_velocity_before.x = b2->dx * b2->velocity; b2_velocity_before.y = b2->dy * b2->velocity;

    b1_velocity_in_line_of_sight_before = Vector2D_scalarProjectionOfVec1OntoVec2(&b1_velocity_before, &line_of_sight);
    b2_velocity_in_line_of_sight_before = Vector2D_scalarProjectionOfVec1OntoVec2(&b2_velocity_before, &line_of_sight);

    b1_velocity_in_line_of_sight = b1_velocity_in_line_of_sight_before * (b1->mass - b2->mass) / (b1->mass + b2->mass) + b2_velocity_in_line_of_sight_before * 2.0 * b2->mass / (b1->mass + b2->mass);
    b2_velocity_in_line_of_sight = b1_velocity_in_line_of_sight_before * 2.0 * b1->mass / (b1->mass + b2->mass) + b2_velocity_in_line_of_sight_before * (b2->mass - b1->mass) / (b2->mass + b1->mass);

    b1_change_in_line_of_sight_velocity = b1_velocity_in_line_of_sight - b1_velocity_in_line_of_sight_before;
    b2_change_in_line_of_sight_velocity = b2_velocity_in_line_of_sight - b2_velocity_in_line_of_sight_before;

    b1_velocity_after.x = b1_velocity_before.x + line_of_sight.x * b1_change_in_line_of_sight_velocity;
    b1_velocity_after.y = b1_velocity_before.y + line_of_sight.y * b1_change_in_line_of_sight_velocity;

    b2_velocity_after.x = b2_velocity_before.x + line_of_sight.x * b2_change_in_line_of_sight_velocity;
    b2_velocity_after.y = b2_velocity_before.y + line_of_sight.y * b2_change_in_line_of_sight_velocity;

    b1->velocity = Vector2D_length(&b1_velocity_after);
    b2->velocity = Vector2D_length(&b2_velocity_after);
    Vector2D_normalize(&b1_velocity_after);
    Vector2D_normalize(&b2_velocity_after);
    b1->dx = b1_velocity_after.x;
    b1->dy = b1_velocity_after.y;
    b2->dx = b2_velocity_after.x;
    b2->dy = b2_velocity_after.y;

    b1->lastEvent.type = EventType::BALL_COLLISION;
    b1->lastEvent.ball = b1;
    b1->lastEvent.ball2 = b2;
    b2->lastEvent.type = EventType::BALL_COLLISION;
    b2->lastEvent.ball = b2;
    b2->lastEvent.ball2 = b1;
  
    /*
      b1->x += b1->dx * b1->velocity * SINGLE_TIME_STEP;
      b1->y += b1->dy * b1->velocity * SINGLE_TIME_STEP;
      b2->x += b2->dx * b2->velocity * SINGLE_TIME_STEP;
      b2->y += b2->dy * b2->velocity * SINGLE_TIME_STEP;
    */

    return true;
}

bool Ball::doObstacleCollision(Obstacle* obstacle, ObstacleCollisionType obsCollType)
{
    switch(obsCollType)
    {
        Vector2D center_of_ball, corner, virtualWall;
        number cosAngle, angle;
      
    case ObstacleCollisionType::LEFT_EDGE:
    case ObstacleCollisionType::RIGHT_EDGE:
        this->dx = -this->dx;
        break;
    case ObstacleCollisionType::TOP_EDGE:
    case ObstacleCollisionType::BOTTOM_EDGE:
        this->dy = -this->dy;
        break;
    case ObstacleCollisionType::TOPLEFT_CORNER:
#ifdef LOGGING
        std::cout << "Ball_doObstacleCollision(): TOPLEFT_CORNER\n";
#endif
        center_of_ball.x = this->x;
        center_of_ball.y = this->y;
        corner.x = obstacle->x1;
        corner.y = obstacle->y1;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, false);
        /* determine angle to x axis (scalar product between virtualWall and
           obstacle edge (0,1)) */
        cosAngle = virtualWall.y;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to y
           axis */
        rotate(angle, false);
        /* 2. treat it like a top wall hit */
        this->dx = -this->dx;
        /* 3. rotate back */
        rotate(angle, true);

        break;
    case ObstacleCollisionType::TOPRIGHT_CORNER:
#ifdef LOGGING
        std::cout << "Ball_doObstacleCollision(): TOPRIGHT_CORNER\n";
#endif

        center_of_ball.x = this->x;
        center_of_ball.y = this->y;
        corner.x = obstacle->x2;
        corner.y = obstacle->y1;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, false);
        /* determine angle to x axis (scalar product between virtualWall and
           obstacle edge (-1, 0)) */
        cosAngle = -virtualWall.x;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to x
           axis */
        rotate(angle, false);
        /* 2. treat it like a bottom wall hit */
        this->dy = -this->dy;
        /* 3. rotate back */
        rotate(angle, true);
        break;

    case ObstacleCollisionType::BOTTOMLEFT_CORNER:
#ifdef LOGGING
        std::cout << "Ball_doObstacleCollision(): BOTTOMLEFT_CORNER\n";
#endif

        center_of_ball.x = this->x;
        center_of_ball.y = this->y;
        corner.x = obstacle->x1;
        corner.y = obstacle->y2;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, false);
        /* determine angle to x axis (scalar product between virtualWall and
           obstacle edge (1,0)) */
        cosAngle = virtualWall.x;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to x
           axis */
        rotate(angle, false);
        /* 2. treat it like a top wall hit */
        this->dy = -this->dy;
        /* 3. rotate back */
        rotate(angle, true);
        break;

    case ObstacleCollisionType::BOTTOMRIGHT_CORNER:
#ifdef LOGGING
        std::cout << "Ball_doObstacleCollision(): BOTTOMRIGHT_CORNER\n";
#endif

        center_of_ball.x = this->x;
        center_of_ball.y = this->y;
        corner.x = obstacle->x2;
        corner.y = obstacle->y2;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, false);
        /* determine angle to y axis (scalar product between virtualWall and
           obstacle edge (0,-1)) */
        cosAngle = -virtualWall.y;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to y
           axis */
        rotate(angle, false);
        /* 2. treat it like a right wall hit */
        this->dx = -this->dx;
        /* 3. rotate back */
        rotate(angle, true);
        break;

    case ObstacleCollisionType::CENTER_INSIDE:
        std::cout << "Warning: last obstacle collision type is CENTER_INSIDE. Need smaller step size?\n";
        abort();
        break;
    }

    this->lastEvent.type = EventType::OBSTACLE_COLLISION;
    this->lastEvent.ball = this;
    this->lastEvent.ball2 = nullptr;
    this->lastEvent.obstacle = obstacle;
    this->lastEvent.obsCollType = obsCollType;

    return true;
}

bool Ball::doPolygonalObstacleCollision(PolygonalObstacle* polyObs, int polyObsCollType)
{
    Vector2D center_of_ball, corner, virtualWall;
    number cosAngle, angle;

    center_of_ball.x = this->x;
    center_of_ball.y = this->y;
    
    if (polyObsCollType < polyObs->points.size()) /* corner collision */
    { 
        corner.x = polyObs->points[polyObsCollType].x;
        corner.y = polyObs->points[polyObsCollType].y;

        if (center_of_ball.x >= corner.x)
        {
            if (center_of_ball.y >= corner.y)
            {   /* ball is bottom right relative to corner */

                /* calculate the normal to the virtual wall */
                virtualWall = Vector2D_sub(&center_of_ball, &corner);
                Vector2D_normalize(&virtualWall);
                /* convert normal to virtual wall */
                Vector2D_rotate(&virtualWall, 90.0, false);
                /* determine angle to y axis (scalar product between virtualWall and
                   obstacle edge (0,-1)) */
                cosAngle = -virtualWall.y;
                angle = acos(cosAngle) * 180.0/M_PI;

                /* 1. rotate the setup so that the virtual wall is parallel to y
                   axis */
                this->rotate(angle, false);
                /* 2. treat it like a right wall hit */
                this->dx = -this->dx;
                /* 3. rotate back */
                this->rotate(angle, true);
            }
            else
            {   /* ball is top right relative to corner */

                /* calculate the normal to the virtual wall */
                virtualWall = Vector2D_sub(&center_of_ball, &corner);
                Vector2D_normalize(&virtualWall);
                /* convert normal to virtual wall */
                Vector2D_rotate(&virtualWall, 90.0, false);
                /* determine angle to x axis (scalar product between virtualWall and
                   obstacle edge (-1, 0)) */
                cosAngle = -virtualWall.x;
                angle = acos(cosAngle) * 180.0/M_PI;

                /* 1. rotate the setup so that the virtual wall is parallel to x
                   axis */
                this->rotate(angle, false);
                /* 2. treat it like a bottom wall hit */
                this->dy = -this->dy;
                /* 3. rotate back */
                this->rotate(angle, true);
            }
        }
        else
        {
            if (center_of_ball.y >= corner.y)
            {   /* ball is bottom left relative to corner */

                /* calculate the normal to the virtual wall */
                virtualWall = Vector2D_sub(&center_of_ball, &corner);
                Vector2D_normalize(&virtualWall);
                /* convert normal to virtual wall */
                Vector2D_rotate(&virtualWall, 90.0, false);
                /* determine angle to x axis (scalar product between virtualWall and
                   obstacle edge (1,0)) */
                cosAngle = virtualWall.x;
                angle = acos(cosAngle) * 180.0/M_PI;

                /* 1. rotate the setup so that the virtual wall is parallel to x
                   axis */
                this->rotate(angle, false);
                /* 2. treat it like a top wall hit */
                this->dy = -this->dy;
                /* 3. rotate back */
                this->rotate(angle, true);

            }
            else
            {   /* ball is top left relative to corner */

                /* calculate the normal to the virtual wall */
                virtualWall = Vector2D_sub(&center_of_ball, &corner);
                Vector2D_normalize(&virtualWall);
                /* convert normal to virtual wall */
                Vector2D_rotate(&virtualWall, 90.0, false);
                /* determine angle to x axis (scalar product between virtualWall and
                   obstacle edge (0,1)) */
                cosAngle = virtualWall.y;
                angle = acos(cosAngle) * 180.0/M_PI;
                
                /* 1. rotate the setup so that the virtual wall is parallel to y
                   axis */
                this->rotate(angle, false);
                /* 2. treat it like a top wall hit */
                this->dx = -this->dx;
                /* 3. rotate back */
                this->rotate(angle, true);
            }
        }
    }
    else /* edge collision */
    {
        Vector2D P1, P2, P1P2, v, yAxis;
        int k = polyObsCollType - polyObs->points.size();
        number cos_alpha, alpha;

        P1 = polyObs->points[k];
        P2 = polyObs->points[k == polyObs->points.size() - 1 ? 0 : k+1];

        P1P2 = Vector2D_sub(&P2, &P1);
        v.x = this->dx;
        v.y = this->dy;
        
        if (P1P2.y < 0.0)
        {
            if (P1P2.x >= 0.0)
            { /* quadrant I */
                yAxis.x = 0.0;
                yAxis.y = -1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                Vector2D_rotate(&v, alpha, false);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, true);
                this->dx = v.x;
                this->dy = v.y;
            }
            else
            { /* Quadrant II */
                yAxis.x = 0.0;
                yAxis.y = -1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                Vector2D_rotate(&v, alpha, true);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, false);
                this->dx = v.x;
                this->dy = v.y;
            }
        }
        else
        {
            if (P1P2.x >= 0.0)
            { /* Quadrant IV */
                yAxis.x = 0.0;
                yAxis.y = 1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                Vector2D_rotate(&v, alpha, true);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, false);
                this->dx = v.x;
                this->dy = v.y;
            }
            else
            { /* Quadrant III */
                yAxis.x = 0.0;
                yAxis.y = 1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                Vector2D_rotate(&v, alpha, false);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, true);
                this->dx = v.x;
                this->dy = v.y;
            }
        }
    }

    this->lastEvent.type = EventType::POLYGONAL_OBSTACLE_COLLISION;
    this->lastEvent.ball = this;
    this->lastEvent.ball2 = nullptr;
    this->lastEvent.polyObstacle = polyObs;
    this->lastEvent.polyObsCollType = polyObsCollType;

    return true;
}
