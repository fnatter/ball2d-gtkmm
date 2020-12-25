/* xscreensaver, Copyright (c) 1992, 1995, 1996, 1997, 1998
 *  Jamie Zawinski <jwz@jwz.org>
 *
 * Permission to use, copy, modify, distribute, and sell this software and its
 * documentation for any purpose is hereby granted without fee, provided that
 * the above copyright notice appear in all copies and that both that
 * copyright notice and this permission notice appear in supporting
 * documentation.  No representations are made about the suitability of this
 * software for any purpose.  It is provided "as is" without express or 
 * implied warranty.
 */

/*
  Copyleft 2011, Felix Natter <fnatter@gmx.net>
  
  If you want to understand the physics for ball collisions, just drop me a
  mail (fnatter@gmx.net) and I'll send you a pdf explaining this (you can also
  ask about other aspects of ball2d.c :-).
 */

#include <math.h>
#include "screenhack.h"
#include "alpha.h"
#ifdef HAVE_DOUBLE_BUFFER_EXTENSION
# include "xdbe.h"
#endif /* HAVE_DOUBLE_BUFFER_EXTENSION */
#include <time.h>
#include <sys/time.h>
#include <X11/Xmu/Drawing.h>

/* #define LOGGING 1 */
/* #define EVENT_LOGGING 1 */

/* typedefs / structs */
typedef double number;

/* collision type for Ball vs axis-aligned rectangular obstacles */
typedef enum {
    LEFT_EDGE = 0, RIGHT_EDGE, TOP_EDGE, BOTTOM_EDGE,
    TOPLEFT_CORNER, TOPRIGHT_CORNER, BOTTOMLEFT_CORNER, BOTTOMRIGHT_CORNER,
    CENTER_INSIDE /* this shouldn't happen */
} ObstacleCollisionType;

typedef struct _Vector2D {
    number x;
    number y;
} Vector2D;

typedef struct _Obstacle
{
    number x1, y1, x2, y2;
} Obstacle;

typedef struct
{
    Vector2D* points;
    int numberPoints;
} PolygonalObstacle;

struct _Ball;

typedef enum {
    NONE = 0,
    LEFT_WALL_HIT, RIGHT_WALL_HIT, TOP_WALL_HIT, BOTTOM_WALL_HIT,
    OBSTACLE_COLLISION, POLYGONAL_OBSTACLE_COLLISION, BALL_COLLISION
} EventType;

typedef struct _Event {
    EventType type;

    Obstacle* obstacle;
    ObstacleCollisionType obsCollType;
    /* 0..numberPoints-1: points, 
       numberpoints..2*numberpoints-1: edges */
    int polyObsCollType; 
    PolygonalObstacle* polyObstacle;

    struct _Ball* ball;
    struct _Ball* ball2;

    number delta_t;
} Event;


typedef struct _Ball {
    number x,y;
    number dx,dy;
    number mass;
    number radius;
    unsigned long color;
    double velocity;

    struct _Event lastEvent;

    GC gc;
} Ball;

struct state {
    Display *dpy;
    Window window;
    long iteration_number;
    time_t startTime;
    time_t lastZombieTime;

    Ball* balls;
    int nObstacles;
    Obstacle* obstacles;
    int nPolyObstacles;
    PolygonalObstacle* polyObstacles;
  
    Bool dbuf;
    int delay;
    int count;
    int ncolors;
    Bool showVelocityVectors;
    int* startAngles;
    int numStartAngles;
    Bool tiny;
    Bool startGrid;
    GC velocityVectorGC, obstacleGC, highlightGC;
    Bool zombies;
    Bool corners;
    Bool debian;
    Bool slowStart;
    number max_radius;
    Bool showFutureCollisions;
    Event nextEvent;

    XColor *colors;
    GC erase_gc;
    XWindowAttributes xgwa;
    Pixmap b, ba, bb;	/* double-buffer to reduce flicker */

# ifdef HAVE_DOUBLE_BUFFER_EXTENSION
    Bool dbeclear_p;
    XdbeBackBuffer backb;
# endif /* HAVE_DOUBLE_BUFFER_EXTENSION */
};

/* globals */
number MINX = -100.0;
number MAXX = 100.0;
number MINY = -100.0;
number MAXY = 100.0;
#define ANGLES_ALL (NULL)

/* was: MAX_RADIUS = 6.0, MAX_MASS=2.0, MIN_MASS=0.1 */
const double MIN_RADIUS = 1.0, MAX_RADIUS = 10.0; 
double MIN_VELOCITY = 0.0, MAX_VELOCITY = 2.0, MIN_MASS = 0.5, MAX_MASS = 2.0,
    SINGLE_TIME_STEP = 0.7, SINGLE_STEP = 0.05, EPS=1.0e-4;
#define MAX_MOMENTUM (MAX_VELOCITY * MAX_MASS)

static number
Ball_findPathPointIntersection(Ball* b, number cx, number cy);


static long
time_microseconds(void)
{
    struct timeval t;
    gettimeofday(&t, NULL);
    return t.tv_sec*1000000 + t.tv_usec;
}

/*
  static int 
  event_comp(const void* elem1, const void* elem2)
  {
  if (((const Event*)elem1)->delta_t < ((const Event*)elem2)->delta_t)
  return -1;
  return ((const Event*)elem1)->delta_t > ((const Event*)elem2)->delta_t;
  }
*/

static Event
Events_find_closest(Event* events, int count)
{
    int i;
    Event* closest = &events[0];

    for (i = 1; i < count; i++)
    {
        if (events[i].delta_t < closest->delta_t)
            closest = &events[i];
    }
    return *closest;
}

#ifdef EVENT_LOGGING
static void
Event_print(Event ev)
{
    char* typeNames[] = { "NONE",
                          "LEFT_WALL_HIT", "RIGHT_WALL_HIT",
                          "TOP_WALL_HIT", "BOTTOM_WALL_HIT",
                          "OBSTACLE_COLLISION", "POLYGONAL_OBSTACLE_COLLISION",
                          "BALL_COLLISION" };

    char* obsTypeNames[] = { "LEFT_EDGE", "RIGHT_EDGE", 
                             "TOP_EDGE", "BOTTOM_EDGE",
                             "TOPLEFT_CORNER", "TOPRIGHT_CORNER", 
                             "BOTTOMLEFT_CORNER", "BOTTOMRIGHT_CORNER",
                             "CENTER_INSIDE" };

    if (ev.type == OBSTACLE_COLLISION)
    {
        printf("ev.type=%s::%s, ev.ball=%x, t=%f\n",
               typeNames[ev.type], obsTypeNames[ev.obsCollType], 
               (unsigned int)ev.ball, ev.delta_t);
    }
    else if (ev.type == POLYGONAL_OBSTACLE_COLLISION)
    {
        printf("ev.type=%s::%d, ev.ball=%x, t=%f\n",
               typeNames[ev.type], ev.polyObsCollType,
               (unsigned int)ev.ball, ev.delta_t);
    }
    else
    {
        printf("ev.type=%s, ev.ball=%x, ev.ball2=%x, t=%f\n",
               typeNames[ev.type], (unsigned int)ev.ball,
               (unsigned int)ev.ball2, ev.delta_t);
    }
}
#endif

static number
pow2(number x)
{
    return x*x;
}

static number
new_number_random (number min, number max) {
    return ( (max-min)*((number)random()/(number)RAND_MAX) ) + min;
}

static int
new_number_random_int (int min, int max) {
    return (int)( (max-min+1)*((number)random()/(number)RAND_MAX) ) + min;
}

/* Vector class */
static Vector2D
new_vector_random (number min_x, number max_x,
                   number min_y, number max_y) {
    Vector2D ret;
    ret.x = new_number_random (min_x, max_x);
    ret.y = new_number_random (min_y, max_y);
    return ret;
}

static number
Vector2D_length(Vector2D* vec)
{
    return sqrt(vec->x*vec->x + vec->y*vec->y);
}

static void
Vector2D_normalize(Vector2D* vec)
{
    number len = Vector2D_length(vec);
    vec->x /= len;
    vec->y /= len;
}

/*
  static void
  Vector2D_negate(Vector2D* vec)
  {
  vec->x = -vec->x;
  vec->y = -vec->y;
  }
*/

static Vector2D
Vector2D_add(Vector2D* vec1, Vector2D* vec2)
{
    Vector2D sum;
    sum.x = vec1->x + vec2->x;
    sum.y = vec1->y + vec2->y;
    return sum;
}

static Vector2D
Vector2D_sub(Vector2D* vec1, Vector2D* vec2)
{
    Vector2D diff;
    diff.x = vec1->x - vec2->x;
    diff.y = vec1->y - vec2->y;
    return diff;
}

static Vector2D
Vector2D_multScalar(Vector2D* vec, number scalar)
{
    Vector2D ret;
    ret.x = vec->x * scalar;
    ret.y = vec->y * scalar;
    return ret;
}

static number
Vector2D_scalarProduct(Vector2D* vec1, Vector2D* vec2)
{
    return vec1->x * vec2->x + vec1->y * vec2->y;
}

static number
Vector2D_scalarProjectionOfVec1OntoVec2(Vector2D* vec1, Vector2D* vec2)
{
    return Vector2D_scalarProduct(vec1, vec2) / Vector2D_length(vec2);
}

static void
Vector2D_rotate(Vector2D* vec, number degrees, Bool clockWise)
{
    number x, y, angle = degrees * M_PI/180.0,
        cosAngle, sinAngle;

    /* see: http://en.wikipedia.org/wiki/Cartesian_coordinate_system#Rotation 
       (CW/CCW is reversed in the article, because it assumes that top
       means high y value (which is not true for ball2d))
    */
    /* also: cos(angle) = cos(-angle) = sin(angle) = -sin(-angle) */
    cosAngle = cos(angle);
    sinAngle = sin(angle);
    if (clockWise)
    {
        x = vec->x*cosAngle - vec->y*sinAngle;
        y = vec->x*sinAngle + vec->y*cosAngle;
    }
    else
    {
        x = vec->x*cosAngle  + vec->y*sinAngle;
        y = -vec->x*sinAngle + vec->y*cosAngle;
    }

    vec->x = x;
    vec->y = y;
}

static void
Vector2D_rotateAroundPoint(Vector2D* vec, Vector2D* point,
                           number degrees, Bool clockWise)
{
    *vec = Vector2D_sub(vec, point);
    Vector2D_rotate(vec, degrees, clockWise);
    *vec = Vector2D_add(vec, point);
}

static void
recomputeYRange(struct state* st)
{
    double wld_height = (MAXX-MINX) * st->xgwa.height/(number)st->xgwa.width;
    MINY = -wld_height/2.0;
    MAXY = wld_height/2.0;
}

/*
  static void
  map_length(struct state* st, double wld, int* screen)
  {
  wld /= (MAXY-MINY);
  wld *= (st->xgwa.height-1);
  *screen = (int)(wld + 0.5);
  }
*/

static void
map_length_x(struct state* st, double wld, int* screen)
{
    wld /= (MAXX-MINX);
    wld *= (st->xgwa.width-1);
    *screen = (int)(wld + 0.5);
}

static void
map_length_y(struct state* st, double wld, int* screen)
{
    wld /= (MAXY-MINY);
    wld *= (st->xgwa.height-1);
    *screen = (int)(wld + 0.5);
}

static void
map2screen(struct state* st, double wldx, double wldy, int* scrx, int* scry)
{
    wldx -= MINX;
    wldx /= (MAXX-MINX);
    wldx *= (st->xgwa.width-1);

    wldy -= MINY;
    wldy /= (MAXY-MINY);
    wldy *= (st->xgwa.height-1);

    *scrx = (int)(wldx + 0.5);
    *scry = (int)(wldy + 0.5);
}


static void
Ball_init(struct state* st, Ball* ball, int ballIdx)
{ 
    int angle;

    if (st->tiny) /* tiny mode: 95% small balls */
    {
        int tinyProb = new_number_random_int(1, 100);
        if (tinyProb < 95)
            ball->mass = new_number_random(MIN_MASS, MIN_MASS + 0.1 * (MAX_MASS-MIN_MASS));
        else /* insert a few large balls */
            ball->mass = new_number_random(MIN_MASS + 0.9*(MAX_MASS-MIN_MASS), MAX_MASS);
    }
    else if (st->debian)
    {
        /* insert only small balls in "debian mode" */
        ball->mass = new_number_random(MIN_MASS, MIN_MASS + 0.1 * (MAX_MASS-MIN_MASS));
    }
    else
    {
        ball->mass = new_number_random(MIN_MASS, MAX_MASS);
    }

    ball->radius = ((MAX_RADIUS - MIN_RADIUS) * (ball->mass - MIN_MASS) / MAX_MASS) + MIN_RADIUS;
    /* TODO: cache mapped radius?
       map_length(wld_radius, &radius); */

    if (st->startAngles == ANGLES_ALL)
    {
        double angle = new_number_random(0.0, 360.0)*M_PI/180.0;
        ball->dx = cos(angle);
        ball->dy = sin(angle);
    }
    else
    {
        int whichAngle;

        whichAngle = new_number_random_int(0, st->numStartAngles - 1);
        /* printf("whichAngle=%d\n", whichAngle); */
        angle = st->startAngles[whichAngle]; 
        ball->dx = cos(M_PI/180.0 * angle);
        ball->dy = sin(M_PI/180.0 * angle);
    }

    ball->velocity = new_number_random(MIN_VELOCITY, MAX_VELOCITY);
    if (st->debian)
    {
        ball->velocity = MIN_VELOCITY + (MAX_VELOCITY-MIN_VELOCITY)/2.0;
    }
    else if (st->corners)
    {
        ball->velocity = MIN_VELOCITY + (MAX_VELOCITY-MIN_VELOCITY)/2.0;
    }

    {
        XGCValues gcv;
        unsigned long flags;
    
        flags = GCForeground | GCBackground;
        gcv.foreground = st->colors[ballIdx % st->ncolors].pixel;
        gcv.background = 0L;
        ball->gc = XCreateGC (st->dpy, st->b, flags, &gcv);
    }  

    ball->lastEvent.type = NONE;
    ball->lastEvent.obstacle = NULL;
    ball->lastEvent.ball = NULL;
    ball->lastEvent.ball2 = NULL;
}

static void
Ball_randomizePosition (struct state* st, Ball* ball)
{
    Vector2D position;
    position = new_vector_random (MINX + ball->radius, MAXX - ball->radius,
                                  MINY + ball->radius, MAXY - ball->radius);
    ball->x = position.x;
    ball->y = position.y;
}

static void
Ball_findPolygonalObstaclePathIntersection(Ball* ball, PolygonalObstacle* polyObs, Event* ev)
{
    int numberPoints = polyObs->numberPoints;
    Event* cornersAndEdges = calloc(numberPoints*2, sizeof(Event));
    int k;

    for (k = 0; k < numberPoints; k++)
    {
        int i = k;
        int j = (k == numberPoints - 1) ? 0 : k+1;
        Vector2D P1, P2, P1P2, v;
        number t;

        /* check for path intersection with corner k */
        cornersAndEdges[k].type = POLYGONAL_OBSTACLE_COLLISION;
        cornersAndEdges[k].polyObstacle = polyObs;
        cornersAndEdges[k].polyObsCollType = k;
        cornersAndEdges[k].ball = ball;
        cornersAndEdges[k].ball2 = NULL;
        cornersAndEdges[k].obstacle = NULL;
        /* do not re-collide with same obstacle corner with no event in
           between! */
        if (ball->lastEvent.type == POLYGONAL_OBSTACLE_COLLISION &&
            ball->lastEvent.polyObsCollType == k)
            cornersAndEdges[k].delta_t = INFINITY;
        else
            cornersAndEdges[k].delta_t =
                Ball_findPathPointIntersection(ball, 
                                               polyObs->points[k].x,
                                               polyObs->points[k].y);

        /* check for path intersection with edge (i,j) */
        P1 = polyObs->points[i];
        P2 = polyObs->points[j];
        
        cornersAndEdges[numberPoints + k].type = POLYGONAL_OBSTACLE_COLLISION;
        cornersAndEdges[numberPoints + k].polyObstacle = polyObs;
        cornersAndEdges[numberPoints + k].polyObsCollType = numberPoints + k;
        cornersAndEdges[numberPoints + k].ball = ball;
        cornersAndEdges[numberPoints + k].ball2 = NULL;
        cornersAndEdges[numberPoints + k].obstacle = NULL;

        /* do not re-collide with same obstacle edge with no event in
           between! */
        if (ball->lastEvent.type == POLYGONAL_OBSTACLE_COLLISION &&
            ball->lastEvent.polyObsCollType == numberPoints + k)
        {
            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
            continue;
        }

        /* 
           1. is b moving *towards* edge (i,j)?
           solve([bx+vx*t = p1x + u*(p2x-p1x), 
                  by+vy*t = p1y + u*(p2y-p1y)], [t,u]); 
        */
        t = -(ball->x*(P2.y-P1.y)+P1.x*(ball->y-P2.y)+(P1.y-ball->y)*P2.x)/
            (ball->dy*(-P2.x+P1.x) + (P2.y-P1.y)*ball->dx);
        if (t < 0.0) /* the ball is moving away from edge (i,j) */
        {
            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
        }
        else /* the ball is moving towards edge (i,j) */
        {
            Vector2D yAxis, b;
            number cos_alpha, alpha;

            P1P2 = Vector2D_sub(&P2, &P1);
            v.x = ball->dx;
            v.y = ball->dy;

            b.x = ball->x;
            b.y = ball->y;

            if (P1P2.y < 0.0)
            {
                yAxis.x = 0.0;
                yAxis.y = -1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                if (P1P2.x >= 0.0)
                { /* Quadrant I */
                    Vector2D_rotate(&v, alpha, False);
                    Vector2D_rotate(&P1P2, alpha, False);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, False);
                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
                    
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + ball->radius - b.x)/(v.x * ball->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - ball->radius - b.x)/(v.x * ball->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * ball->velocity * cornersAndEdges[numberPoints + k].delta_t;
                        if (y < fmin(P1.y, P2.y) || y > fmax(P1.y, P2.y))
                            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    }
                }
                else
                { /* Quadrant II */
                    Vector2D_rotate(&v, alpha, True);
                    Vector2D_rotate(&P1P2, alpha, True);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, True);

                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
           
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + ball->radius - b.x)/(v.x * ball->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - ball->radius - b.x)/(v.x * ball->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * ball->velocity * cornersAndEdges[numberPoints + k].delta_t;
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
                    Vector2D_rotate(&v, alpha, True);
                    Vector2D_rotate(&P1P2, alpha, True);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, True);
                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
                    
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + ball->radius - b.x)/(v.x * ball->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - ball->radius - b.x)/(v.x * ball->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * ball->velocity * cornersAndEdges[numberPoints + k].delta_t;
                        if (y < fmin(P1.y, P2.y) || y > fmax(P1.y, P2.y))
                            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    }
                }
                else
                { /* Quadrant III */
                    Vector2D_rotate(&v, alpha, False);
                    Vector2D_rotate(&P1P2, alpha, False);
                    Vector2D_rotateAroundPoint(&b, &P1, alpha, False);

                    /* recompute P2 based on rotated P1P2 */
                    P2.x = P1.x + P1P2.x;
                    P2.y = P1.y + P1P2.y;
                    
                    if (b.x > P2.x)
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x + ball->radius - b.x)/(v.x * ball->velocity);
                    else
                        cornersAndEdges[numberPoints + k].delta_t = (P2.x - ball->radius - b.x)/(v.x * ball->velocity);
                    if (cornersAndEdges[numberPoints + k].delta_t < 0)
                        cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    else
                    {
                        /* is the intersection outside of the extents of the edge? */
                        number y = b.y + v.y * ball->velocity * cornersAndEdges[numberPoints + k].delta_t;
                        if (y < fmin(P1.y, P2.y) || y > fmax(P1.y, P2.y))
                            cornersAndEdges[numberPoints + k].delta_t = INFINITY;
                    }
                }
            }
        }
    }
        
    *ev = Events_find_closest(cornersAndEdges, numberPoints*2);
    free(cornersAndEdges);
}

static void 
Ball_findWallIntersection(Ball* b, Event* ev)
{
    Event walls[4];
    number 
        bx = b->x + EPS * b->dx, 
        by = b->y + EPS * b->dy;

    /* future collision with left wall?
       (bx + b->dx*b->velocity*t = MINX + b->radius) */
    walls[0].type = LEFT_WALL_HIT;
    walls[0].ball = b;
    walls[0].ball2 = NULL;
    walls[0].delta_t = (MINX - bx + b->radius)/(b->dx * b->velocity);
    if (walls[0].delta_t < 0)
        walls[0].delta_t = INFINITY;

    /* future collision with right wall?
       (bx + b->dx*b->velocity*t = MAXX - b->radius) */
    walls[1].type = RIGHT_WALL_HIT;
    walls[1].ball = b;
    walls[1].ball2 = NULL;
    walls[1].delta_t = (MAXX - bx - b->radius)/(b->dx * b->velocity);
    if (walls[1].delta_t < 0)
        walls[1].delta_t = INFINITY;

    /* future collision with top wall?
       (by + b->dy*b->velocity*t = MINY + b->radius) */
    walls[2].type = TOP_WALL_HIT;
    walls[2].ball = b;
    walls[2].ball2 = NULL;
    walls[2].delta_t = (MINY - by + b->radius)/(b->dy * b->velocity);
    if (walls[2].delta_t < 0)
        walls[2].delta_t = INFINITY;
  
    /* future collision with bottom wall?
       (by + b->dy*b->velocity*t = MAXY - b->radius) */
    walls[3].type = BOTTOM_WALL_HIT;
    walls[3].ball = b;
    walls[3].ball2 = NULL;
    walls[3].delta_t = (MAXY - by - b->radius)/(b->dy * b->velocity);
    if (walls[3].delta_t < 0)
        walls[3].delta_t = INFINITY;

    /* 
       qsort(walls, 4, sizeof(Event), event_comp);
       *ev = walls[0]; 
       */

    *ev = Events_find_closest(walls, 4);
}

/*
  find out at which time the ball will first touch (cx,cy)
  created with maxima using this command:
  solve([(px+t*vx - cx)^2 + (py+t*vy -cy)^2 = r^2], t);
*/
static number
Ball_findPathPointIntersection(Ball* b, number cx, number cy)
{
    number r = b->radius, px = b->x, py = b->y,
        vx = b->dx * b->velocity, vy = b->dy * b->velocity;
    
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

static void
Ball_findObstacleIntersection(Ball* b, Obstacle* ob, Event* ev)
{
    int i;
    Vector2D center;
    Event obsColls[8];
    number 
        bx = b->x + EPS * b->dx, 
        by = b->y + EPS * b->dy,
        r = b->radius;
    number x, y;
    
    for (i = 0; i < 8; i++)
    {
        obsColls[i].type = OBSTACLE_COLLISION;
        obsColls[i].ball = b;
        obsColls[i].ball2 = NULL;
        obsColls[i].obstacle = ob;
    }

    /* future collision with left edge? */
    obsColls[0].obsCollType = LEFT_EDGE;
    obsColls[0].delta_t = (ob->x1 - r - bx)/(b->dx * b->velocity);
    if (obsColls[0].delta_t < 0)
        obsColls[0].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the left edge? */
        y = by + b->dy * b->velocity * obsColls[0].delta_t;
        if (y < ob->y1 || y > ob->y2)
            obsColls[0].delta_t = INFINITY;
    }

    /* future collision with right edge? */
    obsColls[1].obsCollType = RIGHT_EDGE;
    obsColls[1].delta_t = (ob->x2 + r - bx)/(b->dx * b->velocity);
    if (obsColls[1].delta_t < 0)
        obsColls[1].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the right edge? */
        y = by + b->dy * b->velocity * obsColls[1].delta_t;
        if (y < ob->y1 || y > ob->y2)
            obsColls[1].delta_t = INFINITY;
    }

    /* future collision with top edge? */
    obsColls[2].obsCollType = TOP_EDGE;
    obsColls[2].delta_t = (ob->y1 - r - by)/(b->dy * b->velocity);
    if (obsColls[2].delta_t < 0)
        obsColls[2].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the top edge? */
        x = bx + b->dx * b->velocity * obsColls[2].delta_t;
        if (x < ob->x1 || x > ob->x2)
            obsColls[2].delta_t =INFINITY;
    }

    /* future collision with bottom edge? */
    obsColls[3].obsCollType = BOTTOM_EDGE;
    obsColls[3].delta_t = (ob->y2 + r - by)/(b->dy * b->velocity);
    if (obsColls[3].delta_t < 0)
        obsColls[3].delta_t = INFINITY;
    else
    {
        /* is the intersection outside of the extents of the bottom edge? */
        x = bx + b->dx * b->velocity * obsColls[3].delta_t;
        if (x < ob->x1 || x > ob->x2)
            obsColls[3].delta_t =INFINITY;
    }

    center.x = (ob->x1 + ob->x2)/2.0;
    center.y = (ob->y1 + ob->y2)/2.0;

    /* future intersection with top left corner? */
    obsColls[4].obsCollType = TOPLEFT_CORNER;
    if (bx < center.x && by < center.y)
        obsColls[4].delta_t = Ball_findPathPointIntersection(b, ob->x1, ob->y1);
    else
        obsColls[4].delta_t = INFINITY;
    
    /* future intersection with top right corner? */
    obsColls[5].obsCollType = TOPRIGHT_CORNER;
    if (bx > center.x && by < center.y)
        obsColls[5].delta_t = Ball_findPathPointIntersection(b, ob->x2, ob->y1);
    else
        obsColls[5].delta_t = INFINITY;

    /* future intersection with bottom left corner? */
    obsColls[6].obsCollType = BOTTOMLEFT_CORNER;
    if (bx < center.x && by > center.y)
        obsColls[6].delta_t = Ball_findPathPointIntersection(b, ob->x1, ob->y2);
    else
        obsColls[6].delta_t = INFINITY;

    /* future intersection with bottom right corner? */
    obsColls[7].obsCollType = BOTTOMRIGHT_CORNER;
    if (bx > center.x && by > center.y)
        obsColls[7].delta_t = Ball_findPathPointIntersection(b, ob->x2, ob->y2);
    else
        obsColls[7].delta_t = INFINITY;

    /*
      qsort(obsColls, 8, sizeof(Event), event_comp);
      *ev = obsColls[0];
      */
    *ev = Events_find_closest(obsColls, 8);
}

/*
  This was solved via maxima (GPL) with the following command:
  solve([(px2+t*vx2 - px1-t*vx1)^2 + (py2 + t*vy2 - py1-t*vy1)^2 =
  (r1+r2)^2], t);
*/
static number
Balls_findPathIntersection(Ball* b1, Ball* b2)
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

static Bool
Ball_collision_check(Ball* b1, Ball* b2)
{
    double r1pr2, dx, dy;
    r1pr2 = b1->radius + b2->radius;
    dx = b2->x - b1->x;
    dy = b2->y - b1->y;
    if (dx*dx + dy*dy <= r1pr2*r1pr2)
        return True;
    else
        return False;
}

static Bool
Ball_collision_with_other(struct state* st, Ball* b)
{
    int i;
    for (i = 0; i < st->count; i++)
    {
        Ball* other = &st->balls[i];
        if (other == b)
            continue;
        if (Ball_collision_check(b, other))
            return True;
    }
    return False;
}

static void
Ball_setDirectionForTarget(Ball* b, number tx, number ty)

{
    Vector2D dir;
    dir.x = tx - b->x;
    dir.y = ty - b->y;
    Vector2D_normalize(&dir);
    b->dx = dir.x;
    b->dy = dir.y;
}

static Bool
Ball_PointInside(Ball* ball, number x, number y)
{
    number dx, dy;

    dx = ball->x - x;
    dy = ball->y - y;
    if (dx*dx + dy*dy <= ball->radius*ball->radius)
        return True;
    else
        return False;
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
static Bool
Ball_boundedEdge_collision_check(Ball* ball, Vector2D P1, Vector2D P2)
{
    Vector2D H, shortestPath, diffP2P1, diffbP1;
    number u;

    diffP2P1.x = P2.x - P1.x;
    diffP2P1.y = P2.y - P1.y;
    diffbP1.x = ball->x-P1.x;
    diffbP1.y = ball->y-P1.y;
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
    
    shortestPath.x = H.x - ball->x;
    shortestPath.y = H.y - ball->y;
    return Vector2D_length(&shortestPath) <= ball->radius;
}

/*
  Use the ray casting algorithm:
    http://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
  see how many polygon edges a horizontal line starting from p=(px,py) crosses.
  if and only if this is odd, the point lies inside the polygon.
 */
static Bool
PointInPolygon(number px, number py, PolygonalObstacle* polyObs)
{
    int k, numberIntersections = 0;

    for (k = 0; k < polyObs->numberPoints; k++)
    {
        Vector2D pi = polyObs->points[k];
        Vector2D pj = (k == polyObs->numberPoints - 1) ? polyObs->points[0] : polyObs->points[k+1];
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
static Bool
Ball_PolygonalObstacle_collision_check(Ball* ball, PolygonalObstacle* polyObs)
{
    int k;

    /* 1. is the ball's center contained in the polygon? */
    if (PointInPolygon(ball->x, ball->y, polyObs))
    {
        return True;
    }
    
    /* 2. does the ball intersect any edge? */
    for (k = 0; k < polyObs->numberPoints; k++)
    {
        int i = k;
        int j = (k == polyObs->numberPoints - 1) ? 0 : k+1;
        if (Ball_boundedEdge_collision_check(ball,
                                             polyObs->points[i],
                                             polyObs->points[j]))
        {
            return True;
        }
    }
    return False;
}


static Bool
Ball_Obstacle_collision_check(Ball* ball, Obstacle* obstacle,
                              ObstacleCollisionType* type)
{
    /* 1. center of ball is inside rectangle */
    if (ball->x >= obstacle->x1 && ball->x <= obstacle->x2 &&
        ball->y >= obstacle->y1 && ball->y <= obstacle->y2)
    {
        *type = CENTER_INSIDE;
        return True;
    }

    /* 2. ball intersects with edge of rectangle (in simple way) */
    /* 2.1 left edge */
    if (ball->x < obstacle->x1 &&
        ball->y >= obstacle->y1 && ball->y <= obstacle->y2 &&
        ball->x + ball->radius >= obstacle->x1)
    {
        *type = LEFT_EDGE;
        return True;
    }
    /* 2.2 right edge */
    if (ball->x > obstacle->x2 &&
        ball->y >= obstacle->y1 && ball->y <= obstacle->y2 &&
        ball->x - ball->radius <= obstacle->x2)
    {
        *type = RIGHT_EDGE;
        return True;
    }
    /* 2.3 top edge */
    if (ball->y < obstacle->y1 &&
        ball->x >= obstacle->x1 && ball->x <= obstacle->x2 &&
        ball->y + ball->radius >= obstacle->y1)
    {
        *type = TOP_EDGE;
        return True;
    }
    /* 2.4 bottom edge */
    if (ball->y > obstacle->y2 &&
        ball->x >= obstacle->x1 && ball->x <= obstacle->x2 &&
        ball->y - ball->radius <= obstacle->y2)
    {
        *type = BOTTOM_EDGE;
        return True;
    }

    /* 3. ball intersects with a corner of rectangle 
       (these intersections are not found by 2.) */
    if (Ball_PointInside(ball, obstacle->x1, obstacle->y1))
    {
        *type = TOPLEFT_CORNER;
        return True;
    }
    if (Ball_PointInside(ball, obstacle->x2, obstacle->y1))
    {
        *type = TOPRIGHT_CORNER;
        return True;
    }
    if (Ball_PointInside(ball, obstacle->x1, obstacle->y2))
    {
        *type = BOTTOMLEFT_CORNER;
        return True;
    }
    if (Ball_PointInside(ball, obstacle->x2, obstacle->y2))
    {
        *type = BOTTOMRIGHT_CORNER;
        return True;
    }

    return False;
}

static void
PolygonalObstacle_draw(struct state* st, Drawable w, PolygonalObstacle* polyObs)
{
    XPoint* points = calloc(polyObs->numberPoints, sizeof(XPoint));
    int i;

    for (i = 0; i < polyObs->numberPoints; i++)
    {
        int x,y;
        map2screen(st, polyObs->points[i].x, polyObs->points[i].y, &x, &y);
        points[i].x = x;
        points[i].y = y;
    }
    XFillPolygon(st->dpy, w, st->obstacleGC, points, 
                 polyObs->numberPoints, Complex, CoordModeOrigin);
    free(points);
}

static void
Obstacle_draw(struct state* st, Drawable w, Obstacle* obstacle)
{
    int x1, y1, x2, y2;
  
    map2screen(st, obstacle->x1, obstacle->y1, &x1, &y1);
    map2screen(st, obstacle->x2, obstacle->y2, &x2, &y2);
  
    /*
      XFillRectangle(st->dpy, w, st->obstacleGC,
      x1, y1, x2-x1, y2-y1);
    */
    XmuFillRoundedRectangle(st->dpy, w, st->obstacleGC,
                            x1, y1, x2-x1, y2-y1,
                            1, 1);
}

static void
Ball_draw (struct state* st, Drawable w, Ball *ball)
{
    int deviceX, deviceY, radiusX, radiusY;

    map_length_x(st, ball->radius, &radiusX);
    map_length_y(st, ball->radius, &radiusY);
    map2screen(st, ball->x, ball->y,
               &deviceX, &deviceY);

    XFillArc (st->dpy, w, ball->gc,
              deviceX - radiusX,
              deviceY - radiusY,
              radiusX*2, radiusY*2,
              0, 360*64);
    if (st->showFutureCollisions &&
        st->nextEvent.type == BALL_COLLISION &&
        (st->nextEvent.ball == ball ||
         st->nextEvent.ball2 == ball))
    {
        Ball* otherBall = (st->nextEvent.ball == ball) ?
            st->nextEvent.ball2 : st->nextEvent.ball;
        int deviceX2, deviceY2;
        Vector2D coll, collOther, line_of_sight_at_coll;

        XDrawArc (st->dpy, w, st->highlightGC,
                  deviceX - radiusX,
                  deviceY - radiusY,
                  radiusX*2, radiusY*2,
                  0, 360*64);

        /* get the center of b1 at collision time */
        coll.x = ball->x +
            ball->dx * ball->velocity * st->nextEvent.delta_t;
        coll.y = ball->y +
            ball->dy * ball->velocity * st->nextEvent.delta_t;
        /* get the center of b2 at collision time */
        collOther.x = otherBall->x +
            otherBall->dx * otherBall->velocity * st->nextEvent.delta_t;
        collOther.y = otherBall->y +
            otherBall->dy * otherBall->velocity * st->nextEvent.delta_t;

        /* get the point where the two balls will touch */
        line_of_sight_at_coll = Vector2D_sub(&collOther, &coll);
        Vector2D_normalize(&line_of_sight_at_coll);
        coll.x += line_of_sight_at_coll.x * ball->radius;
        coll.y += line_of_sight_at_coll.y * ball->radius;
      
        map2screen(st, coll.x, coll.y, &deviceX2, &deviceY2);
        /* XFillRectangle(st->dpy, w, st->highlightGC, 
           deviceX2 - 3, deviceY2 - 3, 6, 6); */
        XDrawLine(st->dpy, w, st->highlightGC, 
                  deviceX2 - 4, deviceY2 + 4, deviceX2 + 4, deviceY2 - 4);
        XDrawLine(st->dpy, w, st->highlightGC, 
                  deviceX2 - 4, deviceY2 - 4, deviceX2 + 4, deviceY2 + 4);

        /*
          map2screen(st, otherBall->x, otherBall->y, &deviceX2, &deviceY2);
          XDrawLine(st->dpy, w, st->velocityVectorGC,
          deviceX, deviceY, deviceX2, deviceY2);
        */
    }

    if (st->showVelocityVectors) 
    {
        int velocityVectorX, velocityVectorY;
        Vector2D velocityVector;

        velocityVector.x = ball->dx;
        velocityVector.y = ball->dy;
        Vector2D_multScalar(&velocityVector, ball->radius / MAX_VELOCITY);

        velocityVector.x += ball->x;
        velocityVector.y += ball->y;

        map2screen(st, velocityVector.x, velocityVector.y,
                   &velocityVectorX, &velocityVectorY);

        XDrawLine(st->dpy, w, st->velocityVectorGC,
                  deviceX, deviceY,
                  velocityVectorX, velocityVectorY);
    }
}

static void
Ball_doRotate(Ball* b, number degrees, Bool clockWise)
{
    Vector2D dirVec;
  
    dirVec.x = b->dx;
    dirVec.y = b->dy;
    
    Vector2D_rotate(&dirVec, degrees, clockWise);
    
    b->dx = dirVec.x;
    b->dy = dirVec.y;
}

static Bool
Ball_doPolygonalObstacleCollision(struct state* st, Ball* b, PolygonalObstacle* polyObs,
                                  int polyObsCollType)
{
    Vector2D center_of_ball, corner, virtualWall;
    number cosAngle, angle;

    center_of_ball.x = b->x;
    center_of_ball.y = b->y;
    
    if (polyObsCollType < polyObs->numberPoints) /* corner collision */
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
                Vector2D_rotate(&virtualWall, 90.0, False);
                /* determine angle to y axis (scalar product between virtualWall and
                   obstacle edge (0,-1)) */
                cosAngle = -virtualWall.y;
                angle = acos(cosAngle) * 180.0/M_PI;

                /* 1. rotate the setup so that the virtual wall is parallel to y
                   axis */
                Ball_doRotate(b, angle, False);
                /* 2. treat it like a right wall hit */
                b->dx = -b->dx;
                /* 3. rotate back */
                Ball_doRotate(b, angle, True);
            }
            else
            {   /* ball is top right relative to corner */

                /* calculate the normal to the virtual wall */
                virtualWall = Vector2D_sub(&center_of_ball, &corner);
                Vector2D_normalize(&virtualWall);
                /* convert normal to virtual wall */
                Vector2D_rotate(&virtualWall, 90.0, False);
                /* determine angle to x axis (scalar product between virtualWall and
                   obstacle edge (-1, 0)) */
                cosAngle = -virtualWall.x;
                angle = acos(cosAngle) * 180.0/M_PI;

                /* 1. rotate the setup so that the virtual wall is parallel to x
                   axis */
                Ball_doRotate(b, angle, False);
                /* 2. treat it like a bottom wall hit */
                b->dy = -b->dy;
                /* 3. rotate back */
                Ball_doRotate(b, angle, True);
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
                Vector2D_rotate(&virtualWall, 90.0, False);
                /* determine angle to x axis (scalar product between virtualWall and
                   obstacle edge (1,0)) */
                cosAngle = virtualWall.x;
                angle = acos(cosAngle) * 180.0/M_PI;

                /* 1. rotate the setup so that the virtual wall is parallel to x
                   axis */
                Ball_doRotate(b, angle, False);
                /* 2. treat it like a top wall hit */
                b->dy = -b->dy;
                /* 3. rotate back */
                Ball_doRotate(b, angle, True);

            }
            else
            {   /* ball is top left relative to corner */

                /* calculate the normal to the virtual wall */
                virtualWall = Vector2D_sub(&center_of_ball, &corner);
                Vector2D_normalize(&virtualWall);
                /* convert normal to virtual wall */
                Vector2D_rotate(&virtualWall, 90.0, False);
                /* determine angle to x axis (scalar product between virtualWall and
                   obstacle edge (0,1)) */
                cosAngle = virtualWall.y;
                angle = acos(cosAngle) * 180.0/M_PI;
                
                /* 1. rotate the setup so that the virtual wall is parallel to y
                   axis */
                Ball_doRotate(b, angle, False);
                /* 2. treat it like a top wall hit */
                b->dx = -b->dx;
                /* 3. rotate back */
                Ball_doRotate(b, angle, True);
            }
        }
    }
    else /* edge collision */
    {
        Vector2D P1, P2, P1P2, v, yAxis;
        int k = polyObsCollType - polyObs->numberPoints;
        number cos_alpha, alpha;

        P1 = polyObs->points[k];
        P2 = polyObs->points[k == polyObs->numberPoints - 1 ? 0 : k+1];

        P1P2 = Vector2D_sub(&P2, &P1);
        v.x = b->dx;
        v.y = b->dy;
        
        if (P1P2.y < 0.0)
        {
            if (P1P2.x >= 0.0)
            { /* quadrant I */
                yAxis.x = 0.0;
                yAxis.y = -1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                Vector2D_rotate(&v, alpha, False);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, True);
                b->dx = v.x;
                b->dy = v.y;
            }
            else
            { /* Quadrant II */
                yAxis.x = 0.0;
                yAxis.y = -1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                Vector2D_rotate(&v, alpha, True);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, False);
                b->dx = v.x;
                b->dy = v.y;
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
                Vector2D_rotate(&v, alpha, True);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, False);
                b->dx = v.x;
                b->dy = v.y;
            }
            else
            { /* Quadrant III */
                yAxis.x = 0.0;
                yAxis.y = 1.0;
                cos_alpha = Vector2D_scalarProduct(&P1P2, &yAxis)/Vector2D_length(&P1P2);
                alpha = acos(cos_alpha)/M_PI * 180.0;
                Vector2D_rotate(&v, alpha, False);
                v.x = -v.x;
                Vector2D_rotate(&v, alpha, True);
                b->dx = v.x;
                b->dy = v.y;
            }
        }
    }

    b->lastEvent.type = POLYGONAL_OBSTACLE_COLLISION;
    b->lastEvent.ball = b;
    b->lastEvent.ball2 = NULL;
    b->lastEvent.polyObstacle = polyObs;
    b->lastEvent.polyObsCollType = polyObsCollType;

    return True;
}

static Bool
Ball_doObstacleCollision(struct state* st, Ball* b, Obstacle* obstacle,
                         ObstacleCollisionType obsCollType)
{
     switch(obsCollType)
    {
        Vector2D center_of_ball, corner, virtualWall;
        number cosAngle, angle;
      
    case LEFT_EDGE:
    case RIGHT_EDGE:
        b->dx = -b->dx;
        break;
    case TOP_EDGE:
    case BOTTOM_EDGE:
        b->dy = -b->dy;
        break;
    case TOPLEFT_CORNER:
#ifdef LOGGING
        printf("Ball_doObstacleCollision(): TOPLEFT_CORNER\n");
#endif
        center_of_ball.x = b->x;
        center_of_ball.y = b->y;
        corner.x = obstacle->x1;
        corner.y = obstacle->y1;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, False);
        /* determine angle to x axis (scalar product between virtualWall and
           obstacle edge (0,1)) */
        cosAngle = virtualWall.y;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to y
           axis */
        Ball_doRotate(b, angle, False);
        /* 2. treat it like a top wall hit */
        b->dx = -b->dx;
        /* 3. rotate back */
        Ball_doRotate(b, angle, True);

        break;
    case TOPRIGHT_CORNER:
#ifdef LOGGING
        printf("Ball_doObstacleCollision(): TOPRIGHT_CORNER\n");
#endif

        center_of_ball.x = b->x;
        center_of_ball.y = b->y;
        corner.x = obstacle->x2;
        corner.y = obstacle->y1;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, False);
        /* determine angle to x axis (scalar product between virtualWall and
           obstacle edge (-1, 0)) */
        cosAngle = -virtualWall.x;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to x
           axis */
        Ball_doRotate(b, angle, False);
        /* 2. treat it like a bottom wall hit */
        b->dy = -b->dy;
        /* 3. rotate back */
        Ball_doRotate(b, angle, True);
        break;

    case BOTTOMLEFT_CORNER:
#ifdef LOGGING
        printf("Ball_doObstacleCollision(): BOTTOMLEFT_CORNER\n");
#endif

        center_of_ball.x = b->x;
        center_of_ball.y = b->y;
        corner.x = obstacle->x1;
        corner.y = obstacle->y2;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, False);
        /* determine angle to x axis (scalar product between virtualWall and
           obstacle edge (1,0)) */
        cosAngle = virtualWall.x;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to x
           axis */
        Ball_doRotate(b, angle, False);
        /* 2. treat it like a top wall hit */
        b->dy = -b->dy;
        /* 3. rotate back */
        Ball_doRotate(b, angle, True);
        break;

    case BOTTOMRIGHT_CORNER:
#ifdef LOGGING
        printf("Ball_doObstacleCollision(): BOTTOMRIGHT_CORNER\n");
#endif

        center_of_ball.x = b->x;
        center_of_ball.y = b->y;
        corner.x = obstacle->x2;
        corner.y = obstacle->y2;
      
        /* calculate the normal to the virtual wall */
        virtualWall = Vector2D_sub(&center_of_ball, &corner);
        Vector2D_normalize(&virtualWall);
        /* convert normal to virtual wall */
        Vector2D_rotate(&virtualWall, 90.0, False);
        /* determine angle to y axis (scalar product between virtualWall and
           obstacle edge (0,-1)) */
        cosAngle = -virtualWall.y;
        angle = acos(cosAngle) * 180.0/M_PI;

        /* 1. rotate the setup so that the virtual wall is parallel to y
           axis */
        Ball_doRotate(b, angle, False);
        /* 2. treat it like a right wall hit */
        b->dx = -b->dx;
        /* 3. rotate back */
        Ball_doRotate(b, angle, True);
        break;

    case CENTER_INSIDE:
        printf("Warning: last obstacle collision type is CENTER_INSIDE. Need smaller step size?\n");
        abort();
        break;
    }

    b->lastEvent.type = OBSTACLE_COLLISION;
    b->lastEvent.ball = b;
    b->lastEvent.ball2 = NULL;
    b->lastEvent.obstacle = obstacle;
    b->lastEvent.obsCollType = obsCollType;

    return True;
}

static Bool
Ball_doCollision(Ball* b1, Ball* b2)
{
    Vector2D position_b1, position_b2, line_of_sight, b1_velocity_before, b2_velocity_before, b1_velocity_after, b2_velocity_after;
    double b1_velocity_in_line_of_sight, b2_velocity_in_line_of_sight, b1_velocity_in_line_of_sight_before, b2_velocity_in_line_of_sight_before, b1_change_in_line_of_sight_velocity, b2_change_in_line_of_sight_velocity;

#ifdef LOGGING
    printf("Ball_doCollision()\n");
#endif
    /*
      if (b1->lastEvent.type == BALL_COLLISION &&
      b1->lastEvent.otherBall == b2 &&
      b2->lastEvent.type == BALL_COLLISION &&
      b2->lastEvent.otherBall == b1)
      return False;
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

    b1->lastEvent.type = BALL_COLLISION;
    b1->lastEvent.ball = b1;
    b1->lastEvent.ball2 = b2;
    b2->lastEvent.type = BALL_COLLISION;
    b2->lastEvent.ball = b2;
    b2->lastEvent.ball2 = b1;
  
    /*
      b1->x += b1->dx * b1->velocity * SINGLE_TIME_STEP;
      b1->y += b1->dy * b1->velocity * SINGLE_TIME_STEP;
      b2->x += b2->dx * b2->velocity * SINGLE_TIME_STEP;
      b2->y += b2->dy * b2->velocity * SINGLE_TIME_STEP;
    */

    return True;
}

static Bool
Balls_move(struct state* st, number* delta_t)
{
    int i, j, nBalls;
    Ball* balls;
    Ball* cball;
    number this_delta_t;
    time_t timeDelta, currentTime;

    /* return False;*/

#ifdef LOGGING
    printf("\n\nBalls_move(%lf)\n", *delta_t);
#endif

    balls = st->balls;
    nBalls = st->count;

    currentTime = time(NULL);
    timeDelta = currentTime - st->startTime;

    if (st->zombies && st->lastZombieTime != currentTime &&
        timeDelta > 0 && timeDelta % 10 == 0)
    {
        int angle = new_number_random_int(-1,7)*45;
        
        for (i = 0; i < nBalls; i++)
        {
            if (angle == -45)
                Ball_setDirectionForTarget(&balls[i], 0.0, 0.0);
            else
            {
                balls[i].dx = cos(angle/180.0*M_PI);
                balls[i].dy = sin(angle/180.0*M_PI);
            }
        }
        st->lastZombieTime = currentTime; 
        
        /* make sure that events are re-calculated */
        st->nextEvent.delta_t = INFINITY;
        st->nextEvent.type = NONE;

        /* make sure that all collisions are possible again */
        for (i = 0; i < nBalls; i++)
        {
            balls[i].lastEvent.type = NONE;
        }
    }

    if (st->nextEvent.delta_t < *delta_t)
    {
        this_delta_t = st->nextEvent.delta_t;
        st->nextEvent.delta_t = 0.0;
        /* set remaining delta_t */
        *delta_t -= this_delta_t;
    }
    else
    {
        this_delta_t = *delta_t;
        st->nextEvent.delta_t -= this_delta_t;
        /* set remaining delta_t */
        *delta_t = 0.0;
    }

#ifdef LOGGING
    printf("this_delta_t=%.5lf, *delta_t=%lf\n", this_delta_t, *delta_t);
#endif

    /* move balls for delta_t timesteps */
    for (i = 0; i < nBalls; i++)
    {
        cball = &balls[i];
        cball->x += cball->dx * cball->velocity * this_delta_t;
        cball->y += cball->dy * cball->velocity * this_delta_t;
    }

    /* execute nextEvent if it's time */
    if (st->nextEvent.delta_t == 0.0)
    {
        Ball* b = st->nextEvent.ball;
        Ball* b2 = st->nextEvent.ball2;
      
        switch (st->nextEvent.type)
        {
        case LEFT_WALL_HIT:
        case RIGHT_WALL_HIT:
            b->dx = -b->dx;
            b->lastEvent = st->nextEvent;
            break;
          
        case TOP_WALL_HIT:
        case BOTTOM_WALL_HIT:
            b->dy = -b->dy;
            b->lastEvent = st->nextEvent;
            break;
          
        case BALL_COLLISION:
            Ball_doCollision(b, b2);
            break;

        case OBSTACLE_COLLISION:
            Ball_doObstacleCollision(st, b, st->nextEvent.obstacle,
                                     st->nextEvent.obsCollType);
            break;

        case POLYGONAL_OBSTACLE_COLLISION:
            Ball_doPolygonalObstacleCollision(st, b, st->nextEvent.polyObstacle, st->nextEvent.polyObsCollType);
            break;

        case NONE:
            printf("st->nextEvent.type == NONE!\n");
            abort();
            break;
        }
    }

    if (st->nextEvent.delta_t == 0.0 || st->nextEvent.type == NONE)
    {
        /* find the next closest event */
        st->nextEvent.delta_t = INFINITY;
        st->nextEvent.type = NONE;

        /* wall or obstacle collision */
        for (i = 0; i < nBalls; i++)
        {
            Event ev;
            cball = &balls[i];

            /* cball-wall collision? */
            Ball_findWallIntersection(cball, &ev);
       
            /* do not collide with wall twice with no event in between! */
            if (cball->lastEvent.type == ev.type)
                continue;
       
            if (ev.delta_t < st->nextEvent.delta_t)
            {
                st->nextEvent = ev;
            }

#ifdef EVENT_LOGGING
            printf("Checking for path intersections with obstacles...");
#endif
            for (j = 0; j < st->nObstacles; j++)
            {
                /* do not re-collide with same obstacle with no event in
                   between! */
                if (cball->lastEvent.type == OBSTACLE_COLLISION &&
                    cball->lastEvent.obstacle == &st->obstacles[j])
                    continue;

                /* cball-obstacle_j collision? */
                Ball_findObstacleIntersection(cball, &st->obstacles[j],
                                              &ev);
                if (ev.delta_t < st->nextEvent.delta_t)
                {
                    st->nextEvent = ev;
                }
            }
            
#ifdef EVENT_LOGGING
            printf("Checking for path intersections with poly obstacles...");
#endif
            for (j = 0; j < st->nPolyObstacles; j++)
            {
                Ball_findPolygonalObstaclePathIntersection(cball, &st->polyObstacles[j], &ev);

                if (ev.delta_t < st->nextEvent.delta_t)
                {
                    st->nextEvent = ev;
                }
            }
        }

#ifdef EVENT_LOGGING
        printf("found best wall intersection:\n");
        Event_print(st->nextEvent);
#endif
  
        /* ball collision */
        for (i = 0; i < nBalls; i++)
        {
            Ball* b1 = &balls[i];

#ifdef EVENT_LOGGING
            printf("Ball %d: last Event:\n", i);
            printf("\t");
            Event_print(b1->lastEvent);
#endif

            for (j = 0; j < i; j++)
            {
                Ball* b2 = &balls[j];
                number delta_t;

                if (b1->lastEvent.type == BALL_COLLISION &&
                    b1->lastEvent.ball2 == b2 &&
                    b2->lastEvent.type == BALL_COLLISION &&
                    b2->lastEvent.ball2 == b1)
                {
#ifdef EVENT_LOGGING
                    printf("skipping collision(%d,%d)\n", i, j);
#endif
                    continue;
                }
          
                delta_t = Balls_findPathIntersection(b1, b2);
#ifdef EVENT_LOGGING
                /*
                  printf("found ball collision(%x,%x) @%f\n",
                  (unsigned int)b1, (unsigned int)b2, delta_t);
                */
#endif
          
                if (delta_t < st->nextEvent.delta_t)
                {
                    st->nextEvent.type = BALL_COLLISION;
                    st->nextEvent.delta_t = delta_t;
                    st->nextEvent.ball = b1;
                    st->nextEvent.ball2 = b2;
#ifdef EVENT_LOGGING
                    printf("found better ball collision:\n");
                    Event_print(st->nextEvent);
#endif
                }
            }
        }
#ifdef EVENT_LOGGING
        printf("best event:\n");
        Event_print(st->nextEvent);
#endif
    }

    return *delta_t > 0.0;
}

static Bool
Balls_placeOnCorner(struct state* st, int* ballCounter,
                    number cornerX, number cornerY,
                    int numberBalls)
{
    int i;
    Vector2D pos;

    pos.x = cornerX;
    pos.y = cornerY;

    for (i = 0; i < numberBalls; i++)
    {
        st->balls[*ballCounter].x = pos.x;
        st->balls[*ballCounter].y = pos.y;
        Ball_setDirectionForTarget(&st->balls[*ballCounter], 0.0, 0.0);
          
        if (Ball_collision_with_other(st, &st->balls[*ballCounter]))
            return True;
        pos.x += st->balls[*ballCounter].dx * 2*st->max_radius;
        pos.y += st->balls[*ballCounter].dy * 2*st->max_radius;
        (*ballCounter)++;
    }
    return False;
}

static void
Balls_placeOnCorners(struct state* st)
{
    int numberBallsPerCorner = st->count / 8;
    int ballCounter = 0;

    /* left */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MINX + SINGLE_STEP + st->max_radius,
                            MINY + (MAXY-MINY)/2.0,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }

    /* top left */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MINX + SINGLE_STEP + st->max_radius,
                            MINY + SINGLE_STEP + st->max_radius,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }

    /* top */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MINX + (MAXX-MINX)/2.0,
                            MINY + SINGLE_STEP + st->max_radius,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }

    /* top right */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MAXX - SINGLE_STEP - st->max_radius,
                            MINY + SINGLE_STEP + st->max_radius,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }

    /* right */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MAXX - SINGLE_STEP - st->max_radius,
                            MINY + (MAXY-MINY)/2.0,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }

    /* bottom right */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MAXX - SINGLE_STEP - st->max_radius,
                            MAXY - SINGLE_STEP - st->max_radius,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }
      
    /* bottom */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MINX + (MAXX-MINX)/2.0,
                            MAXY - SINGLE_STEP - st->max_radius,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }

    /* bottom left */
    if (Balls_placeOnCorner(st, &ballCounter, 
                            MINX + SINGLE_STEP + st->max_radius,
                            MAXY - SINGLE_STEP - st->max_radius,
                            numberBallsPerCorner))
    {
        /* collision - stop placing balls */
        st->count = ballCounter;
        return;
    }
}

        /* rhombus:
        st->nPolyObstacles = 1;
        st->polyObstacles = calloc(st->nPolyObstacles, sizeof(PolygonalObstacle));
        st->polyObstacles[0].numberPoints = 4;
        st->polyObstacles[0].points = calloc(st->polyObstacles[0].numberPoints,
                                             sizeof(Vector2D));
        st->polyObstacles[0].points[0].x = 0;
        st->polyObstacles[0].points[0].y = -50;
        st->polyObstacles[0].points[1].x = 50;
        st->polyObstacles[0].points[1].y = 0;
        st->polyObstacles[0].points[2].x = 0;
        st->polyObstacles[0].points[2].y = 50;
        st->polyObstacles[0].points[3].x = -50;
        st->polyObstacles[0].points[3].y = 0;
        */
static void
createStarPolygon(PolygonalObstacle* polyObs, int numberPoints,
                  number x, number y, number radius1, number radius2)
{
    number radPerPoint, angle;
    int i;
        
    polyObs->numberPoints = numberPoints;
    polyObs->points = calloc(polyObs->numberPoints, sizeof(Vector2D));
    radPerPoint = 2*M_PI / polyObs->numberPoints;
    angle = new_number_random(0.0, 2*M_PI);
    for (i = 0; i < polyObs->numberPoints; i++)
    {
        polyObs->points[i].x = x +
            cos(angle) * (i % 2 == 0 ? radius1 : radius2);
        polyObs->points[i].y = y +
            sin(angle) * (i % 2 == 0 ? radius1 : radius2);
        angle += radPerPoint;
    }
}

static void *
ball2d_init (Display *dpy, Window window)
{
    struct state *st = (struct state *) calloc (1, sizeof(*st));
    XGCValues gcv;
    int i, j;
    Bool conflict, onedee;
    char* startAngles;

    st->iteration_number = 0;
    st->startTime = time(NULL);
    st->lastZombieTime = 0;
    st->dpy = dpy;

    st->window = window;
    XGetWindowAttributes (st->dpy, st->window, &st->xgwa);
    recomputeYRange(st);

    st->tiny = get_boolean_resource(st->dpy, "tiny", "Boolean");
    st->startGrid = get_boolean_resource(st->dpy, "startGrid", "Boolean");
    st->corners = get_boolean_resource(st->dpy, "corners", "Boolean");
    st->debian = get_boolean_resource(st->dpy, "debian", "Boolean");
    st->slowStart = get_boolean_resource(st->dpy, "slowStart", "Boolean");
    st->count = get_integer_resource (st->dpy, "count", "Integer");
    st->ncolors = get_integer_resource (st->dpy, "ncolors", "Integer");
    st->delay = get_integer_resource (st->dpy, "delay", "Integer");
    st->dbuf = get_boolean_resource (st->dpy, "doubleBuffer", "Boolean");
    onedee = get_boolean_resource(st->dpy, "1D", "Boolean");
    st->zombies = get_boolean_resource(st->dpy, "zombies", "Boolean");
    st->showFutureCollisions = get_boolean_resource(st->dpy, "showFutureCollisions", "Boolean");
    if (get_boolean_resource(st->dpy, "obstacles", "Boolean") &&
        !st->corners)
    {
        st->nObstacles = 2;
        st->obstacles = (Obstacle*) calloc(st->nObstacles, sizeof(Obstacle));

        st->obstacles[0].x1 = -50.0;
        st->obstacles[0].y1 = -40.0;
        st->obstacles[0].x2 = -40.0;
        st->obstacles[0].y2 =  20.0;

        st->obstacles[1].x1 =  30.0;
        st->obstacles[1].y1 = -10.0;
        st->obstacles[1].x2 =  50.0;
        st->obstacles[1].y2 =  10.0;
    }
    else if (get_boolean_resource(st->dpy, "obstacles2", "Boolean") &&
             !st->corners)
    {
        double obstacleSize = 10.0;
        int numCols = 5, numRows = 3;
        double xSpacing = (MAXX-MINX)/(numCols+1);
        double ySpacing = (MAXY-MINY)/(numRows+1);
        int obstacleCounter = 0;

        st->startGrid = False;
        st->nObstacles = numRows * numCols;
        st->obstacles = (Obstacle*) calloc(st->nObstacles, sizeof(Obstacle));

        for (i = 1; i <= numRows; i++)
        {
            for (j = 1; j <= numCols; j++)
            {
                double x = MINX + j*xSpacing;
                double y = MINY + i*ySpacing;

                st->obstacles[obstacleCounter].x1 = x-obstacleSize/2;
                st->obstacles[obstacleCounter].y1 = y-obstacleSize/2;
                st->obstacles[obstacleCounter].x2 = x+obstacleSize/2;
                st->obstacles[obstacleCounter].y2 = y+obstacleSize/2;
              
                obstacleCounter++;
            }
        }

    }
    else
    {
        st->nObstacles = 0;
        st->obstacles = NULL;
    }

    if (get_boolean_resource(st->dpy, "star", "Boolean"))
    {
        st->nPolyObstacles = 1;
        st->polyObstacles = calloc(st->nPolyObstacles, sizeof(PolygonalObstacle));
        createStarPolygon(&st->polyObstacles[0], 24, 0, 0, 70, 10);
    }
    else if (get_boolean_resource(st->dpy, "stars", "Boolean"))
    {
        double obstacleSize = 30.0;
        int numCols = 5, numRows = 3;
        double xSpacing = (MAXX-MINX)/(numCols+1);
        double ySpacing = (MAXY-MINY)/(numRows+1);
        int obstacleCounter = 0;

        st->startGrid = False;
        st->nPolyObstacles = numRows * numCols;
        st->polyObstacles = (PolygonalObstacle*) calloc(st->nPolyObstacles, sizeof(PolygonalObstacle));

        for (i = 1; i <= numRows; i++)
        {
            for (j = 1; j <= numCols; j++)
            {
                double x = MINX + j*xSpacing;
                double y = MINY + i*ySpacing;

                createStarPolygon(&st->polyObstacles[obstacleCounter],
                                  12, x, y,
                                  obstacleSize/4.0,
                                  obstacleSize/3.0);
                obstacleCounter++;
            }
        }
    }
    else
    {
        st->nPolyObstacles = 0;
        st->polyObstacles = NULL;
    }

    st->showVelocityVectors = get_boolean_resource(st->dpy, "showVelocityVectors", "Boolean");
  
    if (onedee)
    {
        if (new_number_random_int(0,1) == 0)
            startAngles = strdup("90,270");
        else
            startAngles = strdup("0,180");
        st->startGrid = True;
        st->nObstacles = 0;
    }
    else if (st->debian)
    {
        startAngles = strdup("45");
    }
    else
        startAngles = get_string_resource(st->dpy, "startAngles", "String");

    if (strcasecmp(startAngles, "all") == 0)
    {
        st->startAngles = ANGLES_ALL;
    }
    else if (strcasecmp(startAngles, "right") == 0)
    {
        st->numStartAngles = 4;
        st->startAngles = (int*)calloc(sizeof(int), 4);
        st->startAngles[0] = 0;
        st->startAngles[1] = 90;
        st->startAngles[2] = 180;
        st->startAngles[3] = 270;
    }
    else
    {
        char* token;

        st->numStartAngles = 1;
        for (i = 0; i < strlen(startAngles); i++)
        {
            if (startAngles[i] == ',')
                st->numStartAngles++;
        }
        st->startAngles = (int*)calloc(sizeof(int), st->numStartAngles);
        i = 0;
        token = strtok(startAngles, ",");
        do
        {
            /* printf("token='%s'\n", token); */
            st->startAngles[i++] = atoi(token);
        }
        while ((token = strtok(NULL, ",")) != NULL);
    }
    /*
      for (i = 0; i < st->numStartAngles; i++)
      {
      printf("startAngle[%d] = %d\n", i, st->startAngles[i]);
      }
      exit(0);
    */

# ifdef HAVE_DOUBLE_BUFFER_EXTENSION
    st->dbeclear_p = get_boolean_resource (st->dpy, "useDBEClear", "Boolean");
#endif

# ifdef HAVE_COCOA	/* Don't second-guess Quartz's double-buffering */
    st->dbuf = False;
# endif


    /* st->transparent_p = get_boolean_resource(st->dpy, "transparent", "Transparent"); */

    st->colors = (XColor *) calloc (sizeof(*st->colors), st->ncolors);
 
    if (get_boolean_resource(st->dpy, "mono", "Boolean"))
    {
    MONO:
        st->ncolors = 1;
        st->colors[0].pixel = get_pixel_resource(st->dpy, st->xgwa.colormap,
                                                 "foreground", "Foreground");
    }
    else
    {
        make_random_colormap (st->xgwa.screen, st->xgwa.visual, st->xgwa.colormap,
                              st->colors, &st->ncolors, True, True, 0, True);
        /*
          make_smooth_colormap (st->xgwa.screen, st->xgwa.visual, st->xgwa.colormap,
          st->colors, &st->ncolors, True, 0, True);
        */
        if (st->ncolors < 2)
            goto MONO;
    }

    if (st->dbuf)
    {
#ifdef HAVE_DOUBLE_BUFFER_EXTENSION
        if (st->dbeclear_p)
            st->b = xdbe_get_backbuffer (st->dpy, st->window, XdbeBackground);
        else
            st->b = xdbe_get_backbuffer (st->dpy, st->window, XdbeUndefined);
        st->backb = st->b;
#endif /* HAVE_DOUBLE_BUFFER_EXTENSION */

        if (!st->b)
        {
            st->ba = XCreatePixmap (st->dpy, st->window, st->xgwa.width, st->xgwa.height,st->xgwa.depth);
            st->bb = XCreatePixmap (st->dpy, st->window, st->xgwa.width, st->xgwa.height,st->xgwa.depth);
            st->b = st->ba;
        }
    }
    else
    {
        st->b = st->window;
    }

    if (st->corners)
    {
        if (st->count < 8)
            st->count = 8;
        else if (st->count > 30)
            st->count = 30;
        else
            st->count -= st->count % 8;
    }

    st->balls = (Ball*) calloc(st->count, sizeof(Ball));
    st->max_radius = -100.0;
    for (i = 0; i < st->count; i++)
    { 
        Ball_init(st, &st->balls[i], i);
        if (st->balls[i].radius > st->max_radius)
            st->max_radius = st->balls[i].radius;
    }


    /* create custom GCs */
    {
        XGCValues gcv;
        unsigned long flags;
        XColor obstacleColor, obstacleColor2, highlightColor, highlightColor2;
    
        flags = GCForeground;
        gcv.foreground = ~0L;
        st->velocityVectorGC = XCreateGC (st->dpy, st->b, flags, &gcv);

        XAllocNamedColor(st->dpy, st->xgwa.colormap, "grey",
                         &obstacleColor2, &obstacleColor);
        gcv.foreground = obstacleColor.pixel;
        flags |= GCCapStyle;
        gcv.cap_style = CapRound;
        st->obstacleGC = XCreateGC (st->dpy, st->b, flags, &gcv);

        XAllocNamedColor(st->dpy, st->xgwa.colormap, "yellow",
                         &highlightColor2, &highlightColor);
        gcv.foreground = highlightColor.pixel;
        /* gcv.background = highlightColor.pixel; */
        st->highlightGC = XCreateGC (st->dpy, st->b, flags, &gcv);
    }  

#ifdef LOGGING
    printf("ball2d_init()\n");
#endif

    if (st->debian)
    {
        number angle, radius;
        int ballCounter = 0;

        st->nObstacles = 0;
        st->nPolyObstacles = 0;
        st->slowStart = True;
        st->tiny = False;

        for (angle = -45.0, radius = 0.0; angle <= 720.0;
             angle += 20.0, radius += st->max_radius*1.5)
        {
            st->balls[ballCounter].x = cos(angle * M_PI/180.0) * radius;
            st->balls[ballCounter].y = sin(angle * M_PI/180.0) * radius;

            /* no more space left? */
            if (st->balls[ballCounter].x - st->balls[ballCounter].radius < MINX ||
                st->balls[ballCounter].x + st->balls[ballCounter].radius > MAXX ||
                st->balls[ballCounter].y - st->balls[ballCounter].radius < MINY ||
                st->balls[ballCounter].y + st->balls[ballCounter].radius > MAXY)
            {
                st->count = ballCounter;
                break;
            }
            
            ballCounter++;
            /* all balls placed? */
            if (ballCounter == st->count)
                break;
        }
    }
    else if (st->corners)
    {
        st->slowStart = True;
        Balls_placeOnCorners(st);
    }
    else if (st->startGrid)
    {
        number gridWidth, gridHeight, x, y;
        ObstacleCollisionType obsCollType;

        gridWidth = st->max_radius*2 + 5.0;
        gridHeight = st->max_radius*2 + 5.0;
      
        x = MINX + st->max_radius + 5;
        y = MINY + st->max_radius + 5;
        for (i = 0; i < st->count;)
        {
            Bool conflict;

            st->balls[i].x = x;
            st->balls[i].y = y;
          
            conflict = False;
            for (j = 0; j < st->nObstacles; j++)
            {
                if (Ball_Obstacle_collision_check(&st->balls[i], &st->obstacles[j],
                                                  &obsCollType))
                    conflict = True;
            }
            for (j = 0; j < st->nPolyObstacles; j++)
            {
                if (Ball_PolygonalObstacle_collision_check(&st->balls[i],
                                                           &st->polyObstacles[j]));
                    conflict = True;
            }
            if (!conflict)
                i++;

            x += gridWidth; 
            if (x >= MAXX - st->max_radius)
            {
                x = MINX + st->max_radius + 5;
                y += gridHeight;
                if (y + gridHeight > MAXY)
                { /* no space left: ignore the st->count - i remaining balls */
                    st->count = i;
                    break;
                }
            }
        }
    }
    else
    {
        ObstacleCollisionType obsCollType;

        for (i = 0; i < st->count; i++)
        {
            int numberConflicts = 0;

            conflict = True;
            while (conflict)
            {
                Ball_randomizePosition(st, &st->balls[i]);
                conflict = False;

                /* check for collision with obstacle(s) */
                for (j = 0; j < st->nObstacles; j++)
                {
                    if (Ball_Obstacle_collision_check(&st->balls[i],
                                                      &st->obstacles[j],
                                                      &obsCollType))
                    {
                        conflict = True;
                        break;
                    }
                }
                for (j = 0; j < st->nPolyObstacles; j++)
                {
                    if (Ball_PolygonalObstacle_collision_check(&st->balls[i],
                                                               &st->polyObstacles[j]))
                    {
                        conflict = True;
                        break;
                    }
                }

                /* check for collision with other balls */
                for (j = 0; j < i; j++)
                {
                    if (Ball_collision_check(&st->balls[i], &st->balls[j])) {
                        conflict = True;
                        break;
                    }
                }
                if (conflict && ++numberConflicts > 10000)
                {
                    /* it is very likely that there's no space left for the
                       i'th ball */
                    st->count = i-1;
                    break;
                }
            }
        }
    }

    gcv.foreground = get_pixel_resource (st->dpy, st->xgwa.colormap,
                                         "background", "Background");
    st->erase_gc = XCreateGC (st->dpy, st->b, GCForeground, &gcv);

    if (st->ba) XFillRectangle (st->dpy, st->ba, st->erase_gc, 0, 0, st->xgwa.width, st->xgwa.height);
    if (st->bb) XFillRectangle (st->dpy, st->bb, st->erase_gc, 0, 0, st->xgwa.width, st->xgwa.height);

    st->nextEvent.type = NONE;
    st->nextEvent.delta_t = INFINITY;

    return st;
}


static unsigned long
ball2d_draw (Display *dpy, Window window, void *closure)
{
    int i;
    struct state *st = (struct state *) closure;
    time_t timeDelta, currentTime;
    number delta_t_remaining;
    long startMoveMicroseconds, endMoveMicroseconds, ballsMoveMicroseconds; 
    long thisDelay;

    st->iteration_number++;

#ifdef HAVE_DOUBLE_BUFFER_EXTENSION
    if (!st->dbeclear_p || !st->backb)
#endif /* HAVE_DOUBLE_BUFFER_EXTENSION */
        XFillRectangle (st->dpy, st->b, st->erase_gc, 0, 0, st->xgwa.width, st->xgwa.height);
  
    startMoveMicroseconds = time_microseconds();
    /* printf("startMove @ %ld\n", startMoveMicroseconds); */
    delta_t_remaining = SINGLE_TIME_STEP;
    while (Balls_move(st, &delta_t_remaining))
    {
        /* printf("in loop: delta_t_remaining=%.5f\n", delta_t_remaining); */
    }
    endMoveMicroseconds = time_microseconds();
    ballsMoveMicroseconds = endMoveMicroseconds - startMoveMicroseconds;
    /* printf("endMove @ %ld\n", endMoveMicroseconds); */
  
    /* printf("Balls_move_time=%ld\n", ballsMoveMicroseconds); */

    for (i = 0; i < st->nPolyObstacles; i++)
    {
        PolygonalObstacle_draw(st, st->b, &st->polyObstacles[i]);
    }
    for (i = 0; i < st->nObstacles; i++)
    {
        Obstacle_draw(st, st->b, &st->obstacles[i]);
    }
    for (i = 0; i < st->count; i++)
    {
        Ball_draw(st, st->b, &st->balls[i]);
    }


#ifdef HAVE_DOUBLE_BUFFER_EXTENSION
    if (st->backb)
    {
        XdbeSwapInfo info[1];
        info[0].swap_window = st->window;
        info[0].swap_action = (st->dbeclear_p ? XdbeBackground : XdbeUndefined);
        XdbeSwapBuffers (st->dpy, info, 1);
    }
    else
#endif /* HAVE_DOUBLE_BUFFER_EXTENSION */
        if (st->dbuf)
        {
            XCopyArea (st->dpy, st->b, st->window, st->erase_gc, 0, 0,
                       st->xgwa.width, st->xgwa.height, 0, 0);
            st->b = (st->b == st->ba ? st->bb : st->ba);
        }

    currentTime = time(NULL);
    timeDelta = currentTime - st->startTime;

    if (st->slowStart && timeDelta <= 15)
        thisDelay = (st->delay - ballsMoveMicroseconds) * 2;
    else
        thisDelay = (st->delay - ballsMoveMicroseconds);
    if (thisDelay < 0)
        return 0;
    else
        return thisDelay;
}

static void
ball2d_reshape (Display *dpy, Window window, void *closure, 
                   unsigned int w, unsigned int h)
{
    struct state *st = (struct state *) closure;
    if (! st->dbuf) {   /* #### more complicated if we have a back buffer... */
        XGetWindowAttributes (st->dpy, st->window, &st->xgwa);
        recomputeYRange(st);
        XClearWindow (dpy, window);
    }
}

static Bool
ball2d_event (Display *dpy, Window window, void *closure, XEvent *event)
{
    return False;
}

static void
ball2d_free (Display *dpy, Window window, void *closure)
{
    struct state *st = (struct state *) closure;
    int i;

    if (st->startAngles != NULL)
        free(st->startAngles);
    free_colors(st->xgwa.screen, st->xgwa.colormap, st->colors, st->ncolors);
    for (i = 0; i < st->count; i++)
    {
        XFreeGC(st->dpy, st->balls[i].gc);
    }
    free(st->balls);
    if (st->obstacles != NULL)
        free(st->obstacles);
    if (st->polyObstacles != NULL)
    {
        for (i = 0; i < st->nPolyObstacles; i++)
        {
            free(st->polyObstacles[i].points);
        }
        free(st->polyObstacles);
    }
    XFreeGC(st->dpy, st->velocityVectorGC);
    XFreeGC(st->dpy, st->obstacleGC);
}


static const char *ball2d_defaults [] = {
    ".background:		black",
    ".foreground:		white",
    "*ncolors:		50", /* TODO: bind this to `count'? */
    "*delay:	 20000",
    "*count:		20",
    "*1D: False",
    "*showVelocityVectors: True",
    "*startAngles: all",
    "*tiny: False",
    "*startGrid: False",
    "*obstacles: False",
    "*obstacles2: False",
    "*star: False",
    "*stars: False",
    "*zombies: False",
    "*corners: False",
    "*debian: False",
    "*showFutureCollisions: True",
    "*slowStart: False",
    "*doubleBuffer:	True",
#ifdef HAVE_DOUBLE_BUFFER_EXTENSION
    "*useDBE:		True",
    "*useDBEClear:	True",
#endif /* HAVE_DOUBLE_BUFFER_EXTENSION */
    0
};

static XrmOptionDescRec ball2d_options [] = {
    { "-delay",		".delay",	XrmoptionSepArg, 0 },
    { "-1D",      ".1D", XrmoptionSepArg, "False" },
    { "-showVelocityVectors", ".showVelocityVectors", XrmoptionSepArg, "True" },
    { "-startAngles", ".startAngles", XrmoptionSepArg, "all" },
    { "-tiny", ".tiny", XrmoptionSepArg, "False" },
    { "-startGrid", ".startGrid", XrmoptionSepArg, "False" },
    { "-obstacles", ".obstacles", XrmoptionSepArg, "False" },
    { "-obstacles2", ".obstacles2", XrmoptionSepArg, "False" },
    { "-star", ".star", XrmoptionSepArg, "False" },
    { "-stars", ".stars", XrmoptionSepArg, "False" },
    { "-zombies", ".zombies", XrmoptionSepArg, "False" },
    { "-corners", ".corners", XrmoptionSepArg, "False" },
    { "-debian", ".debian", XrmoptionSepArg, "False" },
    { "-showFutureCollisions", ".showFutureCollisions", XrmoptionSepArg, "True" },
    { "-slowStart", ".slowStart", XrmoptionSepArg, "False" },
    { "-count",		".count",	XrmoptionSepArg, 0 },
    { "-ncolors",		".ncolors",	XrmoptionSepArg, 0 },
    { "-db",		".doubleBuffer", XrmoptionNoArg,  "True"  },
    { "-no-db",		".doubleBuffer", XrmoptionNoArg,  "False" },
    { 0, 0, 0, 0 }
};


XSCREENSAVER_MODULE ("Ball2D", ball2d)

/*
  Local Variables:
  compile-command: "make ball2d"
  End:
*/
