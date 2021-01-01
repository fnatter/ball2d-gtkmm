#include "draw.h"
#include <iostream>
 
BallDrawingArea::BallDrawingArea(const BallSimulation& sim)
	: m_sim(sim)
{
	signal_draw().connect(
      sigc::mem_fun(*this, &BallDrawingArea::on_drawingarea_draw));
}

void BallDrawingArea::drawObstacles(const Cairo::RefPtr<Cairo::Context>& cr)
{
	for (const Obstacle& obstacle: m_sim.obstacles)
	{
		//std::cout << "Obstacle: " << obstacle << std::endl;
		
		cr->save();
		cr->rectangle(obstacle.x1, obstacle.y1, obstacle.x2-obstacle.x1, obstacle.y2-obstacle.y1);
		cr->set_source_rgba(0.9, 0.9, 0.9, 0.6);
		cr->fill_preserve();
		cr->restore();  // back to opaque black
		cr->stroke();
	}

	for (const PolygonalObstacle& pobstacle: m_sim.polyObstacles)
	{
		cr->save();
		for (size_t i = 0; i < pobstacle.points.size(); i++)
		{
			cr->line_to(pobstacle.points[i].x, pobstacle.points[i].y);
		}
		cr->close_path();
		cr->set_source_rgba(0.9, 0.9, 0.9, 0.6);
		cr->fill_preserve();
		cr->restore();  // back to opaque black
		cr->stroke();
	}
}

void BallDrawingArea::drawBalls(const Cairo::RefPtr<Cairo::Context>& cr,
	bool showVelocityVectors)
{
	for (const Ball& ball: m_sim.balls)
	{
#ifdef LOGGING
		std::cout << "Drawing ball: " << ball << std::endl;
#endif
	
		cr->save();
		cr->arc(ball.x, ball.y, ball.radius, 0.0, 2.0 * M_PI); // full circle
		cr->set_source_rgba(ball.r, ball.g, ball.b, ball.a);
		cr->fill_preserve();
		cr->restore();
		cr->stroke();
		
		if (showVelocityVectors)
		{
			cr->save();
			cr->set_line_width(1.0);
			cr->set_source_rgba(0.95, 0.95, 0.95, 0.9);
			cr->move_to(ball.x, ball.y);
			Vector2D velocityVector;
			velocityVector.x = ball.dx;
			velocityVector.y = ball.dy;
			Vector2D_multScalar(&velocityVector, ball.radius / MAX_VELOCITY);
			velocityVector.x += ball.x;
			velocityVector.y += ball.y;
			cr->line_to(velocityVector.x, velocityVector.y);
			cr->restore();
			cr->stroke();
		}
	}
}

bool BallDrawingArea::on_drawingarea_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
  /* At the start of a draw handler, a clip region has been set on
   * the Cairo context, and the contents have been cleared to the
   * widget's background color. The docs for
   * Gdk::Window::begin_paint_region() give more details on how this
   * works.
   */
	const int width = get_allocated_width();
	const int height = get_allocated_height();

	// outline thickness changes with window size
	cr->set_line_width(0.1);

	// set up user coordinate system: (MINX, MINY) to (MAXX, MAXY)
	cr->scale((width - 1) / (MAXX-MINX), (height - 1) / (MAXY-MINY));
	cr->translate(-MINX, -MINY);
                                      
	drawObstacles(cr);
	drawBalls(cr, m_sim.showVelocityVectors);

	if (m_sim.showFutureCollisions && m_sim.nextEvent.type == EventType::BALL_COLLISION)
	{
		Ball* b1 = m_sim.nextEvent.ball;
		Ball* b2 = m_sim.nextEvent.ball2;
		Vector2D coll, coll2, line_of_sight_at_coll;
        // get the center of b1 at collision time
        coll.x = b1->x +
            b1->dx * b1->velocity * m_sim.nextEvent.delta_t;
        coll.y = b1->y +
            b1->dy * b1->velocity * m_sim.nextEvent.delta_t;

        // get the center of b2 at collision time
        coll2.x = b2->x +
            b2->dx * b2->velocity * m_sim.nextEvent.delta_t;
        coll2.y = b2->y +
            b2->dy * b2->velocity * m_sim.nextEvent.delta_t;

        // get the point where the two balls will touch
        line_of_sight_at_coll = Vector2D_sub(&coll2, &coll);
        Vector2D_normalize(&line_of_sight_at_coll);
        coll.x += line_of_sight_at_coll.x * b1->radius;
        coll.y += line_of_sight_at_coll.y * b1->radius;

        // draw a cross where the balls will meet:
		cr->save();
		cr->set_line_width(0.2);
		cr->set_source_rgba(1.0, 1.0, 0.0, 0.9);

		cr->move_to(coll.x - 3, coll.y + 3);
		cr->line_to(coll.x + 3, coll.y - 3);
		cr->stroke();

		cr->move_to(coll.x - 3, coll.y - 3);
		cr->line_to(coll.x + 3, coll.y + 3);
		cr->stroke();

		cr->restore();
	}

	// return true because we've handled this event, so no
	// further processing is required.
	return true;
}
