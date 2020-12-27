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

void BallDrawingArea::drawBalls(const Cairo::RefPtr<Cairo::Context>& cr)
{
	for (const Ball& ball: m_sim.balls)
	{
#ifdef LOGGING
		std::cout << "Drawing ball: " << ball << std::endl;
#endif
	
		cr->save();
		cr->arc(ball.x, ball.y, ball.radius, 0.0, 2.0 * M_PI); // full circle
		cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
		cr->fill_preserve();
		cr->restore();  // back to opaque black
		cr->stroke();
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
	drawBalls(cr);

	// return true because we've handled this event, so no
	// further processing is required.
	return true;
}
