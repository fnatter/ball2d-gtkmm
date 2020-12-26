#include "draw.h"
#include <iostream>
 
BallDrawingArea::BallDrawingArea(const BallSimulation& sim)
	: m_sim(sim)
{
	signal_draw().connect(
      sigc::mem_fun(*this, &BallDrawingArea::on_drawingarea_draw));

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
	const int lesser = std::min(width, height);
	const int xc = width / 2;
	const int yc = height / 2;

	cr->set_line_width(lesser * 0.0002);  // outline thickness changes
                                      // with window size
                                      
	for (const Ball& ball: m_sim.balls)
	{
		std::cout << "Drawing ball: " << ball << std::endl;
		ball.draw(cr, width, height);
	}

	/*                                      
	cr->save();
	cr->arc(xc, yc, lesser / 4.0, 0.0, 2.0 * M_PI); // full circle
	cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
	cr->fill_preserve();
	cr->restore();  // back to opaque black
	cr->stroke();
	*/
  

	// return true because we've handled this event, so no
	// further processing is required.
	return true;
}
