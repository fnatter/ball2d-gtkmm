#ifndef SRC_DRAW_H
#define SRC_DRAW_H 1

#include <gtkmm.h>
#include "ballsim.h"

class BallDrawingArea : public Gtk::DrawingArea
{
public:
	BallDrawingArea(const BallSimulation& sim);
	~BallDrawingArea() { }
  
protected:
	//signal handlers:
	bool on_drawingarea_draw(const Cairo::RefPtr<Cairo::Context>& cr);
	
	void drawObstacles(const Cairo::RefPtr<Cairo::Context>& cr);
	void drawBalls(const Cairo::RefPtr<Cairo::Context>& cr);
	
	const BallSimulation& m_sim;
};

#endif /* SRC_DRAW_H */
