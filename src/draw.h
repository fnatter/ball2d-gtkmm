#ifndef SRC_DRAW_H
#define SRC_DRAW_H 1

#include <gtkmm.h>
#include "ballsim.h"

class BallDrawingArea : public Gtk::DrawingArea
{
public:
  BallDrawingArea();
  ~BallDrawingArea() { }
  
protected:
  //signal handlers:
  bool on_drawingarea_draw(const Cairo::RefPtr<Cairo::Context>& cr);
  
  BallSimulation m_sim;
};

#endif /* SRC_DRAW_H */
