#ifndef SRC_MAIN_WINDOW_H
#define SRC_MAIN_WINDOW_H 1

#include <gtkmm.h>
#include "draw.h"

class MainWindow : public Gtk::Window
{
public:
  MainWindow();
  ~MainWindow() override;

protected:
  //signal handlers:

  //Member widgets:
  Gtk::Frame m_BallFrame;
  Gtk::Box m_VBox;
  BallDrawingArea m_BallArea;

  //Cairo::RefPtr<Cairo::Surface> m_surface;
};


#endif /* SRC_MAIN_WINDOW_H */
