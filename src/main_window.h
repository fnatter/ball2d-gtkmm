#ifndef SRC_MAIN_WINDOW_H
#define SRC_MAIN_WINDOW_H 1

#include <gtkmm.h>

class MainWindow : public Gtk::Window
{
public:
  MainWindow();
  ~MainWindow() override;

protected:
  //signal handlers:
  bool on_drawingarea_draw(const Cairo::RefPtr<Cairo::Context>& cr);

  //Member widgets:
  Gtk::Frame m_BallFrame;
  Gtk::Box m_VBox;
  Gtk::DrawingArea m_BallArea;

  Cairo::RefPtr<Cairo::Surface> m_surface;
};


#endif /* SRC_MAIN_WINDOW_H */
