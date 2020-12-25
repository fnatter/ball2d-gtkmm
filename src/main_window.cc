#include "main_window.h"

MainWindow::MainWindow()
:
  m_VBox(Gtk::ORIENTATION_VERTICAL, 8)
{
  set_title("ball2d-gtkmm");
  set_border_width(8);

  m_VBox.set_border_width(8);
  add(m_VBox);

  // Create the ball2d area
  m_BallFrame.set_shadow_type(Gtk::SHADOW_IN);
  m_VBox.pack_start(m_BallFrame);

  // set a minimum size
  m_BallArea.set_size_request(1024, 768);
  m_BallFrame.add(m_BallArea);

  m_BallArea.signal_draw().connect(
      sigc::mem_fun(*this, &MainWindow::on_drawingarea_draw));

  show_all();
}

MainWindow::~MainWindow()
{
}

bool MainWindow::on_drawingarea_draw(const Cairo::RefPtr<Cairo::Context>& cr)
{
  /* At the start of a draw handler, a clip region has been set on
   * the Cairo context, and the contents have been cleared to the
   * widget's background color. The docs for
   * Gdk::Window::begin_paint_region() give more details on how this
   * works.
   */
   
  const int width = m_BallArea.get_allocated_width();
  const int height = m_BallArea.get_allocated_height();
  const int lesser = std::min(width, height);
  const int xc = width / 2;
  const int yc = height / 2;

  cr->set_line_width(lesser * 0.02);  // outline thickness changes
                                      // with window size
    
  cr->save();
  cr->arc(xc, yc, lesser / 4.0, 0.0, 2.0 * M_PI); // full circle
  cr->set_source_rgba(0.0, 0.0, 0.8, 0.6);    // partially translucent
  cr->fill_preserve();
  cr->restore();  // back to opaque black
  cr->stroke();

  // return true because we've handled this event, so no
  // further processing is required.
  return true;
}
