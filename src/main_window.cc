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

  show_all();
}

MainWindow::~MainWindow()
{
}

