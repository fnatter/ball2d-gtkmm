#include "main_window.h"

#include <iostream>

MainWindow::MainWindow()
: m_VBox(Gtk::ORIENTATION_VERTICAL, 8), m_BallArea(m_sim)
{
	set_title("ball2d-gtkmm");
	set_border_width(8);

	m_VBox.set_border_width(0);
	add(m_VBox);

	// Create the ball2d area
	m_BallFrame.set_shadow_type(Gtk::SHADOW_IN);
	m_VBox.pack_start(m_BallFrame);

	// set a minimum size
	m_BallArea.set_size_request(1024, 768);
	m_BallFrame.add(m_BallArea);
  
	Glib::signal_timeout().connect(
	  sigc::mem_fun(*this, &MainWindow::on_timer), simTimeoutMs);

	show_all();
}

MainWindow::~MainWindow()
{
}

bool MainWindow::on_timer()
{
	//std::cout << "Timer event\n";
	
	number delta_t_remaining = SINGLE_TIME_STEP;
    while (m_sim.move(&delta_t_remaining))
    {
		//std::cout << "in loop: delta_t_remaining=" << delta_t_remaining << std::endl;
    }
	
	m_BallArea.queue_draw();
	return true;
}

