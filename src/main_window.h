#ifndef SRC_MAIN_WINDOW_H
#define SRC_MAIN_WINDOW_H 1

#include <gtkmm.h>
#include "draw.h"

class MainWindow : public Gtk::Window
{
	
public:
	MainWindow();
	~MainWindow() override;

	//signal handlers:
	bool on_timer();
	int on_cmdline(const Glib::RefPtr<Gio::ApplicationCommandLine> & cmdline,
		Glib::RefPtr<Gtk::Application> &appl);

protected:
	//Member widgets:
	Gtk::Frame m_BallFrame;
	Gtk::Box m_VBox;
	BallDrawingArea m_BallArea;
	BallSimulation m_sim;
	
	//Cairo::RefPtr<Cairo::Surface> m_surface;
};


#endif /* SRC_MAIN_WINDOW_H */
