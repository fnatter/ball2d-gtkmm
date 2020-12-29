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

int MainWindow::on_cmdline(const Glib::RefPtr<Gio::ApplicationCommandLine>& cmdline, Glib::RefPtr<Gtk::Application> &app)
{
	app->activate();
	
	Glib::OptionContext ctx;
	Glib::OptionGroup group("options", "main options");
	
	Glib::OptionEntry entry;
	entry.set_short_name('n');
	entry.set_long_name("number-of-balls");
	entry.set_description("the number of balls in the simulation");
	group.add_entry(entry, m_sim.numberOfBalls);
	
	entry = Glib::OptionEntry();
	entry.set_long_name("tiny");
	entry.set_description("use only small balls");
	group.add_entry(entry, m_sim.tinyMode);
	
	entry = Glib::OptionEntry();
	entry.set_long_name("debian");
	entry.set_description("arrange starting points as Debian logo");
	group.add_entry(entry, m_sim.debianMode);

	entry = Glib::OptionEntry();
	entry.set_long_name("corners");
	entry.set_description("corners are starting points");
	group.add_entry(entry, m_sim.cornersMode);

	ctx.add_group(group);

	// add GTK options, --help-gtk, etc
	Glib::OptionGroup gtkgroup(gtk_get_option_group(true));
	ctx.add_group(gtkgroup);
	
	int argc;
	char **argv = cmdline->get_arguments(argc);
	ctx.parse(argc, argv);
	
	m_sim.init();

	return 0;
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

