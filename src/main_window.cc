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
  
	Glib::signal_timeout().connect(sigc::mem_fun(*this, &MainWindow::on_timer), simTimeoutMs);
	
	// we override the default event signal handler
	add_events(Gdk::KEY_PRESS_MASK);
	
	// Events in gtkmm4:
	//auto controller = Gtk::EventControllerKey::create();
	//controller->signal_key_pressed().connect(sigc::mem_fun(*this, &MainWindow::on_window_key_pressed), false);
	//add_controller(controller);
	  
	show_all();
}

MainWindow::~MainWindow()
{
}

int MainWindow::on_cmdline(const Glib::RefPtr<Gio::ApplicationCommandLine>& cmdline, Glib::RefPtr<Gtk::Application> &app)
{
	// without activate() the window won't be shown.
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

	entry = Glib::OptionEntry();
	entry.set_long_name("slowstart");
	entry.set_description("Move slowly at the beginning");
	group.add_entry(entry, m_sim.slowStartMode);

	entry = Glib::OptionEntry();
	entry.set_long_name("showvelocityvectors");
	entry.set_description("Show each ball's velocity vector");
	group.add_entry(entry, m_sim.showVelocityVectors);

	entry = Glib::OptionEntry();
	entry.set_long_name("speed");
	entry.set_short_name('s');
	entry.set_description("speed ([0;10])");
	group.add_entry(entry, m_sim.speed);

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

	m_sim.innerLoop();
	m_BallArea.queue_draw();

	return true;
}

bool MainWindow::on_key_press_event(GdkEventKey* event)
{
	if (event->keyval == GDK_KEY_Escape ||
	    event->keyval == 'q' || event->keyval == 'Q')
	{
		hide();
		return true;
	}

	if (event->keyval == GDK_KEY_plus || event->keyval == GDK_KEY_KP_Add)
	{
		m_sim.speed = std::min(m_sim.speed + 1, 10);
		return true;
	}

	if (event->keyval == GDK_KEY_minus || event->keyval == GDK_KEY_KP_Subtract)
	{
		m_sim.speed = std::max(m_sim.speed - 1, 0);
		return true;
	}
	
	return false;
}
