/* $Id$ */

#include <gtkmm/main.h>
#include "main_window.h"

int main (int argc, char *argv[])
{
	Glib::RefPtr<Gtk::Application> app =
		Gtk::Application::create(argc, argv, "ball2d", Gio::APPLICATION_HANDLES_COMMAND_LINE);

	MainWindow window;

	// Only one signal handler is invoked. This signal handler must run before
	// the default signal handler, or else it won't run at all.
	app->signal_command_line().connect(sigc::bind(sigc::mem_fun(window, &MainWindow::on_cmdline), app), false);

	return app->run(window);
}
