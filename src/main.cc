/* $Id$ */

#include <gtkmm/main.h>
#include "main_window.h"

int main (int argc, char *argv[])
{
	Glib::RefPtr<Gtk::Application> app =
		Gtk::Application::create(argc, argv, "ball2d", Gio::APPLICATION_HANDLES_COMMAND_LINE);

	MainWindow window;

	// TODO: what meaning does the 2nd argument have??
	app->signal_command_line().connect(sigc::bind(sigc::mem_fun(window, &MainWindow::on_cmdline), app), false);

	return app->run(window);
}
