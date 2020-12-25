/* $Id$ */

#include <gtkmm/main.h>
#include "main_window.h"

int main (int argc, char *argv[])
{
  Glib::RefPtr<Gtk::Application> app =
    Gtk::Application::create(argc, argv);

  MainWindow window;

  return app->run(window);
}
