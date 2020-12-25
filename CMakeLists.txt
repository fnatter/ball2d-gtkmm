cmake_minimum_required(VERSION 3.13)
# https://wiki.gnome.org/Projects/gtkmm/UsingCMake

project(ball2d-gtkmm)

find_package(PkgConfig)

# apt-get install libgtkmm-3.0-dev
pkg_check_modules(GTKMM gtkmm-3.0) # look into FindPkgConfig.cmake, it contains documentation
# Now the variables GTKMM_INCLUDE_DIRS, GTKMM_LIBRARY_DIRS and GTKMM_LIBRARIES 
# contain what you expect 

add_subdirectory(src) 


