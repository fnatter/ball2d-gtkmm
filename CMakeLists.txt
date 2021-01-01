cmake_minimum_required(VERSION 3.13)
# https://wiki.gnome.org/Projects/gtkmm/UsingCMake

project(ball2d-gtkmm)

set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin/Linux")

find_package(PkgConfig)

# apt-get install libgtkmm-3.0-dev
pkg_check_modules(GTKMM gtkmm-3.0) # look into FindPkgConfig.cmake, it contains documentation
# Now the variables GTKMM_INCLUDE_DIRS, GTKMM_LIBRARY_DIRS and GTKMM_LIBRARIES 
# contain what you expect 

add_subdirectory(src) 
