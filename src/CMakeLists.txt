link_directories(${GTKMM_LIBRARY_DIRS})

include_directories(${GTKMM_INCLUDE_DIRS})

add_executable(ball2d
    main.cc 
    main_window.cc 
    main_window.h
    )

target_link_libraries(ball2d ${GTKMM_LIBRARIES})
