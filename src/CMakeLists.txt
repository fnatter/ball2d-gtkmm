link_directories(${GTKMM_LIBRARY_DIRS})

include_directories(${GTKMM_INCLUDE_DIRS})

add_executable(ball2d
    main.cc
    draw.h draw.cc
    ball.h ball.cc
    event.h event.cc
    vector2d.h vector2d.cc
    utils.h
    ballsim.h ballsim.cc
    main_window.cc main_window.h
    )

target_link_libraries(ball2d ${GTKMM_LIBRARIES})
