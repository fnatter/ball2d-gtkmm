#include "event.h"

Event Event::find_closest(Event* events, int count)
{
    int i;
    Event* closest = &events[0];

    for (i = 1; i < count; i++)
    {
        if (events[i].delta_t < closest->delta_t)
            closest = &events[i];
    }
    return *closest;
}

void Event::print(std::ostream& out) const
{
	const char* typeNames[] = { "NONE",
                          "LEFT_WALL_HIT", "RIGHT_WALL_HIT",
                          "TOP_WALL_HIT", "BOTTOM_WALL_HIT",
                          "OBSTACLE_COLLISION", "POLYGONAL_OBSTACLE_COLLISION",
                          "BALL_COLLISION" };

    const char* obsTypeNames[] = { "LEFT_EDGE", "RIGHT_EDGE", 
                             "TOP_EDGE", "BOTTOM_EDGE",
                             "TOPLEFT_CORNER", "TOPRIGHT_CORNER", 
                             "BOTTOMLEFT_CORNER", "BOTTOMRIGHT_CORNER",
                             "CENTER_INSIDE" };

    if (type == EventType::OBSTACLE_COLLISION)
    {
		out << "ev.type=" << typeNames[as_integer(type)] << "::" << obsTypeNames[as_integer(obsCollType)]
		    << ", ev.ball=" << std::hex << ball << std::dec
		    << ", t=" << delta_t << std::endl;
    }
    else if (type == EventType::POLYGONAL_OBSTACLE_COLLISION)
    {
		out << "ev.type=" << typeNames[as_integer(type)] << "::" << polyObsCollType
		    << ", ev.ball=" << std::hex << ball << std::dec
		    << ", t=" << delta_t << std::endl;
    }
    else
    {
		out << "ev.type=" << typeNames[as_integer(type)]
		    << ", ev.ball=" << std::hex << ball << std::dec
		    << ", ev.ball2=" << std::hex << ball2 << std::dec
		    << ", t=" << delta_t << std::endl;
    }
}
