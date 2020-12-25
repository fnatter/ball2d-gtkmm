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
