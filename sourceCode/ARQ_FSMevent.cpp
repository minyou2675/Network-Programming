#include "mbed.h"
#include "ARQ_FSMevent.h"

static uint32_t eventFlag;


void arqEvent_setEventFlag(arqEvent_e event)
{
    eventFlag |= (0x01 << event);
}

void arqEvent_clearEventFlag(arqEvent_e event)
{
    eventFlag &= ~(0x01 << event);
}
void arqEvent_clearAllEventFlag(void)
{
    eventFlag = 0;
}

int arqEvent_checkEventFlag(arqEvent_e event)
{
    return (eventFlag & (0x01 << event));
}