#include "mbed.h"
#include "ARQ_FSMevent.h"
#include "ARQ_parameters.h"



//ARQ retransmission timer
static Timeout timer;                       
static uint8_t timerStatus = 0;


//timer event : ARQ timeout
void arqTimer_timeoutHandler(void) 
{
    timerStatus = 0;
    arqEvent_setEventFlag(arqEvent_arqTimeout);
}

//timer related functions ---------------------------
void arqTimer_startTimer()
{
    uint8_t waitTime = ARQTIMER_MINWAITTIME + rand()%(ARQTIMER_MAXWAITTIME-ARQTIMER_MINWAITTIME);
    timer.attach(arqTimer_timeoutHandler, waitTime);
    timerStatus = 1;
}

void arqTimer_stopTimer()
{
    timer.detach();
    timerStatus = 0;
}

uint8_t arqTimer_getTimerStatus()
{
    return timerStatus;
}
