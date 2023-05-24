typedef enum arqEvent
{
    arqEvent_dataTxDone = 0,
    arqEvent_ackTxDone = 1,
    arqEvent_ackRcvd = 2,
    arqEvent_dataRcvd = 3,
    arqEvent_dataToSend = 4,
    arqEvent_arqTimeout = 5
} arqEvent_e;


void arqEvent_setEventFlag(arqEvent_e event);
void arqEvent_clearEventFlag(arqEvent_e event);
void arqEvent_clearAllEventFlag(void);
int arqEvent_checkEventFlag(arqEvent_e event);