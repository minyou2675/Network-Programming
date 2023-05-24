#include "mbed.h"
#include "PHYMAC_layer.h"
#include "ARQ_FSMevent.h"
#include "ARQ_msg.h"
#include "ARQ_parameters.h"

#define ARQLLI_MAX_PDUSIZE      50

static uint8_t txType;
static uint8_t rcvdData[ARQLLI_MAX_PDUSIZE];
static uint8_t rcvdSrc;
static uint8_t rcvdSize;

//interface event : DATA_CNF, TX done event
void arqLLI_dataCnfFunc(int err) 
{
    if (txType == ARQMSG_TYPE_DATA)
    {
        arqEvent_setEventFlag(arqEvent_dataTxDone);
    }
    else if (txType == ARQMSG_TYPE_ACK)
    {
        arqEvent_setEventFlag(arqEvent_ackTxDone);
    }
}

//interface event : DATA_IND, RX data has arrived
void arqLLI_dataIndFunc(uint8_t srcId, uint8_t* dataPtr, uint8_t size)
{
    debug_if(DBGMSG_ARQ, "\n --> DATA IND : src:%i, size:%i\n", srcId, size);

    memcpy(rcvdData, dataPtr, size*sizeof(uint8_t));
    rcvdSrc = srcId;
    rcvdSize = size;

    //ready for ACK TX
    if (arqMsg_checkIfData(dataPtr))
    {
        arqEvent_setEventFlag(arqEvent_dataRcvd);
    }
    else if (arqMsg_checkIfAck(dataPtr))
    {
        arqEvent_setEventFlag(arqEvent_ackRcvd);
    }
}


void arqLLI_initLowLayer(uint8_t srcId)
{
    phymac_init(srcId, arqLLI_dataCnfFunc, arqLLI_dataIndFunc);
}




//TX function
void arqLLI_sendData(uint8_t* msg, uint8_t size, uint8_t dest)
{
    phymac_dataReq(msg, size, dest);
    txType = msg[ARQMSG_OFFSET_TYPE];
}


//GET functions
uint8_t arqLLI_getSrcId()
{
    return rcvdSrc;
}

uint8_t* arqLLI_getRcvdDataPtr()
{
    return rcvdData;
}

uint8_t arqLLI_getSize()
{
    return rcvdSize;
}