#include "mbed.h"
#include "string.h"

#include "ARQ_FSMevent.h"
#include "ARQ_msg.h"
#include "ARQ_timer.h"
#include "ARQ_LLinterface.h"
#include "ARQ_parameters.h"

//FSM state -------------------------------------------------
#define MAINSTATE_IDLE              0
#define MAINSTATE_TX                1
#define MAINSTATE_ACK               2

//GLOBAL variables (DO NOT TOUCH!) ------------------------------------------
//serial port interface
Serial pc(USBTX, USBRX);

//state variables
uint8_t main_state = MAINSTATE_IDLE; //protocol state

//source/destination ID
uint8_t endNode_ID=1;
uint8_t dest_ID=0;

//PDU context/size
uint8_t arqPdu[200];
uint8_t pduSize;

//SDU (input)
uint8_t originalWord[200];
uint8_t wordLen=0;



//ARQ parameters -------------------------------------------------------------
uint8_t seqNum = 0;     //ARQ sequence number
uint8_t retxCnt = 0;    //ARQ retransmission counter
uint8_t arqAck[5];      //ARQ ACK PDU

void bit_repeat(uint8_t* send, uint8_t idx, uint8_t c){
    uint32_t repeat_c = 0;
    for(int i = 7; i >= 0; i--){
        repeat_c = repeat_c <<3;
        // 1bit -> 3bit repetition code
        if(((c & (1 << i)) >> i)==1)
        {
            repeat_c = repeat_c | 0x7;
        }
        else
        {
            repeat_c = repeat_c | 0x0;
        }
    }
    send[idx]   = (repeat_c & 0x00FF0000) >> 16;
    send[idx+1] = (repeat_c & 0x0000FF00) >> 8;
    send[idx+2] = repeat_c & 0x000000FF;
}

uint8_t* FEC(uint8_t* send){
    
    uint32_t buf = 0x00000000;
    printf("buf = %u \n", buf);
    buf |= ((send[0] << 16) & 0x00FF0000) ;
    printf("buf = %u \n", buf);
    buf |= (send[1] << 8) & 0x0000FF00;
    printf("buf = %u \n", buf);
    buf |= (send[2]) & 0x000000FF;
    
    printf("buf = %u \n", buf);

    uint32_t result = 0x00000000;
    
    // for (int i = 0; i < 9; i++) {
    //     result |= ((buf & (0x00000001 << (i * 3))) >> (i * 2));
    // }
    for (int i = 0; i < 8; i++){
        int bit[3];
        int cnt1 = 0;
        int cnt0 = 0;
        bit[0] = (int)((buf & 0x000000001  << (i * 3)) >> (i *3));
        bit[1] = (int)((buf & 0x000000001  << ((i * 3) + 1)) >> (i * 3) + 1);
        bit[2] = (int)((buf & 0x000000001  << ((i * 3) + 2)) >> (i * 3)+ 2);
            for(int j = 0; j < 3; j++){
                if (bit[0] == 1){
                    cnt1++;
                }
                else {
                    cnt0++;
                }
                
            }
        if (cnt1 > cnt0){
            result |= 0x00000001 << i;
        }
        else{
            result |= 0x00000000;
        }
    }
    
    uint8_t idx = 0;
    uint8_t message[0];
    message[idx++] = (char)result & 0xFF;
    
    printf("message = %s \n",message);
    
    return message;


}
    
    

//application event handler : generating SDU from keyboard input
void arqMain_processInputWord(void)
{
    char c = pc.getc();
    if (main_state == MAINSTATE_IDLE &&
        !arqEvent_checkEventFlag(arqEvent_dataToSend))
    {
        originalWord[wordLen++] = c;
        arqEvent_setEventFlag(arqEvent_dataToSend);
        pc.printf("word is ready! ::: %s\n", originalWord);
    }
}


//FSM operation implementation ------------------------------------------------
int main(void){
    uint8_t flag_needPrint=1;
    uint8_t prev_state = 0;

    //initialization
    pc.printf("------------------ ARQ protocol starts! --------------------------\n");
    arqEvent_clearAllEventFlag();
    
    //source & destination ID setting
    pc.printf(":: ID for this node : ");
    pc.scanf("%d", &endNode_ID);
    pc.printf(":: ID for the destination : ");
    pc.scanf("%d", &dest_ID);
    pc.getc();

    pc.printf("endnode : %i, dest : %i\n", endNode_ID, dest_ID);

    arqLLI_initLowLayer(endNode_ID);
    pc.attach(&arqMain_processInputWord, Serial::RxIrq);



    while(1)
    {
        //debug message
        if (prev_state != main_state)
        {
            debug_if(DBGMSG_ARQ, "[ARQ] State transition from %i to %i\n", prev_state, main_state);
            prev_state = main_state;
        }


        //FSM should be implemented here! ---->>>>
        switch (main_state)
        {
            case MAINSTATE_IDLE: //IDLE state description
                
                if (arqEvent_checkEventFlag(arqEvent_dataRcvd)) //if data reception event happens
                {
                    //Retrieving data info.
                    uint8_t srcId = arqLLI_getSrcId();
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
                    uint8_t size = arqLLI_getSize();

                    pc.printf("\n -------------------------------------------------\nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
                                srcId, arqMsg_getWord(dataPtr), size, arqMsg_getSeq(dataPtr));
                    
                    for(int i = 0; i < 3; i++) {
                        for(int j = 7; j >= 0; j--) {
                            pc.printf("%d", (dataPtr[i+2] >> j) & 1);
                        }
                    }
                    pc.printf("\n");

                    //ACK transmission
                    arqMsg_encodeAck(arqAck, arqMsg_getSeq(dataPtr));
                    arqLLI_sendData(arqAck, ARQMSG_ACKSIZE, srcId);

                    main_state = MAINSTATE_TX; //goto TX state
                    flag_needPrint = 1;

                    arqEvent_clearEventFlag(arqEvent_dataRcvd);
                }
                else if (arqEvent_checkEventFlag(arqEvent_dataToSend)) //if data needs to be sent (keyboard input)
                {
                    uint8_t sendWord[3] = {0,0,0};
                    bit_repeat(sendWord, 0, originalWord[0]);

                    //msg header setting
                    pduSize = arqMsg_encodeData(arqPdu, sendWord, seqNum, wordLen*3);
                    arqLLI_sendData(arqPdu, pduSize, dest_ID);

                    
                    //Setting ARQ parameter 
                    seqNum = (seqNum + 1)%ARQMSSG_MAX_SEQNUM;
                    retxCnt = 0;

                    pc.printf("[MAIN] sending to %i (seq:%i)\n", dest_ID, (seqNum-1)%ARQMSSG_MAX_SEQNUM);

                    main_state = MAINSTATE_TX;
                    flag_needPrint = 1;

                    wordLen = 0;
                    arqEvent_clearEventFlag(arqEvent_dataToSend);
                }
                //ignore events (arqEvent_dataTxDone, arqEvent_ackTxDone, arqEvent_ackRcvd, arqEvent_arqTimeout)
                else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //if data needs to be sent (keyboard input)
                {
                    pc.printf("[WARNING] cannot happen in IDLE state (event %i)\n", arqEvent_dataTxDone);
                    arqEvent_clearEventFlag(arqEvent_dataTxDone);
                }
                else if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) //if data needs to be sent (keyboard input)
                {
                    pc.printf("[WARNING] cannot happen in IDLE state (event %i)\n", arqEvent_ackTxDone);
                    arqEvent_clearEventFlag(arqEvent_ackTxDone);
                }
                else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //if data needs to be sent (keyboard input)
                {
                    pc.printf("[WARNING] cannot happen in IDLE state (event %i)\n", arqEvent_ackRcvd);
                    arqEvent_clearEventFlag(arqEvent_ackRcvd);
                }
                else if (arqEvent_checkEventFlag(arqEvent_arqTimeout)) //if data needs to be sent (keyboard input)
                {
                    pc.printf("[WARNING] cannot happen in IDLE state (event %i)\n", arqEvent_arqTimeout);
                    arqEvent_clearEventFlag(arqEvent_arqTimeout);
                }
                else if (flag_needPrint == 1)
                {
                    pc.printf("Give a word to send : ");
                    flag_needPrint = 0;
                }     

                break;

            case MAINSTATE_TX: //TX state description

                if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) //data TX finished
                {
                    if (arqTimer_getTimerStatus() == 1 ||
                        arqEvent_checkEventFlag(arqEvent_arqTimeout))
                    {
                        main_state = MAINSTATE_ACK;
                    }
                    else
                    {
                        main_state = MAINSTATE_IDLE;
                    }

                    arqEvent_clearEventFlag(arqEvent_ackTxDone);
                }
                else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //data TX finished
                {
                    main_state = MAINSTATE_ACK;
                    arqTimer_startTimer(); //start ARQ timer for retransmission

                    arqEvent_clearEventFlag(arqEvent_dataTxDone);
                }

                break;
            case MAINSTATE_ACK: //ACK state description

                if (arqEvent_checkEventFlag(arqEvent_ackRcvd)) //data TX finished
                {
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
                    if ( arqMsg_getSeq(arqPdu) == arqMsg_getSeq(dataPtr) )
                    {
                        pc.printf("ACK is correctly received! \n");
                        arqTimer_stopTimer();
                        main_state = MAINSTATE_IDLE;
                    }
                    else
                    {
                        pc.printf("ACK seq number is weird! (expected : %i, received : %i\n", arqMsg_getSeq(arqPdu),arqMsg_getSeq(dataPtr));
                    }

                    arqEvent_clearEventFlag(arqEvent_ackRcvd);
                }
                else if (arqEvent_checkEventFlag(arqEvent_arqTimeout)) //data TX finished
                {

                    pc.printf("timeout! retransmit\n");
                    arqLLI_sendData(arqPdu, pduSize, dest_ID);
                    //Setting ARQ parameter 
                    retxCnt += 1;
                    main_state = MAINSTATE_TX;

                    arqEvent_clearEventFlag(arqEvent_arqTimeout);
                }
                else if (arqEvent_checkEventFlag(arqEvent_dataRcvd)) //data TX finished
                {
                    //Retrieving data info.
                    uint8_t srcId = arqLLI_getSrcId();
                    uint8_t* dataPtr = arqLLI_getRcvdDataPtr();
                    uint8_t size = arqLLI_getSize();

                    pc.printf("\n -------------------------------------------------\nRCVD from %i : %s (length:%i, seq:%i)\n -------------------------------------------------\n", 
                                srcId, arqMsg_getWord(dataPtr), size, arqMsg_getSeq(dataPtr));

                    //ACK transmission
                    arqMsg_encodeAck(arqAck, arqMsg_getSeq(dataPtr));
                    arqLLI_sendData(arqAck, ARQMSG_ACKSIZE, srcId);

                    main_state = MAINSTATE_TX; //goto TX state

                    arqEvent_clearEventFlag(arqEvent_dataRcvd);
                }
                else if (arqEvent_checkEventFlag(arqEvent_dataTxDone)) //data TX finished
                {
                    pc.printf("[WARNING] cannot happen in ACK state (event %i)\n", arqEvent_dataTxDone);
                    arqEvent_clearEventFlag(arqEvent_dataTxDone);
                }
                else if (arqEvent_checkEventFlag(arqEvent_ackTxDone)) //data TX finished
                {
                    pc.printf("[WARNING] cannot happen in ACK state (event %i)\n", arqEvent_ackTxDone);
                    arqEvent_clearEventFlag(arqEvent_ackTxDone);
                }

                break;

            default :
                break;
        }
    }
}