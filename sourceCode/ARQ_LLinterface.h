void arqLLI_initLowLayer(uint8_t srcId);
void arqLLI_sendData(uint8_t* msg, uint8_t size, uint8_t dest);
uint8_t arqLLI_getSrcId();
uint8_t* arqLLI_getRcvdDataPtr();
uint8_t arqLLI_getSize();