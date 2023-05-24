#define PHYMAC_STATE_RX             0
#define PHYMAC_STATE_TX             1

#define PHYMAC_ERR_NONE             0
#define PHYMAC_ERR_WRONGSTATE       1
#define PHYMAC_ERR_HWERROR          2
#define PHYMAC_ERR_SIZE             3



int phymac_dataReq(uint8_t* dataPtr, uint8_t size, uint8_t destId);
void phymac_init(uint8_t id, void (*dataCnfFunc)(int), void (*dataIndFunc)(uint8_t, uint8_t*, uint8_t));
