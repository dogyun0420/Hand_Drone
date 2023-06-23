extern float THRO_CAL;

extern int32_t WifiData[50];
enum{
  HEAD1,
  HEAD2,
  HEAD3,
  DATASIZE,
  CMD,
  ROLL,
  PITCH,
  YAW,
  THROTTLE,
  AUX,
  CRC,
  PAKETSIZE
};
extern uint8_t mspPacket[PAKETSIZE];

void init_Wifi(void);
void Wifi_Checking(void);
