#include "main.h"

int32_t WifiData[50];

bool PC_Start_Flag = FALSE;
bool PC_DataFlag = FALSE;

uint8_t mspPacket[PAKETSIZE];

void init_Wifi(void)
{
  Serial2.begin(9600);
  Serial2.setTimeout(5000);
  
  Set_Wifi_Mode_AP();
  while(Serial2.available()){ Serial.write(Serial2.read()); }  
  delay(1000);
  
  Set_Wifi_Mux();
  while(Serial2.available()){ Serial.write(Serial2.read()); }
  delay(1000);
  
  Set_Wifi_SSID_AP();
  while(Serial2.available()){ Serial.write(Serial2.read()); }  
  delay(1000);
  
  Set_Wifi_TCP_Server();
  while(Serial2.available()){ Serial.write(Serial2.read()); }
  delay(1000);
  
  Set_CIPSTO();
  while(Serial2.available()){ Serial.write(Serial2.read()); }
}

void Wifi_Checking()
{
    uint8_t cnt;
    uint8_t PC_cnt;
    uint8_t read_data = 0;

    int32_t PC_Buffer[50];  
    
    while(Serial2.available())
    {
        read_data = Serial2.read();
        if(read_data == '$') // STX
        {
          PC_Start_Flag = TRUE;
          PC_cnt = 0;
        }
        else if((read_data == '#')  && (PC_Start_Flag == 1)) // ETX
        {
          PC_Start_Flag = FALSE;
          PC_DataFlag = TRUE;
          
          for(cnt = 0; cnt < 49; cnt++)
          {
            WifiData[cnt] = PC_Buffer[cnt + 1];
            PC_Buffer[cnt] = '0';
          }
          break;
        }
        
        if(PC_Start_Flag == TRUE)
        {
          PC_Buffer[PC_cnt] = read_data;
          PC_cnt++;
        }
    
        read_data = 0;
    }
  
    if(PC_DataFlag == TRUE)
    {
      //for(cnt = 0; cnt < 50; cnt++) { WifiData[cnt] = '0'; }
        uint8_t cnt;
        
        PC_DataFlag = 0;
        PC_cnt = 0;
        Wifi_Command();

        for(cnt = 0; cnt < 50; cnt++) { WifiData[cnt] = '0'; }
    }  
}
