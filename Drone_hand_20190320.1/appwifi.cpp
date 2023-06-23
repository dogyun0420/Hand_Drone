//#include <SoftwareSerial.h>

#include "Arduino.h"
#include "mpu_sensor.h"
#include "appwifi.h"
#include "LCD_ST7735.h"


//#define BT_RXD 16
//#define BT_TXD 17
//#define BT_RXD 4
//#define BT_TXD 5
//SoftwareSerial ESP_wifi(BT_RXD, BT_TXD);

extern uint8_t client;
extern uint8_t client2;
uint8_t Send_OK = 1;
uint8_t Temp_IP[3] = {48,48, 49};
uint8_t Wifi_Connect = 0;
void Wifi_Reset(void)
{
  Serial2.println("AT+RST");
}
void Set_Wifi_Mode(void)
{
  Serial2.println("AT+CWMODE=1");
}
void Set_Wifi_Mux(void)
{
  Serial2.println("AT+CIPMUX=0");  
}
void Set_Wifi_Join_AP(void)
{
  
  Serial2.print("AT+CWJAP=");
  Serial2.write('"');
  Serial2.print("DRONE1");
  Serial2.write('"');
  Serial2.write(',');
  Serial2.write('"');
  Serial2.print("00700070");
  Serial2.write('"');
  Serial2.write('\r');
  Serial2.write('\n');
}
void Set_Wifi_TCP_Server(void)
{
  Serial2.print("AT+CIPSERVER=1,20000");
  Serial2.write('\r');
  Serial2.write('\n');
}
void Set_Wifi_TCP_Client(void)
{
  Serial2.print("AT+CIPSTART=");
  Serial2.write('"');
  Serial2.print("TCP");
  Serial2.write('"');
  Serial2.write(',');
  Serial2.write('"');
  Serial2.print("192.168.4.1");
  Serial2.write('"');
  Serial2.write(',');
  Serial2.print("333");
  Serial2.write('\r');
  Serial2.write('\n');
}
void Set_Wifi_IP(void)
{
  Serial2.print("AT+CIPSTA=");
  Serial2.write('"');
  Serial2.print("192.168.4.100");
  Serial2.write('"');
  Serial2.write(',');
  Serial2.write('"');
  Serial2.print("192.168.4.1");
  Serial2.write('"');
  Serial2.write(',');
  Serial2.write('"');
  Serial2.print("255.255.255.0");
  Serial2.write('"');
  Serial2.write('\r');
  Serial2.write('\n');
}

void Set_CIPMODE1(void)
{
  Serial2.print("AT+CIPMODE=1");
  Serial2.write('\r');
  Serial2.write('\n');
}
void Set_CIPMODE0(void)
{
  Serial2.print("AT+CIPMODE=0");
  Serial2.write('\r');
  Serial2.write('\n');
}
void Set_Wifi_Send_Data(char *data)
{  
  int buff[100];
  uint8_t read_data[8];
  char szPacket[128]={0,};
  int i = 0;
  int data_len;

  data_len = strlen(data);
  sprintf(szPacket, "AT+CIPSEND=%d",data_len);
  Serial2.println(szPacket);
  
  while(Serial2.available())
  {
    Serial.write(Serial2.read());
  } 
  Serial2.print(data);
 
  while(Serial2.available())
  {
    Serial2.readBytes(read_data, 8);
  }
  delay(200); 
  for(i=0; i<8; i++)
  {
    if((read_data[i] == 'S' && read_data[i+1] == 'E' && read_data[i+2] == 'N' && read_data[i+3] == 'D'))
    {
      Serial.println("SEND OK");
      Wifi_Connect = 1;
      break;
    }
    if((read_data[i] == 'U' && read_data[i+1] == 'n' && read_data[i+2] == 'l' && read_data[i+3] == 'i'))
    {
      Serial.println("Unlink");
      Wifi_Connect = 0;
      break;
    }
  }
  
}
void Init_Wifi(void)
{
  uint8_t read_data[64];
  int i;
  Serial2.begin(9600);
  Serial2.setTimeout(5000);
  Wifi_Reset();
  delay(1000); 
  while(Serial2.available())
  {
    Serial.write(Serial2.read());
  } 
  Set_Wifi_Mode();
  delay(1000); 
  while(Serial2.available())
  {
    Serial2.readBytes(read_data, 63);
  }
  for(i=0; i<63; i++)
  {
    if(read_data[i] == 'b' && read_data[i+1] == 'u' && read_data[i+2] == 's' && read_data[i+3] == 'y')
    {
      Wifi_Connect = 2;
      break;
    }
  }
  Set_Wifi_Join_AP();
  delay(1000);  
  delay(1000);  
  delay(1000);  
  delay(1000);  
  while(Serial2.available())
  {
    Serial.write(Serial2.read());
  }
  Set_Wifi_Mux();
  delay(1000);
  while(Serial2.available())
  {
    Serial2.readBytes(read_data, 63);
  }
  for(i=0; i<63; i++)
  {
    if(read_data[i] == 'b' && read_data[i+1] == 'u' && read_data[i+2] == 's' && read_data[i+3] == 'y')
    {
      Wifi_Connect = 2;
      break;
    }
  }  
  Set_Wifi_TCP_Client();
  delay(1000); 
  while(Serial2.available())
  {
    Serial2.readBytes(read_data, 63);
  }
  for(i=0; i<63; i++)
  {
    if(read_data[i] == 'L' && read_data[i+1] == 'i' && read_data[i+2] == 'n' && read_data[i+3] == 'k' && read_data[i+4] == 'e' && read_data[i+5] == 'd')
    {
      Serial.println("LINK OK");
      Wifi_Connect = 1;
      break;
    }
  }
  Set_CIPMODE0();
  delay(1000); 
  while(Serial2.available())
  {
    Serial.write(Serial2.read());
  }
}
void AT_CMD(void)
{
  if(Serial.available())
  {
    Serial2.write(Serial.read());
  }
  if(Serial2.available())
  {
    Serial.write(Serial2.read());
  }
}
void Get_Wifi_IP(void)
{
  Serial2.println("AT+CIFSR");
}
void Check_Client_Connect(void)
{
  uint8_t read_data[64];
  int i;
  Get_Wifi_IP();
  delay(200);
  while(Serial2.available())
  {
    Serial2.readBytes(read_data, 64);
  }
  for(i=0; i<64; i++)
  {
    if(read_data[i] == '1' && read_data[i+1] == '9' && read_data[i+2] == '2' && read_data[i+3] == '.' && read_data[i+4] == '1' && read_data[i+5] == '6' && read_data[i+6] == '8' && read_data[i+7] == '.' && read_data[i+8] == '4' && read_data[i+9] == '.')
    {
      Temp_IP[0] = read_data[i+10];
      Temp_IP[1] = read_data[i+11];
      Temp_IP[2] = read_data[i+12];
      Serial.print((Temp_IP[0]-48));
      Serial.print((Temp_IP[1]-48));
      Serial.println((Temp_IP[2]-48));
      Serial.println("GET IP OK");
      client = 1;
      client2 = 1;
      break;
    }
  }
}
void Check_Client_Connect2(void)
{
  uint8_t read_data[64];
  int i;
  /*
  Set_Wifi_TCP_Server();
  delay(1000); 
  while(Serial2.available())
  {
    Serial.write(Serial2.read());
  }
  */
  Set_Wifi_TCP_Client();
  delay(1000); 
  while(Serial2.available())
  {
    Serial2.readBytes(read_data, 64);
  }
 for(i=0; i<64; i++)
  {
    if((read_data[i] == 'L' && read_data[i+1] == 'i' && read_data[i+2] == 'n' && read_data[i+3] == 'k'))
    {
      Serial.println("Link");
      Wifi_Connect = 1;
      break;
    }
    if((read_data[i] == 'A' && read_data[i+1] == 'L' && read_data[i+2] == 'R' && read_data[i+3] == 'E'))
    {
      Serial.println("ALREDY");
      Wifi_Connect = 1;
      break;
    }
  }
  //
  //client2 = 0;
}
