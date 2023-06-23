#include "main.h"

void Set_Wifi_Mode_AP(void){ Serial2.println("AT+CWMODE=3"); }
void Set_Wifi_Mux(void){ Serial2.println("AT+CIPMUX=1"); }
void Set_Wifi_SSID_AP(void){
  Serial2.print("AT+CWSAP=");
  Serial2.write('"');
  Serial2.print("DRONE1");
  Serial2.write('"');
  Serial2.write(',');
  Serial2.write('"');
  Serial2.print("00700070");
  Serial2.write('"');
  Serial2.write(',');
  Serial2.write('1');
  Serial2.write(',');
  Serial2.write('2');
  Serial2.write('\r');
  Serial2.write('\n');
}
void Set_Wifi_TCP_Server(void){
  Serial2.print("AT+CIPSERVER=1");
  Serial2.write('\r');
  Serial2.write('\n');
}
void Set_CIPSTO(void){
   Serial2.print("AT+CIPSTO=6000");
   Serial2.write('\r');
   Serial2.write('\n');
}
