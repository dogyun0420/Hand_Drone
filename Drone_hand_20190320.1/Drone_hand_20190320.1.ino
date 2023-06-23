#include <I2Cdev.h>
//#include <SoftwareSerial.h>
#include "appwifi.h"
#include "mpu_sensor.h"
#include "LCD_ST7735.h"

uint8_t client = 0;
uint8_t client2 = 0;


void setup() 
{
  Serial.begin(115200);
  Serial2.begin(9600);
  Init_LCD();   
  Init_mpu6050(); 
  Init_Wifi();
}
void loop() 
{
  //AT_CMD();
  Axis_LCD_Display();
}
