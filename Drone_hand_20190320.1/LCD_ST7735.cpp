#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include "Arduino.h"
#include "appwifi.h"
#include "mpu_sensor.h"
#include "LCD_ST7735.h"

#define TFT_CS     15
#define TFT_RST    14
#define TFT_DC     25

#define BLUE    0xF800
#define BLUE1   0xFC00
#define CYAN    0xFFE0
#define GREEN   0x07E0
#define YELLOW  0x07FF 
#define RED     0x001F
#define MAGENTA 0xF81F
#define BLACK   0x0000
#define WHITE   0xFFFF

#define SENSOR1 37
#define SENSOR2 36
#define SENSOR3 35
#define SENSOR4 34
#define SENSOR5 33


extern float X_Data;
extern float Y_Data;
extern float Z_Data;
extern uint8_t Wifi_Connect;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

void Init_LCD(void)
{
  // 포트 초기화
  pinMode(TFT_CS, OUTPUT);
  pinMode(TFT_RST, OUTPUT);

  // 센서 입력 설정
  pinMode(SENSOR1, INPUT);
  pinMode(SENSOR2, INPUT);
  pinMode(SENSOR3, INPUT);
  pinMode(SENSOR4, INPUT);
  pinMode(SENSOR5, INPUT);

  // 배경 디스플레이
  tft.initR(INITR_BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);

  // 드론 모양 디스플레이
 // drawtext(0,0,"* Drone with Arduino", WHITE,1);

  // 활살표 디스플레이
  tft.drawBitmap(30,50, drone, 64, 28, BLUE1);
  tft.drawBitmap(50,15, up, 22, 22, WHITE);
  tft.drawBitmap(52,85, down, 22, 22, WHITE);
  tft.drawBitmap(3,50, left, 22, 22, WHITE);
  tft.drawBitmap(100,50, right, 22, 22, WHITE);

  // 와이파이 연결 안됨 디스플레이
  //Disconnected_Display();
}
void drawtext(unsigned char xpos, unsigned char ypos, char *text, uint16_t color, uint8_t text_size) 
{
  tft.setCursor(xpos, ypos+32);
  tft.setTextColor(color);
  tft.setTextSize(text_size);
  tft.setTextWrap(true);
  tft.print(text);
}

bool Start_End_Flag = false;
bool Start_Flag = false;
unsigned char FlexSensor[5];  // 손가락 센서 감지
unsigned char AccelValue = 0;
unsigned char GravityValue = 0;

void Axis_LCD_Display(void)
{
  Read_Axis_From_Mpu6050();

  // 와이파이 연결에 따른 LCD 동작
  if(Wifi_Connect == 0)       Disconnected_Display();
  else if(Wifi_Connect == 1)  Axis_Data_Display();
  else if(Wifi_Connect == 2)  Power_Display();

  // 가속도 값 확인.
  AccelValue = AccelMoni(); 

  // 축 값 확인
  GravityValue = Gravity_Check();
    

  // 센서 체크
  if(digitalRead(SENSOR1))  FlexSensor[0] = 1;
  else                      FlexSensor[0] = 0;

  if(digitalRead(SENSOR2))  FlexSensor[1] = 1;
  else                      FlexSensor[1] = 0;

  if(digitalRead(SENSOR3))  FlexSensor[2] = 1;
  else                      FlexSensor[2] = 0;

  if(digitalRead(SENSOR4))  FlexSensor[3] = 1;
  else                      FlexSensor[3] = 0;

  if(digitalRead(SENSOR5))  FlexSensor[4] = 1;
  else                      FlexSensor[4] = 0;
  ///////////////////////////// 동작 내용 /////////////////////////////////////GD////
   //시동 ON 상태
   if(Start_Flag == true)
   {
      //엄지 펴진 상태
      if(FlexSensor[0] == 0)
      {
        //손날이 아래로 향할때
        if(GravityValue == 1)
        {
          //검지, 중지, 약지, 소지가 전부 굽혀졌을때 직진
          if((FlexSensor[1] == 1) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 1))
          {
            Set_Wifi_Send_Data("$FF#");
            Serial.println("$FF#");
      
            // 방향 디스플레이 표시
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
          //명령어 잘못됫을때
          else
          {
            Set_Wifi_Send_Data("$NO#");
            Serial.println("$NO#");
      
            // 방향 디스플레이 표시 안함.
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
        }
      
        //손날이 정면으로 향할때
        else if(GravityValue == 2)
        {
          //검지, 중지, 약지, 소지가 전부 굽혀졌을때 후진
          if((FlexSensor[1] == 1) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 1))
          {
            Set_Wifi_Send_Data("$BA#");
            Serial.println("$BA#");
      
            // 방향 디스플레이 표시
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
          //명령어 잘못됫을때
          else
          {
            Set_Wifi_Send_Data("$NO#");
            Serial.println("$NO#");
      
            // 방향 디스플레이 표시 안함.
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
        }
      
        //손바닥이 아래로 향할때 
        
        else
        {
          //검지만 폈을때 상승
          if((FlexSensor[1] == 0) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 1))
          {
            Set_Wifi_Send_Data("$UP#");
            Serial.println("$UP#");
      
            // 방향 디스플레이 표시 안함.
            tft.drawBitmap(50,15, up, 22, 22, GREEN);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
      
          //소지만 폈을때 하강
          if((FlexSensor[1] == 1) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 0))
          {
            Set_Wifi_Send_Data("$DO#");
            Serial.println("$DO#");
      
            // 방향 디스플레이 표시 안함.
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, GREEN);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
      
          //약지만 접었을때 왼쪽 수평이동
          if((FlexSensor[1] == 0) && (FlexSensor[2] == 0) && (FlexSensor[3] == 1) && (FlexSensor[4] == 0))
          {
            Set_Wifi_Send_Data("$LL#");
            Serial.println("$LL#");
      
            // 방향 디스플레이 표시
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, GREEN);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
      
          //소지만 접었을때 오른쪽 수평이동
          if((FlexSensor[1] == 0) && (FlexSensor[2] == 0) && (FlexSensor[3] == 0) && (FlexSensor[4] == 1))
          {
            Set_Wifi_Send_Data("$RR#");
            Serial.println("$RR#");
      
            // 방향 디스플레이 표시
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, GREEN);
          }
      
          //검지만 접었을때 직진
          if((FlexSensor[1] == 1) && (FlexSensor[2] == 0) && (FlexSensor[3] == 0) && (FlexSensor[4] == 0))
          {
            Set_Wifi_Send_Data("$FF#");
            Serial.println("$FF#");
      
            // 방향 디스플레이 표시
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
      
          //중지만 접었을때 후진
          if((FlexSensor[1] == 0) && (FlexSensor[2] == 1) && (FlexSensor[3] == 0) && (FlexSensor[4] == 0))
          {
            Set_Wifi_Send_Data("$BA#");
            Serial.println("$BA#");
      
            // 방향 디스플레이 표시
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }

          //검지 및 중지만 폈을때 하트
          if((FlexSensor[1] == 0) && (FlexSensor[2] == 0) && (FlexSensor[3] == 1) && (FlexSensor[4] == 1))
          {
            Set_Wifi_Send_Data("$HT#");
            Serial.println("$HT#");
      
            // 방향 디스플레이 표시
            tft.drawBitmap(50,15, up, 22, 22, GREEN);
            tft.drawBitmap(52,85, down, 22, 22, GREEN);
            tft.drawBitmap(3,50, left, 22, 22, GREEN);
            tft.drawBitmap(100,50, right, 22, 22, GREEN);
          }
          
  
          //명령어 잘못됫을때
          else
          {
            Set_Wifi_Send_Data("$NO#");
            Serial.println("$NO#");
      
            // 방향 디스플레이 표시 안함.
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
          
        }
      }
  
      //엄지 접어진 상태
      else
      {
        
        //상승
        if((FlexSensor[1] == 0) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 1))
          {
            Set_Wifi_Send_Data("$UP#");
            Serial.println("$UP#");
      
            // 방향 디스플레이 표시 안함.
            tft.drawBitmap(50,15, up, 22, 22, GREEN);
            tft.drawBitmap(52,85, down, 22, 22, WHITE);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }
          
          //하강
          if((FlexSensor[1] == 1) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 0))
          {
            Set_Wifi_Send_Data("$DO#");
            Serial.println("$DO#");
      
            // 방향 디스플레이 표시 안함.
            tft.drawBitmap(50,15, up, 22, 22, WHITE);
            tft.drawBitmap(52,85, down, 22, 22, GREEN);
            tft.drawBitmap(3,50, left, 22, 22, WHITE);
            tft.drawBitmap(100,50, right, 22, 22, WHITE);
          }

          if((FlexSensor[1] == 1) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 1))
        {
          Set_Wifi_Send_Data("$ET#");
          Serial.println("$ET#");
        }
        else
        {
          Set_Wifi_Send_Data("$NO#");
          Serial.println("$NO#");
        }

        
  
        // 방향 디스플레이 표시 안함.
        tft.drawBitmap(50,15, up, 22, 22, WHITE);
        tft.drawBitmap(52,85, down, 22, 22, WHITE);
        tft.drawBitmap(3,50, left, 22, 22, WHITE);
        tft.drawBitmap(100,50, right, 22, 22, WHITE);
  
        //비행 종료
        //Set_Wifi_Send_Data("$ET#");
      }
   }
   //시동 꺼진 상태
   else
   {
      //손바닥이 아래를 보고 있을때
      if(GravityValue == 0)
      {
        if((FlexSensor[0] == 0) && (FlexSensor[1] == 0) && (FlexSensor[2] == 1) && (FlexSensor[3] == 1) && (FlexSensor[4] == 0))
        {
          Start_Flag = true;
          Set_Wifi_Send_Data("$CA#");
          delay(1000); 
          Set_Wifi_Send_Data("$ST#");
        }
      }
   }
}

void Axis_Data_Display(void)
{
  char szPacket[50]={0,};  
  sprintf(szPacket, "* X:%.2f Y:%.2f",X_Data,Y_Data);
 // tft.fillRect(0,152,128,7,BLACK);
  tft.fillRect(0,0,128,7,BLACK);
 // drawtext(0,120, szPacket, WHITE,1);
  drawtext(0,0, szPacket, WHITE,1);
}
void Disconnected_Display(void)
{
  //tft.fillRect(0,152,128,7,BLACK);
  tft.fillRect(0,0,128,7,BLACK);
  //drawtext(0,120,"* DisConnected", WHITE,1);
  drawtext(0,0,"* DisConnected", WHITE,1);
}
void Power_Display(void)
{
  //tft.fillRect(0,152,128,7,BLACK);
  tft.fillRect(0,0,128,7,BLACK);
  //drawtext(0,120,"* Power Off", WHITE,1);
  drawtext(0,0,"* Link is not", WHITE,1);
}
