#include "main.h"
float motorA_speed, motorB_speed, motorC_speed, motorD_speed;
float throttle;

void init_Motor_Standby(){
  DDRB=0x20;
  TCCR1A=0x82; 
  TCCR1B=0x1A;                          //FAST PWM / 16MHz/8분주=0.5usec
  OCR1A = THROTTLE_MIN;
  ICR1=FREQ;                            //0.5usec*(1+199)=10kHz
  delay(100);

  DDRE= 0x08;
  TCCR3A = 0x82;
  TCCR3B = 0x1A;
  OCR3A = THROTTLE_MIN;    
  ICR3=FREQ;
  delay(100);
    
  DDRH= 0x08;
  TCCR4A = 0x82;
  TCCR4B = 0x1A;
  OCR4A = THROTTLE_MIN;    
  ICR4=FREQ;
  delay(100);
    
  DDRL= 0x08;
  TCCR5A = 0x82;
  TCCR5B = 0x1A;
  OCR5A = THROTTLE_MIN;  
  ICR5=FREQ;
  delay(100);
}

void Calc_Motor_Speed(){

  //모터 속도 보정(호버링)// 
  motorA_speed = (throttle == 0) ? 0 : throttle + Cal_Motor.yaw + Cal_Motor.roll - Cal_Motor.pitch;//10
  motorB_speed = (throttle == 0) ? 0 : throttle - Cal_Motor.yaw - Cal_Motor.roll - Cal_Motor.pitch;//5
  motorC_speed = (throttle == 0) ? 0 : throttle + Cal_Motor.yaw - Cal_Motor.roll + Cal_Motor.pitch;//35
  motorD_speed = (throttle == 0) ? 0 : throttle - Cal_Motor.yaw + Cal_Motor.roll + Cal_Motor.pitch;//40

  if(throttle > THROTTLE_MAX) { throttle = THROTTLE_MAX; }

  if(throttle < THROTTLE_MIN) { throttle = THROTTLE_MIN; }  

  //모터 스피드 범위 지정 
  if(motorA_speed < 0){ motorA_speed = 0; }
  if(motorA_speed > 320){ motorA_speed = 320; }
  if(motorB_speed < 0){ motorB_speed = 0; }
  if(motorB_speed > 320){ motorB_speed = 320; }
  if(motorC_speed < 0){ motorC_speed = 0; }
  if(motorC_speed > 320){ motorC_speed = 320; }
  if(motorD_speed < 0){ motorD_speed = 0; }
  if(motorD_speed > 320){ motorD_speed = 320; }
}
bool Emergency_Drone(){
  if(Sangbo_Filtered.x > 40 || Sangbo_Filtered.x < -40 || Sangbo_Filtered.y > 40 || Sangbo_Filtered.y < -40){
    Motor_Engine = OFF;
    return TRUE;
  }
  else{
    return FALSE;
  }
}

void Drone_No_Direction(){
  /*
  Target_Angle.roll = 0;
  Target_Angle.pitch = 0;  
  Target_Angle.yaw = 0; 
  */
  Target_Angle.roll = First_Hovering.roll;
  Target_Angle.pitch = First_Hovering.pitch;
  Target_Angle.yaw = First_Hovering.yaw;
}

void Update_Motor_Speed(){
  Motor_Command();
  
  OCR5A = motorB_speed;
  OCR4A = motorC_speed;
  OCR3A = motorA_speed;
  OCR1A = motorD_speed;
  
  //팬끄기
  if((Motor_Engine == OFF) || (Emergency_Drone() == TRUE)){
    Drone_No_Direction();
    Motor_Direction = 0;
    throttle = 0;
    OCR5A = 0;   // PORT 10 
    OCR4A = 0;   // PORT 3 
    OCR3A = 0;   // PORT 9
    OCR1A = 0;   // PORT 11
  }
}
