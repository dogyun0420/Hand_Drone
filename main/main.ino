#include "main.h"
//X축 : pitch P1 = 1.75 || I1 = 0.00 || P2 = 1.75 || I2 = 0.00
//Y축 : roll = 2.40 || I1 = 0.00 || P2 = 2.40 || I2 = 0.00
//z축 : yaw
//-----------------------------------------------------------------------------------------------------
bool Debug = FALSE;
float THROTTLE_IDLE = 210;
float THROTTLE_MAX = 350;
float THROTTLE_MIN = 0;

void setup() {
  Serial.begin(115200);                                           //아두이노 및 PC 간 디버깅
  Serial2.begin(9600);///////////////////////////////////////삭제요망
  init_MPU6050();                                                 //MPU6050 모듈 초기화
  Cal_MPU6050();                                                  //MPU6050 모듈 초기값 CAL
  init_Micro_Trig();                                              //micro 단위 트리거 시점
  init_YPR();                                                     //Roll, Pitch, Yaw 초기값 설정
  init_Motor_Standby();                                           //모터(PWM) 초기화
  init_Wifi();                                                    //와이파이 초기화
}
uint8_t msp_packet[11];

void loop() {
  int cnt=0;
  char msp_data=0;

  while(1){    
    if(Serial2.available()){    
      msp_data=Serial2.read();  
      if(msp_data=='$') cnt=0;
      else cnt++;
      msp_packet[cnt]=msp_data;
      if(cnt==10){
        msp_print();      
      }
    }
  } 
}

void msp_print(){
  for(int i=0;i<=4;i++){
    Serial.print((char)msp_packet[i]);
    Serial.print(msp_packet[i]);
  }  
  Serial.print('\t');
  Serial.print(" Roll:");
  Serial.print((char)msp_packet[5]);
  Serial.print(msp_packet[5]);
  Serial.print(" Pitch:");
  Serial.print((char)msp_packet[6]);
  Serial.print(msp_packet[6]);
  Serial.print(" Yaw:");
  Serial.print((char)msp_packet[7]);  
  Serial.print(msp_packet[7]);
  Serial.print(" Throttole:");
  Serial.print((char)msp_packet[8]); 
  Serial.print(msp_packet[8]);
  Serial.print(" Aux:");
  Serial.print((char)msp_packet[9]); 
  Serial.print(msp_packet[9]);
  Serial.print(" 10:");
  Serial.print((char)msp_packet[10]); 
  Serial.println(msp_packet[10]);



   
  standard_ms = millis();                                         //상승 및 하강의 팬속도 참고 시간

  //Motor_Engine = on_state; Drone_Cal_Need_Flag = ON;
  
  Wifi_Checking();                                                //와이파이 수신 명령 체크
  Read_MPU6050();                                                 //MPU6050 모듈값 읽기
  
  Calc_MPU6050_Accel();                                           //MPU6050의 가속도값으로 회전각 계산
  Calc_MPU6050_Gyro();                                            //MPU6050의 자이로값으로 회전각 계산
  Calc_Micro_Trig();                                              //micro 단위 시간 구하기
  Calc_SANGBO_Filter();                                           //MPU6050의 가속도/자이로값으로 상보필터링

  Calc_DualPID();                                                 //상보필터링 값으로 이중 PID 계산
  Calc_Motor_Speed();                                             //이중 PID 값으로 모터값 계산

  if(Drone_Cal_Need_Flag == ON){
    init_Motor_Standby();                                           //모터(PWM) 초기화
    init_MPU6050();                                                 //MPU6050 모듈 초기화
    Cal_MPU6050();                                                  //MPU6050 모듈 초기값 CAL
    init_Micro_Trig();                                              //micro 단위 트리거 시점
    init_YPR();                                                     //Roll, Pitch, Yaw 초기값 설정    
    
    Drone_Cal_Need_Flag = OFF;
    
  }  
  Update_Motor_Speed();                                           //계산된 모터값을 드론에 적용
  /*
  Serial.print(DualPID_pitch.stabilize_iterm);
  Serial.print(":");
  Serial.print(DualPID_pitch.rate_iterm);
  Serial.print(":");
  Serial.print(MPU6050_Raw.AcY);
  Serial.print(":");
  Serial.println(Cal_Motor.pitch);
 */
  /*
  if(Drone_Cal_Need_Flag == ON){
    init_Motor_Standby();                                           //모터(PWM) 초기화
    init_MPU6050();                                                 //MPU6050 모듈 초기화
    Cal_MPU6050();                                                  //MPU6050 모듈 초기값 CAL
    init_Micro_Trig();                                              //micro 단위 트리거 시점
    init_YPR();                                                     //Roll, Pitch, Yaw 초기값 설정    
    
    Drone_Cal_Need_Flag = OFF;
    
  }  */
}
