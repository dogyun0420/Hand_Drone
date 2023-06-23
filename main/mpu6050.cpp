#include "main.h"

const int MPU_addr = 0x68;

float dt;
uint32_t t_now;
uint32_t t_prev;

cal_motor Cal_Motor;
MPU6050_raw MPU6050_Raw;
MPU6050_sum MPU6050_Sum;
MPU6050_base MPU6050_Base;
target_angle Target_Angle;
accel_ypr Accel_YPR;
gyro_ypr Gyro_YPR;
dualpid_roll DualPID_roll;
dualpid_pitch DualPID_pitch;
dualpid_yaw DualPID_yaw;
sangbo_filtered Sangbo_Filtered;
sangbo_temp Sangbo_Temp;
first_hovering First_Hovering;

void init_Micro_Trig(){ t_prev = micros(); }

void Calc_Micro_Trig(){
  t_now = micros();
  dt = (t_now - t_prev) / 1000000.0;
  t_prev = t_now;
}

void Calc_MPU6050_Accel(){
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180 / 3.14159;

  accel_x = MPU6050_Raw.AcX - MPU6050_Base.AcX;
  accel_y = MPU6050_Raw.AcY - MPU6050_Base.AcY;
  accel_z = MPU6050_Raw.AcZ + (16384 - MPU6050_Base.AcZ);

  //accel_angle_y는 Roll각을 의미//
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  Accel_YPR.y = atan(-accel_x / accel_yz) * RADIANS_TO_DEGREES;

  //accel_angle_x는 Pitch값을 의미//
  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  Accel_YPR.x = atan(accel_y / accel_xz) * RADIANS_TO_DEGREES;

  Accel_YPR.z = 0; //중력 가속도(g)의 방향과 정반대의 방향을 가리키므로 가속도 센서를 이용해서는 회전각을 계산할 수 없다.//
  /*
  Serial.print("Sangbo Filter -> ");
  Serial.print(Sangbo_Filtered.x);
  Serial.print(" : ");
  Serial.print(Sangbo_Filtered.y);
  Serial.print(" : ");
  Serial.println(Sangbo_Filtered.z);  
  */
}

void Calc_MPU6050_Gyro(){
  const float GYROXYZ_TO_DEGREES_PER_SEC = 131;

  Gyro_YPR.x = (MPU6050_Raw.GyX - MPU6050_Base.GyX) / GYROXYZ_TO_DEGREES_PER_SEC;
  Gyro_YPR.y = (MPU6050_Raw.GyY - MPU6050_Base.GyY) / GYROXYZ_TO_DEGREES_PER_SEC;
  Gyro_YPR.z = (MPU6050_Raw.GyZ - MPU6050_Base.GyZ) / GYROXYZ_TO_DEGREES_PER_SEC;
}

void Calc_SANGBO_Filter(){
  const float ALPHA = 0.96;
  
  Sangbo_Temp.x = Sangbo_Filtered.x + Gyro_YPR.x * dt;
  Sangbo_Temp.y = Sangbo_Filtered.y + Gyro_YPR.y * dt;
  Sangbo_Temp.z = Sangbo_Filtered.z + Gyro_YPR.z * dt;

  //상보필터 값 구하기(가속도, 자이로 센서의 절충)//
  Sangbo_Filtered.x = ALPHA * Sangbo_Temp.x + (1.0-ALPHA) * Accel_YPR.x;
  Sangbo_Filtered.y = ALPHA * Sangbo_Temp.y + (1.0-ALPHA) * Accel_YPR.y;
  Sangbo_Filtered.z = Sangbo_Temp.z;
//Sangbo_Filtered.z = ALPHA * Sangbo_Temp.z;
  
  /*
  
  */
}

void init_YPR(){
  //초기 호버링의 각도를 잡아주기 위해서 Roll, Pitch, Yaw 상보필터 구하는 과정을 10번 반복한다.//
  if(Drone_Cal_Need_Flag == ON){
    First_Hovering.roll = 0;
    First_Hovering.pitch = 0;
    First_Hovering.yaw = 0;
    Sangbo_Filtered.x = 0;
    Sangbo_Filtered.y = 0;
    Sangbo_Filtered.z = 0;
    Serial.println("YPR -> reset");
  }

  for(int i=0; i<10; i++){
    Read_MPU6050();
    Calc_Micro_Trig();
    Calc_MPU6050_Accel();
    Calc_MPU6050_Gyro();
    Calc_SANGBO_Filter();

    First_Hovering.roll += Sangbo_Filtered.y;
    First_Hovering.pitch += Sangbo_Filtered.x;
    First_Hovering.yaw += Sangbo_Filtered.z;
    delay(100);
  }

  //평균값을 구한다.//
  First_Hovering.roll /= 10;
  First_Hovering.pitch /= 10;
  First_Hovering.yaw /= 10;

  //초기 타겟 각도를 잡아준다.//
  Target_Angle.roll = First_Hovering.roll;
  Target_Angle.pitch = First_Hovering.pitch;
  Target_Angle.yaw = First_Hovering.yaw;
}

//////////////////////
void dualPID(float target_angle, float angle_in, float rate_in, float stabilize_kp,
             float stabilize_ki, float rate_kp, float rate_ki, float &stabilize_iterm,
             float &rate_iterm, float &output
){
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;

  //이중루프PID알고리즘//
  angle_error = target_angle - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;
  stabilize_iterm += stabilize_ki * angle_error * dt; //안정화 적분항//

  desired_rate = stabilize_pterm;

  rate_error = desired_rate - rate_in;

  rate_pterm = rate_kp * rate_error; //각속도 비례항//
  rate_iterm += rate_ki * rate_error * dt; //각속도 적분항//

  output = rate_pterm + rate_iterm + stabilize_iterm; //최종 출력 : 각속도 비례항 + 각속도 적분항 + 안정화 적분항//
}
///////////////////////
void Calc_DualPID(){
  if(Drone_Cal_Need_Flag == ON){
    Sangbo_Filtered.x = 0;
    Sangbo_Filtered.y = 0;
    Sangbo_Filtered.z = 0;

    Gyro_YPR.x = 0;
    Gyro_YPR.y = 0;
    Gyro_YPR.z = 0;

    DualPID_roll.angle_in = 0;
    DualPID_pitch.angle_in = 0;
    DualPID_yaw.angle_in = 0;
    DualPID_roll.stabilize_iterm = 0;
    DualPID_roll.rate_iterm = 0;
    DualPID_pitch.stabilize_iterm = 0;
    DualPID_pitch.rate_iterm = 0;
    DualPID_yaw.stabilize_iterm = 0;
    DualPID_yaw.rate_iterm = 0;
        
    Serial.println("Each PID -> reset");
  }
  DualPID_roll.angle_in = Sangbo_Filtered.y;
  DualPID_roll.rate_in = Gyro_YPR.y;

  dualPID(Target_Angle.roll, DualPID_roll.angle_in, DualPID_roll.rate_in,
          DualPID_roll.stabilize_kp, DualPID_roll.stabilize_ki,
          DualPID_roll.rate_kp, DualPID_roll.rate_ki,
          DualPID_roll.stabilize_iterm, DualPID_roll.rate_iterm,
          Cal_Motor.roll);

  DualPID_pitch.angle_in = Sangbo_Filtered.x;
  DualPID_pitch.rate_in = Gyro_YPR.x;

  dualPID(Target_Angle.pitch, DualPID_pitch.angle_in, DualPID_pitch.rate_in,
          DualPID_pitch.stabilize_kp, DualPID_pitch.stabilize_ki,
          DualPID_pitch.rate_kp, DualPID_pitch.rate_ki,
          DualPID_pitch.stabilize_iterm, DualPID_pitch.rate_iterm,
          Cal_Motor.pitch);

  DualPID_yaw.angle_in = Sangbo_Filtered.z;
  DualPID_yaw.rate_in = Gyro_YPR.z;

  dualPID(Target_Angle.yaw, DualPID_yaw.angle_in, DualPID_yaw.rate_in,
          DualPID_yaw.stabilize_kp, DualPID_yaw.stabilize_ki,
          DualPID_yaw.rate_kp, DualPID_yaw.rate_ki,
          DualPID_yaw.stabilize_iterm, DualPID_yaw.rate_iterm,
          Cal_Motor.yaw);
}

void init_MPU6050(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void Read_MPU6050(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);  //I2C의 제어권을 이어간다//
  Wire.requestFrom(MPU_addr, 14, true);

  //가속도, 자이로 센서의 값을 읽어온다.//
  MPU6050_Raw.AcX = Wire.read() << 8|Wire.read();
  MPU6050_Raw.AcY = Wire.read() << 8|Wire.read();
  MPU6050_Raw.AcZ = Wire.read() << 8|Wire.read();
  MPU6050_Raw.Tmp = Wire.read() << 8|Wire.read();
  MPU6050_Raw.GyX = Wire.read() << 8|Wire.read();
  MPU6050_Raw.GyY = Wire.read() << 8|Wire.read();
  MPU6050_Raw.GyZ = Wire.read() << 8|Wire.read();
}

void Cal_MPU6050(){
  MPU6050_Sum.AcX = 0;
  MPU6050_Sum.AcY = 0;
  MPU6050_Sum.AcZ = 0;
  MPU6050_Sum.GyX = 0;
  MPU6050_Sum.GyY = 0;
  MPU6050_Sum.GyZ = 0;

  Read_MPU6050();

  //초기 보정값은 10번의 가속도 자이로 센서의 값을 받아 해당 평균값을 가진다.//
  for(int i=0; i<10; i++){
    Read_MPU6050();
    MPU6050_Sum.AcX += MPU6050_Raw.AcX;
    MPU6050_Sum.AcY += MPU6050_Raw.AcY;
    MPU6050_Sum.AcZ += MPU6050_Raw.AcZ;
    MPU6050_Sum.GyX += MPU6050_Raw.GyX;
    MPU6050_Sum.GyY += MPU6050_Raw.GyY;
    MPU6050_Sum.GyZ += MPU6050_Raw.GyZ;
    delay(100);
  }

  MPU6050_Base.AcX = MPU6050_Sum.AcX / 10;
  MPU6050_Base.AcY = MPU6050_Sum.AcY / 10;
  MPU6050_Base.AcZ = MPU6050_Sum.AcZ / 10;
  MPU6050_Base.GyX = MPU6050_Sum.GyX / 10;
  MPU6050_Base.GyY = MPU6050_Sum.GyY / 10;
  MPU6050_Base.GyZ = MPU6050_Sum.GyZ / 10;
}
