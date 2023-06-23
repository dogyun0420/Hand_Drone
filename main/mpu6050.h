void Cal_MPU6050(void);
void Calc_MPU6050_Accel(void);
void Calc_MPU6050_Gyro(void);
void Calc_SANGBO_Filter(void);
void Calc_DualPID(void);
void Read_MPU6050(void);
void init_MPU6050(void);
void init_YPR(void);
void init_Micro_Trig(void);

void Calc_Micro_Trig(void);

extern float roll_target_angle ;
extern float pitch_target_angle;
extern float yaw_target_angle;
extern float dt;

//extern float Roll_stabilize_kp, Roll_stabilize_ki, Roll_rate_kp, Roll_rate_ki;
//extern float Pitch_stabilize_kp, Pitch_stabilize_ki, Pitch_rate_kp, Pitch_rate_ki;
//extern float Yaw_stabilize_kp, Yaw_stabilize_ki, Yaw_rate_kp, Yaw_rate_ki;

struct cal_motor{ float roll, pitch, yaw; };
extern cal_motor Cal_Motor;

struct MPU6050_raw{ int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; };
extern MPU6050_raw MPU6050_Raw;

struct MPU6050_sum{ float AcX, AcY, AcZ, GyX, GyY, GyZ; };
extern MPU6050_sum MPU6050_Sum;

struct MPU6050_base{ float AcX, AcY, AcZ, GyX, GyY, GyZ; };
extern MPU6050_base MPU6050_Base;

struct target_angle{ float roll = 0.0, pitch = 0.0, yaw = 0.0; };
extern target_angle Target_Angle;

struct accel_ypr{ float x, y, z; };
extern accel_ypr Accel_YPR;

struct gyro_ypr{ float x, y, z; };
extern gyro_ypr Gyro_YPR;

struct sangbo_filtered{ float x, y, z; };
extern sangbo_filtered Sangbo_Filtered;
  
struct sangbo_temp{ float x, y, z; };
extern sangbo_temp Sangbo_Temp;

struct first_hovering{ float roll, pitch, yaw; };
extern first_hovering First_Hovering;

//X축 : pitch P1 = 1.75 || I1 = 0.00 || P2 = 1.75 || I2 = 0.00
//Y축 : roll = 2.40 || I1 = 0.00 || P2 = 2.40 || I2 = 0.00

struct dualpid_roll{ float angle_in , rate_in, stabilize_kp = 1.00, stabilize_ki = 0.00, rate_kp = 1.00, rate_ki = 0.00,   //P1||I1||P2||I2
                     stabilize_iterm, rate_iterm; };
extern dualpid_roll DualPID_roll;

struct dualpid_pitch{ float angle_in, rate_in, stabilize_kp = 1.00, stabilize_ki = 0.00, rate_kp = 1.00, rate_ki = 0.00,
                     stabilize_iterm, rate_iterm; };
extern dualpid_pitch DualPID_pitch;

struct dualpid_yaw { float angle_in, rate_in, stabilize_kp = 1.00, stabilize_ki = 0.00, rate_kp = 1.00, rate_ki = 0.00,
                     stabilize_iterm, rate_iterm; };
extern dualpid_yaw DualPID_yaw;
