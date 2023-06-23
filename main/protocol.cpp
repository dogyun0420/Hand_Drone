#include "main.h"

motor_engine Motor_Engine;
motor_direction Motor_Direction;

extern volatile unsigned long timer0_millis;

bool Motor_Controllable_Flag = OFF;
bool Drone_Cal_Need_Flag = OFF;

uint32_t standard_ms, trigger_ms;

void Motor_Command(void)
{
  switch(Motor_Engine)
  {
    case on_state:
      if(Motor_Controllable_Flag == ON){
        switch(Motor_Direction){
          case up_state:
          Serial.println("Rising");

          if((standard_ms - trigger_ms) % THROTTLE_Period_ms == 0){
            throttle = throttle + 5;
          }

          Drone_No_Direction();
          break;
  
          case down_state:
          Serial.println("Falling");

          if((standard_ms - trigger_ms) % THROTTLE_Period_ms == 0){
            throttle = throttle - 5;
          }

          Drone_No_Direction();
          break;
  
          case right_state:
          Serial.println("Turning Right");
          //throttle = THROTTLE_IDLE;

          Target_Angle.roll = 0;
          Target_Angle.pitch = -5;
          break;
  
          case left_state:
          Serial.println("Turning Left");
          //throttle = THROTTLE_IDLE;

          Target_Angle.roll = 0;
          Target_Angle.pitch = 5;
          break;
  
          case front_state:
          Serial.println("Advancing");
          //throttle = THROTTLE_IDLE;

          Target_Angle.roll = 5;
          Target_Angle.pitch = 0;
          break;
  
          case back_state:
          Serial.println("Reversing");
          //throttle = THROTTLE_IDLE;

          Target_Angle.roll = -5;
          Target_Angle.pitch = 0;
          break;

          case 0:
          Serial.println("Hoverling");
         // throttle = throttle;
          Drone_No_Direction();
          break;
        }
      }

      else{
        throttle = THROTTLE_IDLE;
        Drone_No_Direction();
      }
    break;

    default:
      Motor_Controllable_Flag = OFF;
      throttle = THROTTLE_IDLE;      
      Drone_No_Direction();      
    break;
  }
}

void Wifi_Command(void)
{
  if(WifiData[0] == 'E' && WifiData[1] == 'T')  {Motor_Engine = off_state;                            Serial.println("Engine -> OFF");        }
  if(WifiData[0] == 'C' && WifiData[1] == 'A')  {Motor_Engine = on_state; Drone_Cal_Need_Flag = ON;   Serial.println("Engine -> ON");         }
  if(WifiData[0] == 'S' && WifiData[1] == 'T')  {Motor_Controllable_Flag = ON;                        Serial.println("Controllable -> ON");   }
  if(WifiData[0] == 'N' && WifiData[1] == 'O')  {Motor_Controllable_Flag = ON; Motor_Direction = 0;  Serial.println("Controllable -> OFF");  }

  if(Motor_Controllable_Flag == ON){
    if(WifiData[0] == 'U' && WifiData[1] == 'P')  {Motor_Direction = up_state; trigger_ms = millis();   Serial.println("Up -> ON");             }
    if(WifiData[0] == 'D' && WifiData[1] == 'O')  {Motor_Direction = down_state; trigger_ms = millis(); Serial.println("Down -> ON");           }
    if(WifiData[0] == 'R' && WifiData[1] == 'R')  {Motor_Direction = right_state;                       Serial.println("Right -> ON");          }
    if(WifiData[0] == 'L' && WifiData[1] == 'L')  {Motor_Direction = left_state;                        Serial.println("Left -> ON");           }
    if(WifiData[0] == 'F' && WifiData[1] == 'F')  {Motor_Direction = front_state;                       Serial.println("Front -> ON");          }
    if(WifiData[0] == 'B' && WifiData[1] == 'A')  {Motor_Direction = back_state;  
    if(WifiData[0] == 'H' && WifiData[1] == 'T')  {Motor_Direction = heart_state;                        
    Serial.println("Heart -> ON");   }Serial.println("Back -> ON");           }
  }
  
  if(WifiData[0] == 'D' && WifiData[1] == 'E' && WifiData[2] == 'O' && WifiData[3] == 'N')  {Debug = TRUE; Serial.println("Debug -> ON");    }

  if(Debug == TRUE){
    if(WifiData[0] == 'D' && WifiData[1] == 'E' && WifiData[2] == 'O' && WifiData[3] == 'F' && WifiData[4] == 'F')  {Debug = FALSE; Serial.println("Debug -> OFF");    }
    if(WifiData[0] == 'T' && WifiData[1] == 'H'){
      
      THROTTLE_IDLE = (WifiData[2] - 0x30) * 100;
      THROTTLE_IDLE += (WifiData[3] - 0x30) * 10;
      THROTTLE_IDLE += (WifiData[4] - 0x30) * 1;
      THROTTLE_IDLE += (double)(WifiData[6] - 0x30) / (double)10;
      THROTTLE_IDLE += (double)(WifiData[7] - 0x30) / (double)100;
      
      Serial.print("THROTTLE_IDLE -> ");
      Serial.println(THROTTLE_IDLE); 
    }

    if(WifiData[0] == 'R' && WifiData[1] == 'P' && WifiData[2] == '1' && WifiData[3] == 'P'){
      Drone_Cal_Need_Flag = ON;
      DualPID_roll.stabilize_kp = (WifiData[4] - 0x30) * 1;
      DualPID_roll.stabilize_kp += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_roll.stabilize_kp += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'R' && WifiData[1] == 'P' && WifiData[2] == '1' && WifiData[3] == 'I'){
      Drone_Cal_Need_Flag = ON;
      DualPID_roll.stabilize_ki = (WifiData[4] - 0x30) * 1;
      DualPID_roll.stabilize_ki += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_roll.stabilize_ki += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'R' && WifiData[1] == 'P' && WifiData[2] == '2' && WifiData[3] == 'P'){
      Drone_Cal_Need_Flag = ON;
      DualPID_roll.rate_kp = (WifiData[4] - 0x30) * 1;
      DualPID_roll.rate_kp += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_roll.rate_kp += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'R' && WifiData[1] == 'P' && WifiData[2] == '2' && WifiData[3] == 'I'){
      Drone_Cal_Need_Flag = ON;
      DualPID_roll.rate_ki = (WifiData[4] - 0x30) * 1;
      DualPID_roll.rate_ki += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_roll.rate_ki += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'P' && WifiData[1] == 'P' && WifiData[2] == '1' && WifiData[3] == 'P'){
      Drone_Cal_Need_Flag = ON;
      DualPID_pitch.stabilize_kp = (WifiData[4] - 0x30) * 1;
      DualPID_pitch.stabilize_kp += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_pitch.stabilize_kp += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'P' && WifiData[1] == 'P' && WifiData[2] == '1' && WifiData[3] == 'I'){
      Drone_Cal_Need_Flag = ON;
      DualPID_pitch.stabilize_ki = (WifiData[4] - 0x30) * 1;
      DualPID_pitch.stabilize_ki += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_pitch.stabilize_ki += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'P' && WifiData[1] == 'P' && WifiData[2] == '2' && WifiData[3] == 'P'){
      Drone_Cal_Need_Flag = ON;
      DualPID_pitch.rate_kp = (WifiData[4] - 0x30) * 1;
      DualPID_pitch.rate_kp += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_pitch.rate_kp += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'P' && WifiData[1] == 'P' && WifiData[2] == '2' && WifiData[3] == 'I'){
      Drone_Cal_Need_Flag = ON;
      DualPID_pitch.rate_ki = (WifiData[4] - 0x30) * 1;
      DualPID_pitch.rate_ki += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_pitch.rate_ki += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'Y' && WifiData[1] == 'P' && WifiData[2] == '1' && WifiData[3] == 'P'){
      Drone_Cal_Need_Flag = ON;
      DualPID_yaw.stabilize_kp = (WifiData[4] - 0x30) * 1;
      DualPID_yaw.stabilize_kp += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_yaw.stabilize_kp += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'Y' && WifiData[1] == 'P' && WifiData[2] == '1' && WifiData[3] == 'I'){
      Drone_Cal_Need_Flag = ON;
      DualPID_yaw.stabilize_ki = (WifiData[4] - 0x30) * 1;
      DualPID_yaw.stabilize_ki += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_yaw.stabilize_ki += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'Y' && WifiData[1] == 'P' && WifiData[2] == '2' && WifiData[3] == 'P'){
      Drone_Cal_Need_Flag = ON;
      DualPID_yaw.rate_kp = (WifiData[4] - 0x30) * 1;
      DualPID_yaw.rate_kp += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_yaw.rate_kp += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'Y' && WifiData[1] == 'P' && WifiData[2] == '2' && WifiData[3] == 'I'){
      Drone_Cal_Need_Flag = ON;
      DualPID_yaw.rate_ki = (WifiData[4] - 0x30) * 1;
      DualPID_yaw.rate_ki += (double)(WifiData[6] - 0x30) / (double)10;
      DualPID_yaw.rate_ki += (double)(WifiData[7] - 0x30) / (double)100;
    }

    if(WifiData[0] == 'S' && WifiData[1] == 'T' && WifiData[2] == 'A'){
      Drone_Cal_Need_Flag = ON;
      Serial.print("THROTTLE_IDLE -> ");
      Serial.println(THROTTLE_IDLE);
      
      Serial.print("Roll_1P -> "); Serial.print(DualPID_roll.stabilize_kp); Serial.print("Roll_1I -> "); Serial.print(DualPID_roll.stabilize_ki);
      Serial.print("Roll_2P -> "); Serial.print(DualPID_roll.rate_kp); Serial.print("Roll_2I -> "); Serial.println(DualPID_roll.rate_ki);

      Serial.print("Pitch_1P -> "); Serial.print(DualPID_pitch.stabilize_kp); Serial.print("Pitch_1I -> "); Serial.print(DualPID_pitch.stabilize_ki);
      Serial.print("Pitch_2P -> "); Serial.print(DualPID_pitch.rate_kp); Serial.print("Pitch_2I -> "); Serial.println(DualPID_pitch.rate_ki);

      Serial.print("Yaw_1P -> "); Serial.print(DualPID_yaw.stabilize_kp); Serial.print("Yaw_1I -> "); Serial.print(DualPID_yaw.stabilize_ki);
      Serial.print("Yaw_2P -> "); Serial.print(DualPID_yaw.rate_kp); Serial.print("Yaw_2I -> "); Serial.println(DualPID_yaw.rate_ki);
    }
  }
    
}
