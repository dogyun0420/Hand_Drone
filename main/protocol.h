enum motor_engine{
  off_state,
  on_state
};
extern motor_engine Motor_Engine;

enum motor_direction{
  up_state = 1,
  down_state,
  right_state,
  left_state,
  front_state,
  back_state,
  heart_state
};
extern motor_direction Motor_Direction;

extern uint32_t standard_ms, trigger_ms;
extern bool Drone_Cal_Need_Flag;

void Wifi_Command(void);
void Motor_Command(void);
