# constants
int32 INIT_STATE = 100
int32 STANDBY_STATE = 200
int32 READY_STATE = 300
int32 EMERGENCY_STATE = 400
int32 FAILURE_STATE = 500
int32 SHUTDOWN_STATE = 600


# state of the component
int32 state
# Control mode
string control_mode
# Flag active when the hand position has been initialized
bool hand_initialized
# desired control loop frecuency
float32 desired_freq
# real frecuency 
float32 real_freq
# Description of the state
string state_description
# [temp_motor, temp_puck]
float32[2] temp_f1
# [temp_motor, temp_puck]
float32[2] temp_f2
# [temp_motor, temp_puck]
float32[2] temp_f3
# [temp_motor, temp_puck]
float32[2] temp_spread
