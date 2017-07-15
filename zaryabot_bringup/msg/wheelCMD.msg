#wheel constant
uint8 LF_wheel = 0
uint8 RF_wheel = 1
uint8 LB_wheel = 2
uint8 RB_wheel = 3

#wheel Mode
uint8 MODE_EN    = 0
uint8 MODE_STOP  = 1
uint8 MODE_BRAKE = 2

#wheel Status
float64[] position
float64[] velocity
uint8[] mode
uint8[] jointid
