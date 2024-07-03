# Plannignstatus
# Author : Jaeyoung Kang @ KAIST
# Date : 2021.11.03

builtin_interfaces/Time stamp

uint8 planning_code 255
string status
bool lane_change false
float32 callback_delay 0.0

# Planning_code
uint8 STRAIGHT = 0
uint8 ACC = 10
uint8 ATTEMPT = 50
uint8 OVERTAKE_DONE = 100
uint8 STOP = 255
