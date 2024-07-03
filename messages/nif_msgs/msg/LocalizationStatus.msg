# Localization status message for ros2
# Author : Daegyu Lee @ KAIST
# Date : 2021.10.01

builtin_interfaces/Time stamp

# Localization_status
float64 bottom_error
float64 top_error
float64 diff
float64 bottom_weight
float64 top_weight
string status

float64 uncertainty

# Sensor init
bool top_initialized
bool bottom_initialized

# Localization_code
uint8 BEST_STATUS = 61
uint8 SENSOR_FUSION = 62
uint8 VECTORNAV_INS = 63
uint8 SECONDARY_SENSOR_FUSION = 71
uint8 ONLY_TOP = 72
uint8 ONLY_BOTTOM = 73
uint8 MONZA_TEMP_GPS_HIGH_ERROR = 80
uint8 GPS_HIGH_ERROR = 150
uint8 HEADING_ERROR = 215
uint8 UNCERTAINTY_TOO_HIGH = 225
uint8 NO_CONVERGED = 235
uint8 NO_SENSOR_INITIALIZED = 255

uint8 localization_status_code 0

uint8 pos_type_0 0
uint8 pos_type_1 0

float64 heading
uint8 top_num_of_satellite
uint8 bot_num_of_satellite
uint8 ublox_num_of_satellite

# Heading Correction Source
uint8 FORCED_HEADING = 50
uint8 VNINS = 1
uint8 NOVATEL_INS = 2
uint8 NOVATEL_BESTVEL = 11
uint8 NOVATEL_HEADING2 = 21
uint8 KINEMATIC = 22
uint8 HEADING_INVALID = 100

uint8 heading_source

bool vectornav_connection