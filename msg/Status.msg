# Normal vs setting in the UAM manual.
uint16 NORMAL=0
uint16 SETTING=1
uint16 operating_mode
uint16 area_number
# If the laser is reporting an error or not.
bool error_state
# The error code the laser is reporting.
uint16 error_code
# Does the laser report that it is locked out.
bool lockout_state
# For the OSSD states and warning states this is possible state
# 0: Off (No detection)
# 1: On (Detection)
bool ossd1_state
bool ossd2_state
bool warning1_state
bool warning2_state
# Optical Window contamination warning
# 0: Off (Not contaminated)
# 1: On (Contaminated)
bool optical_window_contaminated