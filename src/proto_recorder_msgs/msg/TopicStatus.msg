# Topic status information
string name
string type
float64 rate

# Rate status relative to expected range
uint8 RATE_STATUS_NORMAL = 0
uint8 RATE_STATUS_UNKNOWN = 1
uint8 RATE_STATUS_TOO_LOW = 2
uint8 RATE_STATUS_TOO_HIGH = 3
uint8 RATE_STATUS_NO_MESSAGES = 4
uint8 rate_status