# Proto recorder status message
std_msgs/Header header

# Hardware/machine identification
string hardware_id

# Recording status
bool is_recording
float64 recording_duration

# Overall status
uint8 ERROR_LEVEL_OK = 0
uint8 ERROR_LEVEL_WARN = 1
uint8 ERROR_LEVEL_ERROR = 2
uint8 error_level
string status_message

# Topic status monitoring
TopicStatus[] topic_statuses
