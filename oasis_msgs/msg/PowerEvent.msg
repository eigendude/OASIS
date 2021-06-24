################################################################################
# Event occurring while powering a device on or off
################################################################################

# Timestamp and frame_id
std_msgs/Header header

# The machine hosting the device being powered
string system

# Path to the device being powered
string device

# The new power state of type PowerMode
string power_mode
