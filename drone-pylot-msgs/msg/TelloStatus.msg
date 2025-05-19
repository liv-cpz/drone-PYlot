# Acceleration from IMU
geometry_msgs/Vector3 acceleration

# Speed from IMU
geometry_msgs/Vector3 speed

# Altitude of the drone from sensor
int32 barometer
int32 distance_tof

# Percentage of battery left
uint8 battery

# Time the motors have been active in seconds
int32 flight_time

# Highest temperature in C
int32 highest_temperature
# Lowest temperature in C
int32 lowest_temperature
# Temperature in C
float32 temperature

# Pitch in degrees
int32 pitch
# Roll in degrees
int32 roll
# Yaw in degrees
int32 yaw

# Wi-Fi SNR
string wifi_snr