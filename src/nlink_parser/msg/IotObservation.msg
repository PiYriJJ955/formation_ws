# One timestamped directed observation from one IoT module to another vehicle/module.
std_msgs/Header header
uint32 device_time
uint32 frame_sequence
uint8 source_vehicle
uint8 target_vehicle
uint32 source_uid
uint32 target_uid
float64 range_raw
float64 azimuth_raw
float64 range
float64 azimuth
float64[4] covariance
float32 fp_rssi
float32 rx_rssi
bool range_valid
bool azimuth_valid
bool timestamp_synced
