micro_sas

CAN Message IDs
0x23 - Steering angle Message - first 4 bytes are a signed 32 bit integer
0x230 - Configuration message - send DEADFACE as the 8 bytes and it will reset the internal angle to 0
0x231 - Error message - publishes any errors that the sensor is experiencing

