# SiLog
Low latency vibrations logger using accelo-gyro MEMS, based on Arduino framework and designed for Maple Mini.</br>
The project might be ported to Arduino Uno or similar with some effort because of lack of certain hardware features of Uno (second Serial port, fewer interrupt channels, etc.).

The firmware itself is based on example project "LowLatencyLoggerMPU6050" for excellent Arduino FAT16/FAT32 Library SdFat  (<https://github.com/greiman/SdFat>).</br>

To compile the project two more libraries are required:</br> 
    1. RTCLib from NeiroNx (<https://github.com/NeiroNx/RTCLib>)</br>
    2. MicroNMEA from stevemarple (<https://github.com/stevemarple/MicroNMEA>)
