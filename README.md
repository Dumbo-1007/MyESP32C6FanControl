### fan control system
This is an example of a fan control system, the project is based on the ESP-IDF development environment, combined with the FreeRTOS real-time operating system and fuzzy PID control algorithm, to implement a closed-loop temperature control fan system.
- Collect environmental data through DHT11 temperature and humidity sensor, dynamically adjust PID parameters by fuzzy logic, and control through PWM output
The speed of the fan is finally displayed in real time on the OLED display, and the temperature and humidity, error value and control output are displayed in real time to achieve automatic adjustment of room temperature
and use the wifi module to observe the data in the self-built LAN with the HTTP protocol.
