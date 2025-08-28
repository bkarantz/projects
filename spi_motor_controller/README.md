SPI motor controller made for the Waterloo Aerial Robotics Group (WARG) design team. The servo motor can be controlled
by rotating the potentioemter, which allows you to test the servos functionality and performance. 

The system consists of three four components:
- Potentiometer
- MCP3004 ADC chip
- STM32F072RB Nucleo board
- Servo motor

The potentiometer outputs an analog voltage value from 0 to 3.3 V. It is connected to channel 0 on the MCP3004,
which stores that value. the MCP3004 converts the analog voltage to a digital value, then sends the data over to
the STM32 via SPI. The microcontroller converts this digital value into a PWM signal which is used to control the
servo. 
