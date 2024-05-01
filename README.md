# Arduino Motor Control with PID

## Overview
This project uses an Arduino to control motor speed and direction based on input from ultrasonic and analog sensors. It applies PID (Proportional, Integral, Derivative) control techniques to maintain a specified orientation and distance from objects.

## Hardware Requirements
- Arduino Uno or similar microcontroller
- 2 x Motors with motor driver
- 2 x Analog distance sensors
- 1 x Ultrasonic distance sensor (HC-SR04)
- Jumper wires
- Breadboard

## Setup
1. Connect the motor pins to the Arduino digital pins as defined in the code.
2. Attach the analog sensors to the analog pins.
3. Set up the ultrasonic sensor with one pin for trigger and one for echo.
4. Ensure the motor driver is connected correctly to both the motors and the Arduino.

## Software
The `arduino_code.ino` file contains all necessary code to implement the PID control. Load this file into the Arduino IDE, compile it, and upload it to your Arduino board.

### Key Components:
- **PID Constants**: Adjust KP, KI, KD for tuning the PID response.
- **Safety Checks**: Basic checks to prevent the motors from running under unsafe conditions.

## Usage
Once the code is uploaded, the system will automatically adjust the motor speeds based on the distance measurements from the sensors. Fine-tune the PID settings as necessary to achieve desired responsiveness.

## Contributing
Feel free to fork this project, make changes, and submit pull requests if you have improvements or fixes.

## License
This project is released under the MIT License.
