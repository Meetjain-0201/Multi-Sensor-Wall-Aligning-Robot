# Autonomous Docking Robot System

## Overview
This project implements an autonomous docking robot that can:
1. Locate a charging station wall
2. Align itself perpendicular to the wall using PID control
3. Make two-point contact with the charging station
4. Shut down upon successful docking

The system combines ultrasonic sensors for distance measurement and a BNO055 IMU for orientation detection, with PID controllers for precise movement control.

## Hardware Components
- **Microcontroller**: Arduino-compatible board
- **Sensors**:
  - 2x HC-SR04 Ultrasonic sensors (left and right)
  - Adafruit BNO055 9-DOF IMU
- **Motors**:
  - 2x DC motors with PWM speed control
  - Motor driver (L298N or similar)
- **Power**: Charging station with contact pins

## Pin Configuration
| Component       | Arduino Pin |
|-----------------|-------------|
| Right Ultrasonic Echo | 12         |
| Right Ultrasonic Trig | 3          |
| Left Ultrasonic Echo  | 4          |
| Left Ultrasonic Trig  | 9          |
| Left Motor PWM        | 5          |
| Left Motor Direction  | 8          |
| Right Motor PWM       | 6          |
| Right Motor Direction | 11         |
| Interrupt Pin         | 2          |
| BNO055 I2C           | SDA/SCL    |

## Docking Algorithm
1. **Wall Finding Stage**:
   - Robot rotates until IMU detects proper orientation (Q angle between 3-20°)
   - Uses differential wheel speeds for rotation

2. **Alignment Stage**:
   - Dual PID controllers maintain set distance (20cm) from wall
   - Independently adjusts left and right motors based on ultrasonic readings

3. **Contact Stage**:
   - Triggered by interrupt on pin 2
   - Moves forward at reduced speed (50 PWM)
   - Uses physical contact switch on pin 7 for final confirmation

4. **Shutdown**:
   - Stops all motors
   - Enters low-power state

## PID Tuning Parameters
```cpp
double kp = 24;  // Proportional gain
double ki = 0;   // Integral gain
double kd = 0;   // Derivative gain
int setPoint = 20; // Target distance in cm
