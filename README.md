# 🤖 Multi-Sensor Wall-Aligning Robot

---

## 🌐 Overview  
Autonomous wall aligning robot with four-stage precision alignment:  
1. Wall detection via IMU orientation  
2. PID-controlled perpendicular alignment  
3. Two-point contact initiation  
4. Automatic shutdown  

Combines ultrasonic ranging (HC-SR04) and 9-DOF IMU (BNO055) with dual PID controllers for millimeter-precision docking.

---

## 🛠️ Hardware Components  
- **Brain**: Arduino-compatible MCU  
- **Senses**:  
  - HC-SR04 Ultrasonic Pair (20-400cm range)  
  - BNO055 IMU (±0.5° orientation accuracy)  
- **Movement**:  
  - 2x DC Motors (PWM controlled)  
  - L298N Motor Driver  
- **Power**: 🔋 Charging station with magnetic contacts  

---

## 🔌 Pinout  
| Component               | Pin  | Type        |  
|-------------------------|------|-------------|  
| Right US Echo          | 12   | Input       |  
| Right US Trig          | 3    | Output      |  
| Left US Echo           | 4    | Input       |  
| Left US Trig           | 9    | Output      |  
| Motor L PWM            | 5    | PWM         |  
| Motor L Dir            | 8    | Digital     |  
| Motor R PWM            | 6    | PWM         |  
| Motor R Dir            | 11   | Digital     |  
| Dock Interrupt         | 2    | Digital*    |  
| Contact Confirm        | 7    | Digital**   |  

*Pull-up enabled  
**Normally-open contact switch  

---

## 🧠 Aligning Algorithm  
### 1. 🕵️‍♂️ Wall Acquisition Phase  
- Continuous rotation until IMU reports 3° < x-axis < 20°  
- Differential steering (L: forward, R: backward @ 60 PWM)  

### 2. 📐 Precision Alignment  
- Dual PID controllers (one per side)  
- Maintains 20cm standoff from wall  
- Real-time ultrasonic feedback (4-50cm operational range)  

### 3. 🤝 Contact Sequence  
- Triggered by IR/physical dock signal (Pin 2 LOW)  
- Creep mode @ 50 PWM until Pin 7 confirms contact  

### 4. 🛑 System Shutdown  
- Motor kill command  
- Safety lockout until manual reset  

---

## 🎛️ Control Parameters  
```cpp
// PID Constants
const double kp = 24.0;  // Proportional (dominant term)
const double ki = 0.0;   // Integral (disabled)  
const double kd = 0.0;   // Derivative (disabled)

// Operational Limits
const int SAFE_RANGE_CM = 50;  // Max valid distance
const int ALIGN_SPEED = 50;     // Final approach PWM
const int SCAN_SPEED = 60;     // Search mode PWM
