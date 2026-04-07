# FPGA-Based Maze Solver Robot

## 🚀 Overview
An FPGA-based autonomous maze solver robot designed in Verilog HDL that uses ultrasonic sensor, DHT11 sensor, and soil moisture sensor for real-time sensing, with an FSM-driven control system enabling intelligent navigation, PWM-based servo control for movement, and UART for debugging.

## 🧠 Features
- Maze solving using FSM
- Ultrasonic-based obstacle detection
- Servo motor control (PWM)
- UART debugging
- Soil moisture detection
- DHT11 environmental sensing


## ⚙️ Modules
- Top Module
- Maze Solver FSM
- Ultrasonic Sensor Interface
- Servo Control
- UART Communication
- DHT11 Interface
- Soil Moisture Interface

## 🔄 FSM Design
```
WAIT_FOR_START
↓
RESET_TICKS
↓
FORWARD
↓
RESET_TICKS
↓
DECIDE
│ │ │
│ │ └── SENSOR_READ → U_TURN
│ │
│ └── FORWARD
│
└── TURN_LEFT / TURN_RIGHT
↓
RESET_TICKS
↓
FORWARD
```
## 🧪 Results

🎥 **Maze Solver Robot Demo**

This video demonstrates the robot autonomously navigating the maze using sensor-based decision making and FSM control.

🔗 Watch here:  
https://youtu.be/5W3ZDTSREt4?t=147

## 🛠️ Tools Used
- Verilog HDL
- FPGA Board (mention yours)
- Simulation tool (ModelSim / Vivado)

## 👨‍💻 My Contributions
- Sree Balaji P
- Vignesh M
- Vidula Balamurugan 
