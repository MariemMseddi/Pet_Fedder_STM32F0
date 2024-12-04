# Automated Pet Feeder - STM32F0 Based
This project presents an innovative solution for pet care through an automated pet feeder. Designed using embedded systems and mechanical components, this project aims to provide a reliable and user-friendly feeding mechanism that ensures pets are fed on a scheduled basis, even when owners are away. The system integrates the STM32F0 microcontroller, an MH-Sensor-Series IR sensor, a push button, and an L298N motor driver for accurate and automated feeding.

## Keywords
- MATLAB
- STM32 Programmer
- Pet Feeder
- Embedded Systems

## Project Overview
The automated pet feeder utilizes the STM32F0 microcontroller to handle real-time tasks such as controlling the stepper motor and managing sensor data. With a combination of hardware and software, the system dispenses food to pets at a predefined schedule or when triggered by the pet. The major components involved are:
- **STM32F0 Microcontroller**: Manages inputs and controls outputs such as the stepper motor and sensor data.
- **MH-Sensor-Series IR Sensor**: Detects food shortages by emitting infrared light and measuring the reflection.
- **Push Button**: Detects the presence of the pet and activates the system to dispense food.
- **L298N Motor Driver**: Controls the stepper motor to ensure precise food portions are dispensed.
- **17HS4401 NEMA 17 Bipolar Stepper Motor**: Provides accurate and controlled movements to dispense the food.

## Installation

### Requirements:
- **STM32F0** microcontroller board
- **MH-Sensor-Series IR Sensor**
- **Push Button**
- **L298N Motor Driver**
- **17HS4401 Stepper Motor**
- **MATLAB** (for system simulation and analysis)


## Usage
1. **Feeding Schedule**:
   - Once the system is powered on, the automated feeder will dispense food based on the pet's activity.
   - The IR sensor will monitor the food level, ensuring that it triggers food dispensing when the level is too low.
   
2. **Manual Feeding**:
   - Press the button to activate the feeding cycle manually.

