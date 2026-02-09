# STM32 Ultrasonic Distance Detection with UART Communication

## Project Overview
This project implements an ultrasonic distance measurement system using an STM32 microcontroller.  
Distance data is acquired from an ultrasonic sensor and transmitted via UART serial communication to an external system such as a PC, another microcontroller, or a ROS-based host node.

The project focuses on low-level sensor interfacing and serial communication, making it suitable as a foundation for embedded systems, IoT, or robotics applications.

---

##  Hardware & Tools
- MCU: STM32 (configured using STM32CubeIDE)
- Ultrasonic Sensor: HC-SR04 (or equivalent)
- Timer:
  - TIM21 – Echo pulse duration measurement
  - TIM2 – PWM output (actuator/buzzer)
- Communication: UART (USART2)
- IDE: STM32CubeIDE v1.15.0

---

##  System Workflow
1. STM32 triggers the ultrasonic sensor via the TRIG pin
2. Echo pulse duration is measured using TIM21
3. Distance is calculated based on the echo time
4. Output logic is applied:
   - Distance < 5 cm → send `1`
   - Distance ≥ 5 cm → send `0`
5. Result is transmitted via UART
6. Incoming UART data can control PWM output (e.g., buzzer or actuator)

---

##  Communication Method
- UART communication operates at 9600 baud rate
- Data is transmitted as ASCII values
- UART interrupt is used for receiving incoming data

> ROS integration is intended to be handled on the host side (PC) by subscribing to serial data and publishing it as ROS topics.

---

## Project Structure

- `Core/`
  - `Src/`
    - `main.c` 
  - `Inc/` 
- `Drivers/` 
- `.project`, `.cproject`, `.mxproject` 
- `STM32L031K6TX_FLASH.ld`
- `final1.ioc` 
- `README.md`

This project is provided as-is for educational purposes.
