# 🏎️ TEKNOFEST Autonomous Vehicle Competiton — EV Powertrain & Actuator Controller (2023-2024)

This repository contains the microcontroller firmware developed for our **TEKNOFEST Autonomous Vehicle (NovaDrive) Student Team**. The core focus of this project was to handle dynamic motor velocity profiles, mechanical braking, and low-level safety states on an ARM Cortex-M based STM32 platform.

---

## Project Status & Current Scope

> **Note on Completion Status:** 
> This is an ongoing/milestone project from the competition. Due to timeline constraints, the In-Vehicle Communication (CAN-Bus) was currently incomplete/missing from this codebase. However, all core localized vehicle actuators and safety-critical state transitions have been tested, and optimized recently (2026).

### What Actually Works:
* **Analog DC Motor Propulsion:** Synthesizes analog control voltage via **DAC** to drive the main powertrain H-Bridge smoothly.
* **Servo Braking:** Operates high-precision **PWM** signals to dynamically sweep and deploy mechanical brake surfaces.
* **Quadrature Encoder Feedback:** Utilizes timer Input Capture interrupts to decode wheel encoder ticks for real-time RPM calculation.
* **Asynchronous State Machine:** I updated The entire system runs on a non-blocking delta-time architecture (`HAL_GetTick()`), allowing the CPU to execute speed ramps while remaining responsive to safety events.

---

## Architecture & Safety Features

### 1. Finite State Machine (FSM)
The system transitions dynamically between 4 states based on real-time driver inputs or emergency triggers:
* `STATE_FORWARD`: Default state. Activates forward H-Bridge pins and outputs cruising speed via DAC.
* `STATE_REVERSE`: Activates reverse configurations with a non-blocking software acceleration ramp.
* `STATE_BRAKE`: Cuts off DC motor drive excitation instantly and sweeps the mechanical brake servo gradually.
* `STATE_EMERGENCY`: Zero-latency safety trip. Overrides all running tasks, drops DAC speed to zero, and forces steering/brake servos back to their neutral safe positions.

---

## ⚙️ Hardware Peripherals Used

* **DAC1:** Direct analog speed control generation for the motor controller.
* **TIM2 & TIM3 (PWM Mode):** Drives the braking and steering servo actuators.
* **TIM4 (Encoder Mode):** Hardware-level pulse accumulation for velocity tracking.
* **ADC1:** Monitored system rail voltage parameters.
