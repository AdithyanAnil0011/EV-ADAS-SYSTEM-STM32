# EV ADAS System – STM32 Blue Pill

A simulation-based Electric Vehicle (EV) control and Advanced Driver Assistance System (ADAS) developed using the **STM32F103C8T6 (Blue Pill)**. The project combines EV dynamics, battery monitoring, regenerative braking, ultrasonic-based ADAS, fault detection, and a real-time Python dashboard.

The system is developed and tested using **STM32CubeIDE, STM32CubeMX, and PICSimLab**.

---

## Project Overview

The system simulates the major control and monitoring functions of an electric vehicle.

The STM32 handles:

- EV speed and torque control
- Accelerator and brake pedal inputs
- ECO, NORMAL and SPORT driving modes
- Battery State of Charge (SOC) estimation
- Regenerative braking
- Motor temperature monitoring
- Ultrasonic-based obstacle detection
- Forward Collision Warning (FCW)
- Time-to-Collision (TTC) calculation
- Blind Spot Detection (BSD)
- Fault detection and handling

A Python-based dashboard receives the STM32 data through UART and provides a real-time visualization of the vehicle status.

---

## System Architecture

```text
                ┌──────────────────────┐
                │      PICSimLab       │
                │  Blue Pill + Sensors │
                └──────────┬───────────┘
                           │
                           ▼
                ┌──────────────────────┐
                │      STM32F103C8T6    │
                │                      │
                │  EV Control          │
                │  ADAS                │
                │  Fault Management    │
                │  Ultrasonic Sensors  │
                └──────────┬───────────┘
                           │ UART
                           ▼
                ┌──────────────────────┐
                │   Python Dashboard   │
                │                      │
                │ Speed / SOC / Range  │
                │ EV Metrics           │
                │ ADAS Status          │
                │ Fault / Alarm        │
                └──────────────────────┘
```
## Main Features
### 1. EV Control

The EV model calculates vehicle behaviour based on accelerator and brake inputs.

The system calculates:
- Motor torque
- Vehicle speed
- Mechanical power
- Battery SOC
- Estimated driving range
- Motor temperature

The vehicle supports three drive modes:
```
Mode	Torque Scaling
ECO	60%
NORMAL	100%
SPORT	130%
```
The maximum simulated motor torque is defined as: 
```
#define EV_MAX_TORQUE_NM 150.0f
```
### 2. Regenerative Braking

When the brake pedal exceeds the regenerative braking threshold, the system enters regenerative braking operation.
```
#define EV_REGEN_THRESHOLD_PCT 5.0f
#define EV_REGEN_TORQUE_MAX_NM 80.0f
```
The motor torque becomes negative during regeneration, representing braking torque.

The recovered energy is used to increase the simulated battery SOC.

### 3. Battery SOC and Range

The battery model uses a simulated:
```
#define EV_BATTERY_CAPACITY_KWH 60.0f
```
SOC is updated through energy integration based on the calculated power and elapsed time.

The estimated range is calculated using the remaining battery energy and the selected driving-mode efficiency.
```
Example:
Remaining Energy
        │
        ▼
Battery SOC × Battery Capacity
        │
        ▼
Energy Consumption (Wh/km)
        │
        ▼
Estimated Range
```
### 4. ADAS

The ADAS module uses three ultrasonic sensors:

- Front
- Left
- Right

The system calculates obstacle distance and Time-to-Collision (TTC).

ADAS functions include:

- Forward Collision Warning
- Critical collision detection
- Blind Spot Detection
- TTC monitoring

The system generates warnings based on configurable thresholds.

### 5. Fault Management

The fault management module monitors critical vehicle conditions.

Fault conditions include:

- Motor over-temperature
- Critically low battery SOC
- Critical collision condition

The system maintains a fault bitmask and an overall fault state.

Example:

FAULT = 0x00

means no active fault.

When a critical fault occurs, the vehicle can enter the fault state and motor torque is disabled.

## EV State Machine

The EV controller uses different operating states:
```
PARKED
   │
   │ Accelerator pressed
   ▼
READY
   │
   ▼
DRIVING
   │
   │ Brake applied
   ▼
REGEN
   │
   │ Brake released
   └──────────────► DRIVING

Critical fault
      │
      ▼
    FAULT
```
This allows the EV model to respond differently depending on the current operating condition.

## Sensor Inputs

The STM32 ADC is used for simulated vehicle inputs.

- Input	STM32 Pin
- Accelerator	PA0
- Brake	PA1
- SOC	PA2
- Motor Temperature	PA3

Ultrasonic sensors provide:

- FRONT distance
- LEFT distance
- RIGHT distance

## Python Dashboard

The Python dashboard communicates with the STM32 through UART.

It displays:

- Real-time speedometer
- Battery SOC
- Estimated range
- Drive mode
- Motor torque
- Accelerator percentage
- Brake percentage
- Motor temperature
- Alarm status
- Fault status
- UART signal status
- Speed history
- ADAS bird's-eye view
- Obstacle distances
- TTC

The dashboard provides a visual representation of the vehicle's current state while the simulation is running.

## Hardware / Software
### Hardware
- STM32F103C8T6 Blue Pill
- Ultrasonic sensors
- Simulated accelerator input
- Simulated brake input
- LEDs for status indication
- Buzzer
- PICSimLab
### Software
- STM32CubeIDE
- STM32CubeMX
- PICSimLab
- C / Embedded C
- Python
- Matplotlib
- PySerial

## PROJECT STRUCTURE
```
EV-ADAS-SYSTEM-STM32/
│
├── Core/
│   ├── Inc/
│   │   ├── ev_control.h
│   │   ├── adas.h
│   │   ├── fault.h
│   │   ├── ultrasonic.h
│   │   └── common.h
│   │
│   └── Src/
│       ├── main.c
│       ├── ev_control.c
│       ├── adas.c
│       ├── fault.c
│       ├── Ultrasonic.c
│       └── uart_shell.c
│
├── Drivers/
│
├── Debug/
│
├── ev_dash.ioc
│
└── STM32F103C8TX_FLASH.ld
```
## Running the STM32 Simulation
- Open the project in STM32CubeIDE.
- Build the project.
- Program/run the STM32 Blue Pill configuration in PICSimLab.
- Start the UART communication.
- Note the COM port assigned to the simulation.
- Running the Dashboard

Create/activate the Python virtual environment and install the required packages.
```
pip install matplotlib pyserial
```
Then run:
```
python ev_dashboard.py --port COM2
```
Replace COM2 with the COM port used by your system.

The dashboard will then receive the vehicle telemetry from the STM32 through UART.

## Testing

The system can be tested under different operating conditions:

## EV Control
- Accelerator variation
- Brake application
- ECO mode
- NORMAL mode
- SPORT mode
- Different vehicle speeds
- Battery
- Normal SOC operation
- Low SOC condition
- SOC discharge during acceleration
- SOC increase during regenerative braking
- Range estimation
## ADAS
- Safe obstacle distance
- Warning distance
- Critical collision distance
- Different vehicle speeds
- Left/right obstacle detection
- TTC changes
- Fault System
- Motor over-temperature
- Critical SOC
- Critical collision
- Fault indication
- Vehicle response during fault
## Purpose

This project demonstrates how EV control, ADAS, sensor processing, fault management, and real-time monitoring can be integrated into a single embedded system.

It is intended as a simulation and learning platform for EV embedded control and ADAS concepts rather than a production automotive safety system.

## Future Improvements

Possible extensions include:

- CAN communication
- More accurate EV motor and battery models
- Real vehicle sensors
- Improved SOC estimation
- More advanced collision prediction
- Additional ADAS functions
- Hardware deployment on an actual EV platform
- Data logging and analysis


## Author 
Adithyan Anilkumar

