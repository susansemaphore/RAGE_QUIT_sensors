### RAGE_QUIT Sensors Project

## Hardware

- Arduino Pro Micro
- Accelerometer and gyroscope sensor module (e.g., MPU6050)
- Jumper wires (for now)
- USB cable to connect the Arduino Pro Micro to the computer for both power and serial communication

Arduino Pro Micro (5V)              GY-87 10DOF Module
 ┌──────────────────────┐            ┌────────────────────┐
 │                  VCC ├───────────►│ VCC_IN             │
 │                  GND ├───────────►│ GND                │
 │              2 (SDA) ├───────────►│ SDA                │
 │              3 (SCL) ├───────────►│ SCL                │
 └──────────────────────┘            │ 3.3V (Do Not Use)  │
                                     └────────────────────┘

## Software Overview
This project involves using accelerometer and gyroscope sensors to detect and respond to various physical interactions with a table, such as holding, twitching, and flipping motions. The system calibrates itself on startup, continuously monitors sensor data, and transitions between different states based on the detected movements.

The main states include:
- **STATE_CALIBRATING**: The system collects initial sensor data to establish baseline readings.
- **STATE_SCENARIO_SELECT**: The idle state where the system waits for user interaction.
- **STATE_SENSING**: Active monitoring of table movements to detect potential twitches or flips.
- **STATE_TWITCH**: A brief, small movement detected within a defined time window.
- **STATE_HOLD**: The table is held at an elevated angle for a specified duration.
- **STATE_FLIPPING**: A significant movement indicating a table flip.
- **STATE_COOLDOWN**: A temporary state after a flip to prevent immediate re-triggering.
- **STATE_HOLD**: The table is held at an elevated angle for a specified duration, used to confirm intentional holds before transitioning to other states.
- **STATE_SCENARIO_SELECT**: The idle state where the system waits for user interaction, allowing the user to scroll through scenarios or make selections.
- **STATE_SENSING**: The system is actively monitoring the table for movements that could indicate a twitch or flip, transitioning to the appropriate state based on detected motion.   

## Notes
- This project ONLY detects state changes based on the sensor data.
- Please find the correct serial port to read the data from the Arduino Pro Micro