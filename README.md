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

## Sensor ranges
The MPU6050 is set to **±4 g** and **±500 °/s** at startup (the chip's defaults are ±2 g and ±250 °/s). On the defaults, impacts clipped at 2.000 g and strong flips already reached ~247 °/s, so every hard flip would have read the same. All thresholds are in g and °/s, so they mean the same as before.

## Serial messages (Arduino → PC)
USB serial, **115200 baud**, one message per line. The Pro Micro only sends while the PC has **DTR** switched on (in C#/Unity: `SerialPort.DtrEnable = true`); without it nothing arrives.

| Line | Meaning |
|---|---|
| `*** STATE: STATE_CALIBRATING ***` | Learning "at rest" for 2 s (at power-up, or after `CAL`). Keep the table still. |
| `STATUS:SCENARIO_SELECT (…)` | Idle, table at rest. Now sent on **every** return to idle (after a twitch, a dropped lift, a hold or a round). |
| `*** STATE: STATE_SENSING ***` | The table started moving. Followed by a `t_ms,ax,az,gx,gy,gz` header and one CSV row per 20 ms sample while sensing/flipping. |
| `STATUS:HOLD_START 820` | The table is tilted into the hold angle **and still**; the number is the ms left until the hold completes. Sent again (with the full 1000) whenever the timer restarts. Not sent while the table only swings through the hold angle. |
| `STATUS:HOLD_CANCEL` | The tilt was lost after a `HOLD_START`, before the hold completed. |
| `*** STATE: STATE_HOLD ***` | Hold completed (selection). |
| `*** STATE: STATE_TWITCH ***` | Quick lift-and-drop (back at rest within 400 ms). |
| `*** STATE: STATE_FLIPPING ***` then `STARTED ROUND` | Still moving after 400 ms and not holding: a flip. |
| `*** STATE: STATE_COOLDOWN ***` then `ENDED ROUND` | After a flip, the table has been still for 400 ms within 0.10 g of its rest angle. That spot becomes the new rest, since a flipped table often lands a few degrees off and would otherwise never count as "home". |
| `PONG RAGE_QUIT` | Reply to `PING`. |

## Commands (PC → Arduino)
Send one per line (ending in a newline), not case-sensitive:

| Command | What it does |
|---|---|
| `PING` | Replies `PONG RAGE_QUIT`. Lets the game find the right serial port by trying each one. |
| `CAL` | Recalibrates "at rest" (2 s, keep the table still) without unplugging the Arduino. |

## Notes
- This project ONLY detects state changes based on the sensor data; the game works out flip power from the CSV rows.
- Please find the correct serial port to read the data from the Arduino Pro Micro (or send `PING` to each port and use the one that answers).