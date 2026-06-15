# LAGER

Live Attitude and Gimbal Error Resolver: a Python-based controller for UAV payload systems, designed to manage and coordinate a drone (DJI M600) and a gimbal (Gremsy T7) for autonomous Point-of-Interest (POI) tracking. The system reads a YAML configuration file, establishes serial connections to the hardware, logs telemetry data, and continuously points the gimbal at a fixed geographic target while the UAV moves.

---

## Table of Contents

- [Overview](#overview)
- [Repository Structure](#repository-structure)
- [Hardware](#hardware)
- [Requirements](#requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Autostart](#autostart)
- [Modules](#modules)
- [Tests](#tests)
- [TODO](#todo)

---

## Overview

The controller runs on a companion computer onboard the UAV (tested on Raspberry Pi). At startup it:

1. Loads a YAML configuration file.
2. Instantiates and connects to the drone (DJI M600 via DJI OSDK over serial) and the gimbal (Gremsy T7 via MAVLink over serial).
3. Starts telemetry logging for both devices to timestamped CSV files.
4. Optionally starts a live terminal display (powered by `rich`) showing real-time telemetry panels and the last log lines.
5. Continuously reads the UAV GPS position and computes the azimuth/elevation from the UAV to the POI using `pymap3d`, then commands the gimbal pitch accordingly.

---

## Repository Structure

```
lager/
├── autostart.sh            # Entry-point called by telemetry.service at boot
├── cli.py                  # Rich-based live terminal display
├── Controller.py           # Main orchestration class
├── DataLoader.py           # Generic data loading utilities
├── decode_telemetry.py     # Offline telemetry decoding and plotting
├── plot_telemetry.py       # Telemetry plotting utilities
├── follow_POI.py           # End-to-end POI tracking entry point
├── GPIO.py                 # GPIO pin control for status LED
├── parameters.py           # Global parameters and paths
├── POI.py                  # POI class and gimbal tracking logic
├── requirements.txt        # Python dependencies
├── config/
│   ├── follow_POI_config.yaml          # Config for POI tracking flight
│   ├── move_gimbal_config.yaml         # Config for gimbal movement tests
│   ├── show_telemetry_config.yaml      # Config for live telemetry display
│   └── show_telemetry_sim_config.yaml  # Config for simulated telemetry display
├── Drones/
│   └── DJI_M600/           # DJI M600 driver (connection, telemetry, utils)
├── Gimbals/
│   └── Gremsy_T7/          # Gremsy T7 driver (MAVLink, telemetry, goto)
└── tests/
    ├── data/
    │   └── gimbal_motion/   # Logged data from gimbal movement tests
    ├── move_gimbal.py       # Gimbal movement test
    ├── show_telemetry.py    # Live telemetry display test
    └── show_telemetry_sim.py# Simulated telemetry display test
```

---

## Hardware

| Component | Model | Interface |
|-----------|-------|-----------|
| Drone | DJI M600 | Serial (DJI OSDK), `/dev/serial0` |
| Gimbal | Gremsy T7 | Serial (MAVLink), `/dev/serial1` |
| Companion computer | Raspberry Pi (or equivalent) | — |
| Status LED | — | GPIO pin 17 (BCM) |

### Wiring

Both serial connections use 3.3 V logic (TX / RX / GND — no hardware flow control).

| Device | RPi UART | `/dev` node | RPi pin (BCM) | Signal |
|--------|----------|-------------|---------------|--------|
| DJI M600 | UART0 (default) | `/dev/serial0` | 14 | TXD → drone RX |
| DJI M600 | UART0 (default) | `/dev/serial0` | 15 | RXD ← drone TX |
| Gremsy T7 | UART4 | `/dev/serial1` | 8 | TXD4 → gimbal RX |
| Gremsy T7 | UART4 | `/dev/serial1` | 9 | RXD4 ← gimbal TX |

Enable UART4 by adding `dtoverlay=uart4` to `/boot/firmware/config.txt` and rebooting. The default UART0 must have the Linux console detached (set `enable_uart=1` and remove `console=serial0` from `/boot/firmware/cmdline.txt`).

---

## Requirements

Python 3.8 or later is recommended. Install dependencies with:

```bash
pip install -r requirements.txt
```

Key dependencies:

| Package | Purpose |
|---------|---------|
| `pymavlink` | MAVLink communication with Gremsy T7 |
| `pyserial` | Serial communication |
| `pymap3d` | Geodetic ↔ AER coordinate conversion for POI pointing |
| `rich` | Live terminal display |
| `pandas` / `numpy` | Telemetry data handling |
| `matplotlib` | Telemetry plotting |
| `pyyaml` | Configuration file parsing |
| `RPi.GPIO` | GPIO pins handling |
| `mpl_ascii` | ASCII telemetry plotting |

---

## Installation

```bash
git clone https://github.com/POLOCALC/lager.git
cd lager
pip install -r requirements.txt
```

---

## Configuration

All behaviour is controlled through a YAML configuration file. A complete example for POI tracking:

```yaml
Controller:
  name: Lab Test Controller
  display:
    enable: True
    refresh_rate: 2  # Hz

Drone:
  name: DJI M600
  simulator: False
  connection:
    type: serial
    protocol: DJI OSDK
    port: /dev/serial0
    baudrate: 115200
    timeout: 3
  telemetry:
    frequency: 50  # Hz

Gimbal:
  name: Gremsy T7
  simulator: False
  connection:
    type: serial
    protocol: mavlink
    port: /dev/serial1
    baudrate: 115200
    timeout: 3
  telemetry:
    heartbeat_frequency: 1  # Hz
    frequency: 50  # Hz

POI:
  name: Target
  latitude: 37.7749
  longitude: -122.4194
  altitude: 30.0       # meters above sea level
  max_distance: 700.0  # meters; tracking is paused if UAV is farther than this
```

Set `simulator: True` for either device to run without physical hardware (synthetic telemetry data will be generated).

---

## Usage

Run the main scripts from the project root:

```bash
# POI tracking (requires drone + gimbal connected)
python3 follow_POI.py

# Decode and inspect previously logged telemetry files
python3 decode_telemetry.py

# Plot previously decoded telemetry files
python3 plot_telemetry.py
```

Run test scripts from the `tests/` directory:

```bash
cd tests/

# Live telemetry display only
python3 show_telemetry.py

# Simulated telemetry display (no hardware needed)
python3 show_telemetry_sim.py

# Move gimbal to specific angles
python3 move_gimbal.py
```

Press **Ctrl+C** to stop any running script gracefully; telemetry logging will be stopped and devices disconnected cleanly.

Each run creates a timestamped folder under `parameters.DATA_PATH` containing the log file and a copy of the configuration used. A `current` symlink is automatically updated to point to the latest run folder.

---

## Autostart

`autostart.sh` is the entry-point executed automatically at boot by the `telemetry.service` systemd unit. It activates the project's virtual environment and launches `follow_POI.py` with the flight configuration. If it crashes, it restarts automatically after 5 seconds.

```bash
#!/bin/bash
set -e

source /home/polocalc/lager_venv/bin/activate
cd /home/polocalc/lager

while true; do
    exec python3 follow_POI.py --config config/follow_POI_config.yaml
    EXIT_CODE=$?
    sleep 5
done
```

To install, enable, and manage the service:

```bash
# Enable the service to start at every boot
sudo systemctl enable telemetry.service

# Start / stop manually
sudo systemctl start telemetry.service
sudo systemctl stop telemetry.service

# Check status and live logs
sudo systemctl status telemetry.service
journalctl -u telemetry.service -f
```

---

## Modules

### `Controller`
The central orchestrator. Loaded with a YAML config file, it:
- Instantiates the drone and gimbal objects via `get_drone()` / `get_gimbal()`.
- Instantiates the `POI` object via `get_POI()`.
- Exposes `connect()`, `disconnect()`, `start_telemetry()`, `stop_telemetry()`.
- Optionally starts the `cli.live_display()` thread.

### `POI`
Represents a fixed geographic point of interest. Key methods:
- `start_tracking(uav, gimbal)` — starts a background thread that polls the UAV position at 10 Hz, computes the gimbal pitch via `pymap3d.geodetic2aer`, and sends a `gimbal.goto()` command.
- `stop_tracking()` — stops the tracking thread.
- `distance(uav_lat, uav_lon, uav_alt)` — returns the 3-D Euclidean distance (metres) between the UAV and the POI via ECEF coordinates.

Pitch computation:
```python
az, el, rng = pymap3d.geodetic2aer(poi_lat, poi_lon, poi_alt,
                                    uav_lat, uav_lon, uav_alt, deg=True)
gimbal_pitch = el
```

### `Drones/DJI_M600`
Driver for the DJI M600 using the DJI OSDK protocol over serial. Provides:
- Serial connection management.
- Telemetry subscription and background logging.
- A `rich` panel renderer for the live display.

### `Gimbals/Gremsy_T7`
Driver for the Gremsy T7 using MAVLink over serial. Provides:
- Heartbeat loop and MAVLink session management.
- `goto(yaw, pitch, roll, wait)` for absolute angle commanding.
- Telemetry subscription and background logging.
- A `rich` panel renderer for the live display.

### `cli`
Builds and refreshes a `rich` `Live` layout with three panes:
- **Drone panel** — real-time drone telemetry.
- **Gimbal panel** — real-time gimbal telemetry.
- **Footer** — last N lines of the log file, colour-coded by log level.

### `GPIO`
Controls a status LED on a Raspberry Pi GPIO pin (BCM pin 17 by default). <font color="red">Don't forget a 220 Ohm resistor to avoid frying the diode and GPIO.</font> The LED reflects the controller lifecycle:

| LED state | Meaning |
|-----------|----------|
| Off | Controller not running / disconnected |
| Solid on | Initialising or connected, telemetry idle |
| Blinking | Telemetry acquisition in progress |

The `LED` class wraps `RPi.GPIO` and handles the blink loop in a daemon thread. If `RPi.GPIO` is not available (e.g. on a development machine) it fails silently and the rest of the code is unaffected. The LED pin and enable flag are configured via `parameters.py` (`LED_INDICATOR_PIN`, `ENABLE_LED_INDICATOR`).

---

## Tests

| Script | Location | Description |
|--------|----------|-------------|
| `follow_POI.py` | root | Full end-to-end POI tracking run |
| `decode_telemetry.py` | root | Load and print a saved telemetry binary file |
| `show_telemetry.py` | `tests/` | Live display with real hardware |
| `show_telemetry_sim.py` | `tests/` | Live display with simulated data |
| `move_gimbal.py` | `tests/` | Command the gimbal to a sequence of angles |

---

## TODO
- POI pointing track (HIGH PRIORITY)
    - add a dynamic correction related to the uav attitude (yaw, pitch, roll)

- DJI M600
    - understand heading: from magnetometer? from rtk?
    - simulate data writing on file
    - separate graphic function from main file

- Gremsy T7
    - separate graphic function from main file