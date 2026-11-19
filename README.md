# LAGER

Live Attitude and Gimbal Error Resolver: a Python package that drives a gimbal (Gremsy T7) and reads a DJI drone's telemetry (Matrice 600 or Matrice 400), to keep a payload pointed at a fixed geographic target, the Point of Interest (POI), while the drone moves.

lager is used as a library by [PORTER](https://github.com/POLOCALC/porter), the POLOCALC flight software, which creates the `PointingController` from the `pointing_controller` block of its configuration. Before November 2026 lager was also a standalone program; that version is kept in the `old_standalone_version` branch.

---

## Table of Contents

- [Overview](#overview)
- [Repository structure](#repository-structure)
- [Hardware](#hardware)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [POI tracking and image roll](#poi-tracking-and-image-roll)
- [Data files and plots](#data-files-and-plots)
- [Modules](#modules)
- [TODO](#todo)

---

## Overview

`PointingController` (in `lager/PointingController.py`) is built from a configuration dictionary or YAML file. It:

1. creates the gimbal object (Gremsy T7, MAVLink over serial),
2. creates the drone object, if configured:
   - **DJI Matrice 600**: DJI OSDK over serial,
   - **DJI Matrice 400**: Payload SDK through the E-Port, using the `psdk_telemetry` program from the `Payload-SDK` submodule,
3. creates the `POI`, if configured,
4. connects to the devices and logs their telemetry to binary files in the data folder (`connect()`, `start_telemetry()`),
5. on request, tracks the POI: about 10 times per second it computes the line of sight from the current position to the POI with `pymap3d` and sends the gimbal angles (`poi.start_tracking(...)`).

The drone link is **passive**: lager only receives its telemetry, it never commands the drone.

---

## Repository structure

```
lager/                              # repository root
├── pyproject.toml                  # package "lager" and its pinned dependencies
├── lager/                          # the Python package
│   ├── PointingController.py       # builds gimbal, drone and POI from the configuration
│   ├── POI.py                      # POI class: tracking loop, line of sight -> gimbal angles (with image roll)
│   ├── DataLoader.py               # decodes the telemetry files of a data folder into pandas DataFrames
│   ├── GPIO.py                     # optional status LED
│   ├── parameters.py               # data paths and logging defaults when used outside porter, LED settings
│   ├── Gimbals/
│   │   └── Gremsy_T7/              # Gremsy T7 driver (MAVLink, telemetry, goto)
│   └── UAVs/
│       ├── DJI_M600/               # DJI Matrice 600 driver (DJI OSDK over serial)
│       └── DJI_M400/               # DJI Matrice 400 driver (psdk_telemetry stream, from a socket or a child process)
├── decoder/
│   ├── decode_telemetry.py         # decodes a data folder and saves it as a pickle
│   └── plot_telemetry.py           # one PNG per UAV and gimbal telemetry quantity
├── test_roll_pointing.py           # bench test of the image-roll pointing on the gimbal alone
└── modules/
    └── Payload-SDK/                # git submodule: POLOCALC's fork of DJI's Payload SDK (psdk_telemetry)
```

---

## Hardware

| Component | Model | Interface |
|-----------|-------|-----------|
| Drone | DJI Matrice 600 | Serial (DJI OSDK) |
| Drone | DJI Matrice 400 | E-Port, USB device mode (`/dev/ttyGS0`), through the Payload SDK |
| Gimbal | Gremsy T7 | Serial (MAVLink) |
| Companion computer | Raspberry Pi 5 | — |
| Status LED (optional) | — | GPIO pin 17 (BCM) |

### Serial wiring (Matrice 600 and Gremsy T7)

Both serial connections use 3.3 V logic (TX / RX / GND, no hardware flow control). Example wiring:

| Device | RPi UART | `/dev` node | RPi pin (BCM) | Signal |
|--------|----------|-------------|---------------|--------|
| DJI M600 | UART0 (default) | `/dev/serial0` | 14 | TXD → drone RX |
| DJI M600 | UART0 (default) | `/dev/serial0` | 15 | RXD ← drone TX |
| Gremsy T7 | UART4 | `/dev/serial1` | 8 | TXD4 → gimbal RX |
| Gremsy T7 | UART4 | `/dev/serial1` | 9 | RXD4 ← gimbal TX |

Enable UART4 by adding `dtoverlay=uart4` to `/boot/firmware/config.txt` and rebooting. The default UART0 must have the Linux console detached (set `enable_uart=1` and remove `console=serial0` from `/boot/firmware/cmdline.txt`). On the POLOCALC payload the ports are set in porter's configuration (e.g. the gimbal on `/dev/ttyAMA4`).

### Matrice 400

The Matrice 400 telemetry comes from `psdk_telemetry`, in the `Payload-SDK` submodule. The Pi's USB-C port must be in device mode and the DJI credentials must be set before building it: see [its README](modules/Payload-SDK/README.md).

---

## Installation

```bash
git clone --recurse-submodules https://github.com/POLOCALC/lager.git
cd lager
pip install -e .
```

`pip install -e .` installs the `lager` package and its dependencies from `pyproject.toml` (`pymavlink`, `pyserial`, `pymap3d`, `pandas`, `numpy`, `matplotlib`, `mpl_ascii`, `rich`, `pyyaml`, `RPi.GPIO`). In an existing clone, fetch the Payload SDK with `git submodule update --init --recursive`.

On the POLOCALC payload, porter's `install_porter.sh` installs lager in porter's virtual environment (`pip install --no-deps -e modules/lager`, with the dependency versions fixed by porter's `requirements.lock`) and builds `psdk_telemetry`.

---

## Configuration

The configuration has up to three blocks: `gimbal`, `uav` and `poi`. In porter they sit under `pointing_controller`. Leave out `uav` or `poi` if not used.

```yaml
gimbal:
  name: Gremsy T7
  simulator: False
  connection:
    type: serial
    protocol: mavlink
    port: /dev/ttyAMA4
    baudrate: 115200
    timeout: 3
  telemetry:
    heartbeat_frequency: 1      # Hz
    frequency: 50               # Hz

# DJI Matrice 600
uav:
  name: DJI M600
  simulator: False
  connection:
    type: serial
    protocol: DJI OSDK
    port: /dev/serial0
    baudrate: 115200
    timeout: 3
  telemetry:
    frequency: 50               # Hz

# or DJI Matrice 400
uav:
  name: DJI M400
  simulator: False
  connection:
    type: usb
    protocol: DJI PSDK
    port: /dev/ttyGS0
    socket: /tmp/psdk_telemetry.sock   # read the stream from psdk-bridge.service; remove it to run psdk_telemetry from lager
    timeout: 60                 # s to wait for the drone link
  telemetry:                    # with "socket" set, psdk-bridge.service sets these rates
    imu_frequency: 0            # IMU navigation group, 50 Hz only; 0 = off
    frequency: 10               # fast group: attitude, IMU, fused position, obstacles, RC
    status_frequency: 5         # status group: GPS, RTK, flight state, anomalies, ESCs
    slow_frequency: 1           # slow group: battery, GPS date/time, home point
    stats_s: 5                  # s between link statistics

poi:
  name: Target
  latitude: 45.5136599
  longitude: 9.2113794
  altitude: 170.0               # metres above mean sea level
  max_distance: 1000.0          # metres; tracking pauses when farther than this
  roll_deg: 0.0                 # optional image roll about the line of sight, default 0 (horizon level)
```

- `name` selects the driver: `Gremsy T7`, `DJI M600` or `DJI M400`.
- `simulator: True` generates synthetic telemetry instead of opening the device.
- `altitude` must use the same reference as the position source: height above mean sea level.

---

## Usage

From Python, the same way porter uses it:

```python
from lager.PointingController import PointingController

pc = PointingController("config.yaml", data_folder="/path/to/run", verbose=True)
pc.connect()
pc.start_telemetry()

pc.poi.start_tracking(pc.gimbal, uav=pc.uav)              # position from the drone telemetry
# or: pc.poi.start_tracking(pc.gimbal, gnss_source=gnss)  # position from another GNSS (a dict with latitude, longitude, altitude)
...
pc.poi.stop_tracking()
pc.stop_telemetry()
pc.disconnect()
```

`configuration` is a YAML path or an already-loaded dictionary. Without `data_folder`, lager creates `telemetry_<UTC time>/` under `parameters.DATA_PATH` (`/home/polocalc/pointing_control_data`), updates the `current` symlink there and logs to `controller.log`. With `data_folder` (porter's case), it writes into that folder and leaves the logging setup to the caller.

**Bench test of the image roll** (gimbal only, no drone):

```bash
python3 test_roll_pointing.py --port /dev/serial1 --yaw 0 --pitch 30 --angle 20
```

It points the gimbal at a fixed yaw/pitch, then sweeps the image roll 0 → +angle → −angle → 0 while keeping the same line of sight (`--step` and `--dt` set the sweep speed).

---

## POI tracking and image roll

The tracking loop (`POI.py`) runs at about 10 Hz:

1. reads the current position, from the drone telemetry or from the given GNSS source,
2. computes the distance to the POI; beyond `max_distance` tracking pauses and is checked again every 5 s,
3. computes the elevation of the POI with `pymap3d.geodetic2aer`,
4. converts the line of sight and the requested image roll into motor angles with `POI.los_to_gimbal_angles(az, el, roll)`, and sends them with `gimbal.goto(...)`.

**Image roll.** The Gremsy T7 motors are stacked yaw → roll → pitch (from the drone to the payload). To rotate the image by `roll_deg` about the line of sight without moving the line of sight, the rotation is split between the roll and yaw motors and the pitch is corrected:

```
roll  = asin( cos(el) · sin(r) )
pitch = atan2( sin(el), cos(el) · cos(r) )
yaw   = az − atan2( sin(el) · sin(r), cos(r) )
```

with `r` = `roll_deg`. Motor convention: +yaw turns right (clockwise seen from above), +roll lowers the right side. Keep `roll_deg` within ±90°.

**Azimuth.** The gimbal yaw follows the drone heading (`forward_heading=True`): the POI azimuth is not used yet, so the drone must face the POI. Only the elevation and the image roll are tracked.

---

## Data files and plots

Each run writes, in the data folder:

| File | Content |
|---|---|
| `uav_telemetry.bin` | raw drone telemetry, record by record (M600 or M400) |
| `psdk_telemetry.log` | `psdk_telemetry`'s log, only when lager runs it itself (M400 without `socket`) |
| `gimbal_telemetry.bin` | raw gimbal telemetry |
| `controller.log` | lager's log, only when used without `data_folder` |

Decode and plot a data folder (default: `/home/polocalc/pointing_control_data/current`):

```bash
python3 decoder/decode_telemetry.py [--data-folder <folder>] [--force-decode] [--overwrite]
python3 decoder/plot_telemetry.py   [--data-folder <folder>] [--dpi 120]
```

- `decode_telemetry.py` prints the UAV and gimbal DataFrames and saves them as `pointing_control_data.pkl`.
- `plot_telemetry.py` saves one PNG per quantity in `<folder>/pointing_controller_plots/`, without a display. Plots whose fields aren't in the data are skipped, so the same script serves the M600 and the M400.

porter's `decoder/plot_sensors_data.py` runs `plot_telemetry.py` automatically on each run folder.

---

## Modules

### `PointingController`
Builds the devices and the POI from the configuration (`get_gimbal()`, `get_uav()`, `get_POI()`), and exposes `connect()`, `disconnect()`, `start_telemetry()`, `stop_telemetry()`. A missing or unknown `uav` or `poi` block is logged and that part is skipped (`pc.uav` / `pc.poi` is `None`).

### `POI`
A fixed geographic target. Key methods:
- `start_tracking(gimbal, uav=None, gnss_source=None, forward_heading=True)`: starts the tracking thread, from the drone telemetry or from a GNSS source,
- `stop_tracking()`: stops it (wakes the loop immediately),
- `distance(lat, lon, alt)`: 3-D distance in metres from a position to the POI, through ECEF coordinates,
- `los_to_gimbal_angles(az, el, roll)`: line of sight and image roll → gimbal motor angles,
- `get_data()`: name, coordinates, `max_distance`, `roll_deg`, current distance and tracking state, for porter's telemetry.

### `UAVs/DJI_M600`
DJI Matrice 600 driver using the DJI OSDK protocol over serial: connection, telemetry subscription and background logging.

### `UAVs/DJI_M400`
DJI Matrice 400 driver. It gets the `psdk_telemetry` record stream in one of two ways:
- with `socket` set: from `/tmp/psdk_telemetry.sock`, served by porter's `psdk-bridge.service`, which owns the PSDK connection from boot and also runs the DJI Pilot 2 widgets,
- otherwise: it runs `psdk_telemetry` itself as a child process and reads its stdout.

It parses the records into its telemetry state (position, attitude, GPS/RTK, battery, flight state, ESCs, anomalies) and writes the raw stream to `uav_telemetry.bin`.

### `modules/Payload-SDK`
Git submodule with POLOCALC's fork of DJI's Payload SDK. It holds `psdk_telemetry`, the C program that gets the Matrice 400 telemetry through the E-Port, and the USB gadget setup for the Raspberry Pi. See [its README](modules/Payload-SDK/README.md) for configuration and build.

### `Gimbals/Gremsy_T7`
Gremsy T7 driver using MAVLink over serial:
- heartbeat loop and MAVLink session management,
- `goto(yaw, pitch, roll, wait)` for absolute angle commands,
- telemetry subscription and background logging.

### `GPIO`
Optional status LED on a Raspberry Pi GPIO pin (BCM 17 by default), disabled by default (`ENABLE_LED_INDICATOR = False` in `lager/parameters.py`). <font color="red">Don't forget a 220 Ohm resistor to avoid frying the diode and GPIO.</font>

| LED state | Meaning |
|-----------|----------|
| Off | Controller not running / disconnected |
| Solid on | Initialising or connected, telemetry idle |
| Blinking | Telemetry acquisition in progress |

The `LED` class wraps `RPi.GPIO` and handles the blink loop in a daemon thread. `RPi.GPIO` is imported unconditionally (and `PointingController` imports `GPIO.py`), so it must be installed even with the LED disabled; on a development machine, `pip install RPi.GPIO` or a stub module is needed.

---

## TODO
- POI tracking
    - use the POI azimuth (`forward_heading=False`) instead of relying on the drone heading
    - add a dynamic correction for the drone attitude (yaw, pitch, roll)
- DJI M600
    - understand heading: from magnetometer? from RTK?
