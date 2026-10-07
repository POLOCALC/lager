"""
plot_telemetry.py - save one PNG per UAV and gimbal telemetry entry.

Usage:
  python3 plot_telemetry.py [--data-folder <folder>] [--dpi 120]

Plots are written to <data-folder>/pointing_controller_plots/. Nothing is displayed.
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")                     # files only, no display needed
from matplotlib import pyplot as plt
import pandas as pd

from lager.DataLoader import DataLoader
import lager.parameters as params

parser = argparse.ArgumentParser(description="Save UAV and gimbal telemetry plots as PNG files")
parser.add_argument("--data-folder",
                    default=os.path.join(params.DATA_PATH, params.CURRENT_DATA_FOLDER),
                    help="Folder containing the telemetry data")
parser.add_argument("--dpi", type=int, default=120, help="Resolution of the saved PNG files")


# ---------------------------------------------------------------------------
# Plot definitions
#   file:     PNG file name (without extension)
#   title:    figure title
#   keyword:  for the gimbal, the MAVLink message the fields come from (None for the UAV)
#   fields:   one subplot per entry: (column, label, unit, scale, step)
#             scale multiplies the raw value (e.g. 1e-3 for mV -> V)
#             step=True draws status/flag values as steps instead of lines
# ---------------------------------------------------------------------------
UAV_PLOTS = [
    dict(file="uav_accelerometer", title="UAV accelerometer", fields=[
        ("ax", "Acc X", "g", 1, False),
        ("ay", "Acc Y", "g", 1, False),
        ("az", "Acc Z", "g", 1, False)]),
    dict(file="uav_gyroscope", title="UAV gyroscope", fields=[
        ("gx", "Gyro X", "rad/s", 1, False),
        ("gy", "Gyro Y", "rad/s", 1, False),
        ("gz", "Gyro Z", "rad/s", 1, False)]),
    dict(file="uav_magnetometer", title="UAV magnetometer", fields=[
        ("mx", "Mag X", "counts", 1, False),
        ("my", "Mag Y", "counts", 1, False),
        ("mz", "Mag Z", "counts", 1, False)]),
    dict(file="uav_attitude", title="UAV attitude", fields=[
        ("roll",  "Roll",  "deg", 1, False),
        ("pitch", "Pitch", "deg", 1, False),
        ("yaw",   "Yaw",   "deg", 1, False)]),
    dict(file="uav_velocity", title="UAV velocity", fields=[
        ("vx", "Vel X", "m/s", 1, False),
        ("vy", "Vel Y", "m/s", 1, False),
        ("vz", "Vel Z", "m/s", 1, False)]),
    dict(file="uav_position_fused", title="UAV fused position", fields=[
        ("lat_deg", "Latitude",  "deg", 1, False),
        ("lon_deg", "Longitude", "deg", 1, False),
        ("alt_m",   "Altitude",  "m",   1, False)]),
    dict(file="uav_height", title="UAV height above ground", fields=[
        ("hgt_m", "Height", "m", 1, False)]),
    dict(file="uav_gps_position", title="UAV GPS position", fields=[
        ("gps_lat_deg", "Latitude",  "deg", 1, False),
        ("gps_lon_deg", "Longitude", "deg", 1, False),
        ("gps_hfsl_m",  "Height MSL", "m",  1, False)]),
    dict(file="uav_gps_velocity", title="UAV GPS velocity (NED)", fields=[
        ("gps_vn_cms", "North", "cm/s", 1, False),
        ("gps_ve_cms", "East",  "cm/s", 1, False),
        ("gps_vd_cms", "Down",  "cm/s", 1, False)]),
    dict(file="uav_gps_quality", title="UAV GPS quality", fields=[
        ("gps_nsv",    "Satellites", "",  1, True),
        ("hdop",       "HDOP",       "",  1, False),
        ("pdop",       "PDOP",       "",  1, False),
        ("gps_health", "GPS health", "",  1, True)]),
    dict(file="uav_rtk_position", title="UAV RTK position", fields=[
        ("rtk_lat_deg", "Latitude",   "deg", 1, False),
        ("rtk_lon_deg", "Longitude",  "deg", 1, False),
        ("rtk_hfsl_m",  "Height MSL", "m",   1, False)]),
    dict(file="uav_rtk_heading", title="UAV RTK heading", fields=[
        ("rtk_yaw_deg", "Heading", "deg", 1, False)]),
    dict(file="uav_rtk_health", title="UAV RTK health", fields=[
        ("rtk_pos_health", "Position health", "", 1, True),
        ("rtk_yaw_health", "Heading health",  "", 1, True)]),
    dict(file="uav_relative_position", title="UAV obstacle distances", fields=[
        ("rp_down",  "Down",  "m", 1, False),
        ("rp_front", "Front", "m", 1, False),
        ("rp_right", "Right", "m", 1, False),
        ("rp_back",  "Back",  "m", 1, False),
        ("rp_left",  "Left",  "m", 1, False),
        ("rp_up",    "Up",    "m", 1, False)]),
    dict(file="uav_rc_sticks", title="RC sticks", fields=[
        ("rc_roll",  "Roll",     "a.u.", 1, False),
        ("rc_pitch", "Pitch",    "a.u.", 1, False),
        ("rc_yaw",   "Yaw",      "a.u.", 1, False),
        ("rc_thr",   "Throttle", "a.u.", 1, False)]),
    dict(file="uav_rc_switches", title="RC switches", fields=[
        ("rc_mode", "Mode switch", "a.u.", 1, True),
        ("rc_gear", "Gear switch", "a.u.", 1, True)]),
    dict(file="uav_gimbal_port", title="Gimbal angles reported by the UAV", fields=[
        ("gim_roll",  "Roll",  "deg", 1, False),
        ("gim_pitch", "Pitch", "deg", 1, False),
        ("gim_yaw",   "Yaw",   "deg", 1, False),
        ("gim_flags", "Flags", "",    1, True)]),
    dict(file="uav_flight_status", title="UAV flight status", fields=[
        ("flight_status", "Flight status", "", 1, True),
        ("display_mode",  "Display mode",  "", 1, True),
        ("landing_gear",  "Landing gear",  "", 1, True),
        ("flight_error",  "Flight error",  "", 1, True)]),
    dict(file="uav_battery", title="UAV battery", fields=[
        ("bat_percentage",   "Charge",   "%",   1,    False),
        ("bat_voltage_mv",   "Voltage",  "V",   1e-3, False),
        ("bat_current_ma",   "Current",  "A",   1e-3, False),
        ("bat_capacity_mah", "Capacity", "mAh", 1,    False)]),
    dict(file="uav_sdk_status", title="UAV SDK status", fields=[
        ("sdk_control_mode", "Control mode", "", 1, True),
        ("sdk_device_bits",  "Device bits",  "", 1, True)]),
]

GIMBAL_PLOTS = [
    dict(file="gimbal_attitude", title="Gimbal attitude", keyword="MOUNT_ORIENTATION", fields=[
        ("roll",         "Roll",         "deg", 1, False),
        ("pitch",        "Pitch",        "deg", 1, False),
        ("yaw",          "Yaw",          "deg", 1, False),
        ("yaw_absolute", "Yaw absolute", "deg", 1, False)]),
    dict(file="gimbal_angular_velocity", title="Gimbal angular velocity",
         keyword="GIMBAL_DEVICE_ATTITUDE_STATUS", fields=[
        ("angular_velocity_x", "X", "rad/s", 1, False),
        ("angular_velocity_y", "Y", "rad/s", 1, False),
        ("angular_velocity_z", "Z", "rad/s", 1, False)]),
    dict(file="gimbal_device_flags", title="Gimbal device flags",
         keyword="GIMBAL_DEVICE_ATTITUDE_STATUS", fields=[
        ("flags",         "Flags",         "", 1, True),
        ("failure_flags", "Failure flags", "", 1, True)]),
    dict(file="gimbal_imu_accelerometer", title="Gimbal IMU accelerometer", keyword="RAW_IMU", fields=[
        ("xacc", "Acc X", "raw", 1, False),
        ("yacc", "Acc Y", "raw", 1, False),
        ("zacc", "Acc Z", "raw", 1, False)]),
    dict(file="gimbal_imu_gyroscope", title="Gimbal IMU gyroscope", keyword="RAW_IMU", fields=[
        ("xgyro", "Gyro X", "raw", 1, False),
        ("ygyro", "Gyro Y", "raw", 1, False),
        ("zgyro", "Gyro Z", "raw", 1, False)]),
    dict(file="gimbal_imu_magnetometer", title="Gimbal IMU magnetometer", keyword="RAW_IMU", fields=[
        ("xmag", "Mag X", "raw", 1, False),
        ("ymag", "Mag Y", "raw", 1, False),
        ("zmag", "Mag Z", "raw", 1, False)]),
    dict(file="gimbal_imu_temperature", title="Gimbal IMU temperature", keyword="RAW_IMU", fields=[
        ("temperature", "Temperature", "°C", 1e-2, False)]),          # MAVLink: centi-degrees
    dict(file="gimbal_mount_status", title="Gimbal mount status", keyword="MOUNT_STATUS", fields=[
        ("pointing_a", "Pitch", "deg", 1e-2, False),                  # MAVLink: centi-degrees
        ("pointing_b", "Roll",  "deg", 1e-2, False),
        ("pointing_c", "Yaw",   "deg", 1e-2, False),
        ("mount_mode", "Mode",  "",    1,    True)]),
    dict(file="gimbal_power", title="Gimbal power", keyword="SYS_STATUS", fields=[
        ("voltage_battery",   "Supply voltage", "V", 1e-3, False),    # MAVLink: mV
        ("current_battery",   "Supply current", "A", 1e-2, False),    # MAVLink: cA
        ("battery_remaining", "Remaining",      "%", 1,    False)]),
    dict(file="gimbal_system_load", title="Gimbal system load and errors", keyword="SYS_STATUS", fields=[
        ("load",           "CPU load",      "%", 0.1,  False),        # MAVLink: per mille
        ("drop_rate_comm", "Comm drops",    "%", 0.01, False),        # MAVLink: centi-percent
        ("errors_comm",    "Comm errors",   "",  1,    True)]),
    dict(file="gimbal_heartbeat", title="Gimbal heartbeat", keyword="HEARTBEAT", fields=[
        ("system_status", "System status", "", 1, True),
        ("base_mode",     "Base mode",     "", 1, True),
        ("custom_mode",   "Custom mode",   "", 1, True)]),
]


def save_plot(df, spec, t0, plot_folder, dpi):
    """Draw one figure with one subplot per field and save it as PNG. Return the file path or None."""
    if df is None or df.empty or "timestamp" not in df.columns:
        return None
    if spec.get("keyword") is not None:
        if "keyword" not in df.columns:
            return None
        df = df[df["keyword"] == spec["keyword"]]

    # keep only the fields that exist and contain at least one numeric value
    fields = []
    for col, label, unit, scale, step in spec["fields"]:
        if col in df.columns:
            values = pd.to_numeric(df[col], errors="coerce")
            if values.notna().any():
                fields.append((col, label, unit, scale, step, values))
    if not fields:
        print(f"  skipped {spec['file']}: no data")
        return None

    fig, axs = plt.subplots(len(fields), 1, figsize=(12, 2.6 * len(fields) + 0.8),
                            sharex=True, squeeze=False)
    for ax, (col, label, unit, scale, step, values) in zip(axs[:, 0], fields):
        valid = values.notna()
        t = df.loc[valid, "timestamp"] - t0
        y = values[valid] * scale
        ax.plot(t, y, linewidth=0.8, drawstyle="steps-post" if step else "default")
        ax.set_ylabel(f"{label} [{unit}]" if unit else label)
        ax.grid(True, alpha=0.3)
    axs[-1, 0].set_xlabel("Time [s]")
    fig.suptitle(spec["title"])
    fig.tight_layout()

    path = os.path.join(plot_folder, spec["file"] + ".png")
    fig.savefig(path, dpi=dpi)
    plt.close(fig)                                          # free memory: many figures in one run
    return path


def main():
    args = parser.parse_args()
    data_folder = args.data_folder

    data = DataLoader(data_folder).get_data()
    uav_data = data.get("uav") if data else None
    gimbal_data = data.get("gimbal") if data else None

    plot_folder = os.path.join(data_folder, "pointing_controller_plots")
    os.makedirs(plot_folder, exist_ok=True)

    # common time origin, so UAV and gimbal plots share the same time axis
    starts = [df["timestamp"].min() for df in (uav_data, gimbal_data)
              if df is not None and not df.empty and "timestamp" in df.columns]
    if not starts:
        print("No UAV or gimbal telemetry found, nothing to plot.")
        return
    t0 = min(starts)

    saved = []
    for df, specs, name in ((uav_data, UAV_PLOTS, "UAV"), (gimbal_data, GIMBAL_PLOTS, "gimbal")):
        if df is None or df.empty:
            print(f"No {name} telemetry found.")
            continue
        print(f"Plotting {name} telemetry...")
        for spec in specs:
            path = save_plot(df, spec, t0, plot_folder, args.dpi)
            if path:
                saved.append(path)

    print(f"Saved {len(saved)} plots to {plot_folder}")


if __name__ == "__main__":
    main()
