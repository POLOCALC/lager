import os
from pandas import DataFrame

import lager.UAVs.DJI_M400.parameters as params
import lager.UAVs.DJI_M400.stream as stream
import lager.UAVs.DJI_M400.utils as utils 

# M600 key -> (per-battery field, how the two batteries are combined)
BATTERY_SUMMARY = {"bat_percentage":   ("percentage", "min"),
                   "bat_voltage_mv":   ("voltage_mv", "mean"),
                   "bat_current_ma":   ("current_ma", "sum"),
                   "bat_capacity_mah": ("remaining_mah", "sum")}


class DataLoader:
    """Offline decoding of uav_telemetry.bin (the raw psdk_telemetry stream)."""
    def __init__(self, data_folder):
        self.data_folder = data_folder

    def load_records(self, filename=params.TELEMETRY_FILENAME):
        path = os.path.join(self.data_folder, filename)
        try:
            with open(path, "rb") as f:
                data = f.read()
        except FileNotFoundError:
            print(f"File not found: {path}")
            return None
        parser = stream.StreamParser()
        records = parser.feed(data)
        print(f"UAV telemetry loaded from {path}: {len(records)} records, {parser.bad_crc} bad CRC")
        return records

    def parse_records(self, records, fill=True):
        """One row per DATA record (timestamp + that topic's columns); fill=True forward-fills the others."""
        topics, rows, statuses = {}, [], []
        for rtype, topic, host_ns, ac_ms, ac_us, payload in records:
            if rtype == params.RECORD_TOPIC_TABLE:
                topics = stream.parse_topic_table(payload)      # a new table at every restart of the program
            elif rtype == params.RECORD_STATUS:
                state, text = stream.parse_status(payload)
                statuses.append((host_ns / 1e9, params.STATE_NAMES.get(state, state), text))
            elif rtype == params.RECORD_DATA and topic in topics:
                values = utils.decode(topics[topic]["name"], payload)
                if values:
                    values.pop("q", None)                        # list column: roll/pitch/yaw are enough
                    rows.append({"timestamp": host_ns / 1e9, "t_aircraft_ms": ac_ms,
                                 "topic": topics[topic]["name"], **values})
        for t, state, text in statuses:
            print(f"  {t:.3f} {state}: {text}")

        df = DataFrame(rows)
        if df.empty:
            return df
        df = df.sort_values("timestamp", kind="stable").reset_index(drop=True)
        if fill:
            df = df.ffill()
            # whole-aircraft battery keys, as the M600 (the live Drone computes them with utils.battery_summary)
            for col, (field, func) in BATTERY_SUMMARY.items():
                src = [c for c in (f"bat1_{field}", f"bat2_{field}") if c in df]
                if src:
                    df[col] = getattr(df[src], func)(axis=1)
        return df

    def get_data(self, fill=True):
        records = self.load_records()
        if records is None:
            return None
        return self.parse_records(records, fill=fill)
