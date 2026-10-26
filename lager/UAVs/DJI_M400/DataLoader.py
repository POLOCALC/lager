import os
from pandas import DataFrame

import lager.UAVs.DJI_M400.parameters as params
import lager.UAVs.DJI_M400.stream as stream
import lager.UAVs.DJI_M400.utils as utils 

# M600 battery key -> field of the Matrice 400's single battery (bat1_*); battery2, in older recordings, repeats it
BATTERY_SUMMARY = {"bat_percentage":   "bat1_percentage",
                   "bat_voltage_mv":   "bat1_voltage_mv",
                   "bat_current_ma":   "bat1_current_ma",
                   "bat_capacity_mah": "bat1_remaining_mah"}


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
            for col, src in BATTERY_SUMMARY.items():
                if src in df:
                    df[col] = df[src]
        return df

    def get_data(self, fill=True):
        records = self.load_records()
        if records is None:
            return None
        return self.parse_records(records, fill=fill)
