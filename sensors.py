#!/opt/iors/iors_control_venv/bin/python
"""
sensors.py - Student On Orbit Sensor System (Python port)

Reads the Astro Pi Sense HAT (v2) and writes telemetry to disk:
  - A real-time (RT) file, overwritten each sample via atomic write-then-rename,
    read by iors_control when it needs to broadcast live telemetry.
  - A WOD (Whole Orbit Data) file, appended on a slower period and rolled into
    a timestamped file when it exceeds a size threshold.

Runtime behaviour is controlled by a state file that iors_control rewrites in
response to ground commands. The state file is re-read every cycle so changes
take effect without restarting. Per-sensor enable flags default OFF; sensors
are commissioned on from the ground after flight.

Static per-deployment configuration lives in the config file (read once at
startup). A missing or corrupt config or state file falls back to defaults
rather than aborting.

Replaces the Waveshare C implementation. Telemetry layout matches
sensor_telemetry_t.
"""

import os
import sys
import time
import struct
import argparse
import signal

try:
    from sense_hat import SenseHat
except ImportError:
    SenseHat = None

from sensors_state import (
    load_state, load_config,
    STATE_DEFAULTS, CONFIG_DEFAULTS,
)
from gpiozero import MotionSensor

# Clock sanity: timestamps before this are treated as "clock not set".
CLOCK_2024_01_01 = 1704085200

# Sensor valid-flag values (match the C SENSOR_* enum)
SENSOR_OFF = 0
SENSOR_ON = 1
SENSOR_ERR = 2

# Matches MAX_NUMBER_FILE_IO_ERRORS in the C sensors_config.h
MAX_FILE_IO_ERRORS = 5

# Fallback period (seconds) to re-check the state file even when not sampling,
# matching the C period_to_load_state_file.
STATE_RECHECK_PERIOD = 60

DEFAULT_DATA_FOLDER = "/ariss"
RT_TELEM_NAME = "rt_telemetry.dat"
WOD_SUBDIR    = "pacsat/wod"          # relative to the data dir
WOD_PREFIX    = "sooss_wod_"          # roll appends YYMMDDHHMM
STATE_FILE_NAME = "sensors.state"
CONFIG_FILE_NAME = "sensors.config"

# Packed record format for sensor_telemetry_t (little-endian, no padding).
TELEM_FORMAT = "<I I H H H H H H H H H H H H H B B I H"

TELEM_SIZE = struct.calcsize(TELEM_FORMAT)

VERBOSE = False
pir = MotionSensor(26)
_running = True

def _handle_sigterm(signum, frame):
    global _running
    _running = False

# --------------------------------------------------------------------------
# Value clamping helpers
# --------------------------------------------------------------------------
def clamp_u16(v):
    v = int(round(v))
    return 0 if v < 0 else (0xFFFF if v > 0xFFFF else v)


def clamp_s16_as_u16(v):
    v = int(round(v))
    if v < -32768:
        v = -32768
    if v > 32767:
        v = 32767
    return v & 0xFFFF


def clamp_u8(v):
    v = int(round(v))
    return 0 if v < 0 else (0xFF if v > 0xFF else v)


# --------------------------------------------------------------------------
# Sensor reads (gated on state enable flags)
# --------------------------------------------------------------------------
def read_sensors(sense, state):
    """
    Read enabled Sense HAT sensors into a telemetry dict. Each sensor group is
    only read when its state enable flag is set; otherwise its fields are
    zeroed and its valid flag is SENSOR_OFF. A read that throws sets SENSOR_ERR.
    """
    t = {
        "timestamp": int(time.time()),
        "LPS25_pressure": 0, "LPS25_temp": 0,
        "HTS221_temp": 0, "HTS221_humidity": 0,
        "AccelerationX": 0, "AccelerationY": 0, "AccelerationZ": 0,
        "GyroX": 0, "GyroY": 0, "GyroZ": 0,
        "MagX": 0, "MagY": 0, "MagZ": 0,
        "IMUTemp": 0,
        "light_level": 0, "light_RGB": 0,
        "MotionDetected": 0,
        "PirValid": SENSOR_OFF,
        "ImuValid": SENSOR_OFF,
        "TempHumidityValid": SENSOR_OFF,
        "PressureValid": SENSOR_OFF,
        "ColorValid": SENSOR_OFF,
    }

    # --- Pressure + its temperature (LPS25H) ---
    if state["pressure_enabled"]:
        try:
            pressure_hpa = sense.get_pressure()
            temp_from_p = sense.get_temperature_from_pressure()
            t["LPS25_pressure"] = int(round(pressure_hpa * 4096)) & 0xFFFFFFFF
            t["LPS25_temp"] = clamp_s16_as_u16(temp_from_p * 100)
            t["PressureValid"] = SENSOR_ON
        except Exception as e:
            t["PressureValid"] = SENSOR_ERR
            if VERBOSE:
                print(f"Pressure read failed: {e}")

    # --- Temperature + Humidity (HTS221) ---
    if state["temp_humidity_enabled"]:
        try:
            humidity = sense.get_humidity()
            temp_h = sense.get_temperature_from_humidity()
            # HTS221: library returns calibrated float (per-chip factory compensation
            # already applied). We encode the calibrated value at the sensor's real
            # resolution (0.1 degC, 1 %RH) as an unsigned int. Ground applies a simple
            # linear polynomial to recover physical units — no per-chip cal needed on the
            # ground, because it's already baked into the downlinked value.
            t["HTS221_temp"]     = int(round(temp_h * 10)) & 0xFFFF     # 0.1 degC/count
            t["HTS221_humidity"] = int(round(humidity)) & 0xFFFF        # 1 %RH/count
            t["TempHumidityValid"] = SENSOR_ON
        except Exception as e:
            t["TempHumidityValid"] = SENSOR_ERR
            if VERBOSE:
                print(f"Temp/Humidity read failed: {e}")

    # --- IMU (LSM9DS1): accel (G), gyro (rad/s), mag (uT) ---
    if state["imu_enabled"]:
        try:
            accel = sense.get_accelerometer_raw()
            gyro = sense.get_gyroscope_raw()
            mag = sense.get_compass_raw()
            t["AccelerationX"] = clamp_s16_as_u16(accel["x"] * 1000)
            t["AccelerationY"] = clamp_s16_as_u16(accel["y"] * 1000)
            t["AccelerationZ"] = clamp_s16_as_u16(accel["z"] * 1000)
            t["GyroX"] = clamp_s16_as_u16(gyro["x"] * 1000)
            t["GyroY"] = clamp_s16_as_u16(gyro["y"] * 1000)
            t["GyroZ"] = clamp_s16_as_u16(gyro["z"] * 1000)
            t["MagX"] = clamp_s16_as_u16(mag["x"] * 100)
            t["MagY"] = clamp_s16_as_u16(mag["y"] * 100)
            t["MagZ"] = clamp_s16_as_u16(mag["z"] * 100)
            t["IMUTemp"] = 0  # sense_hat exposes no separate IMU die temp
            t["ImuValid"] = SENSOR_ON
        except Exception as e:
            t["ImuValid"] = SENSOR_ERR
            if VERBOSE:
                print(f"IMU read failed: {e}")

    # --- Colour / light (TCS34725, Sense HAT v2) ---
    if state["color_enabled"]:
        try:
            rgb = sense.colour.colour      # (r, g, b, c), gain/integration set at init
            r, g, b, c = rgb
            t["light_level"] = clamp_u8(c)
            t["light_RGB"] = ((clamp_u8(r) << 16) |
                              (clamp_u8(g) << 8) |
                              clamp_u8(b)) & 0xFFFFFFFF
            t["ColorValid"] = SENSOR_ON
        except Exception as e:
            t["ColorValid"] = SENSOR_ERR
            if VERBOSE:
                print(f"Colour read failed: {e}")

    # --- PIR motion (GPIO 26) ---
    if state["pir_enabled"]:
        try:
            d = 1 if pir.motion_detected else 0
            t["MotionDetected"] = d
            if VERBOSE:
                print(f"PIR: {d}")
            t["PirValid"] = SENSOR_ON
        except Exception as e:
            t["PirValid"] = SENSOR_ERR
            if VERBOSE:
                print(f"PIR read failed: {e}")
    return t


# --------------------------------------------------------------------------
# Packing
# --------------------------------------------------------------------------
def pack_telemetry(t):
    """Pack the telemetry dict into bytes matching sensor_telemetry_t."""
    flags = ((t["MotionDetected"] & 0x1) |
             ((t["PirValid"] & 0x3) << 1) |
             # bits 3-7 pad1
             ((t["ImuValid"] & 0x3) << 8) |
             ((t["TempHumidityValid"] & 0x3) << 10) |
             ((t["PressureValid"] & 0x3) << 12) |
             ((t["ColorValid"] & 0x3) << 14))

    return struct.pack(
        TELEM_FORMAT,
        t["timestamp"] & 0xFFFFFFFF,
        t["LPS25_pressure"] & 0xFFFFFFFF,
        t["LPS25_temp"],
        t["HTS221_temp"],
        t["HTS221_humidity"],
        t["AccelerationX"], t["AccelerationY"], t["AccelerationZ"],
        t["GyroX"], t["GyroY"], t["GyroZ"],
        t["MagX"], t["MagY"], t["MagZ"],
        t["IMUTemp"],
        t["light_level"],
        0,                       # pad1
        t["light_RGB"] & 0xFFFFFFFF,
        flags,
    )


# --------------------------------------------------------------------------
# RT telemetry: atomic overwrite
# --------------------------------------------------------------------------
def save_rt_telem(data, rt_path):
    """Atomic write: tmp then rename, so iors_control never reads a partial file."""
    tmp_path = rt_path + ".tmp"
    try:
        with open(tmp_path, "wb") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        os.rename(tmp_path, rt_path)
        return True
    except OSError as e:
        if VERBOSE:
            print(f"ERROR writing RT telem: {e}")
        return False


# --------------------------------------------------------------------------
# WOD telemetry: append + roll
# --------------------------------------------------------------------------
def wod_tmp_name(wod_path):
    """The active (accumulating) WOD file is the base path plus .tmp"""
    return wod_path + ".tmp"


def wod_append(wod_path, data):
    """
    Append one packed telemetry record to the active WOD tmp file.
    Returns the file size after the write, or -1 on error.
    """
    tmp_path = wod_tmp_name(wod_path)
    try:
        with open(tmp_path, "ab") as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        return os.path.getsize(tmp_path)
    except OSError as e:
        if VERBOSE:
            print(f"ERROR appending WOD: {e}")
        return -1


def wod_roll(wod_path):
    """
    Roll the active WOD file: rename base.tmp to base + YYMMDDHHMM (UTC, minute
    resolution, matching the C log_add_to_directory). A fresh WOD file starts on
    the next append. Returns the rolled path, or None on error.
    """
    tmp_path = wod_tmp_name(wod_path)
    try:
        if not os.path.exists(tmp_path):
            return None
        stamp = time.strftime("%y%m%d%H%M", time.gmtime())
        dest = wod_path + stamp
        os.rename(tmp_path, dest)
        if VERBOSE:
            print(f"Rolled WOD to {dest}")
        return dest
    except OSError as e:
        if VERBOSE:
            print(f"ERROR rolling WOD: {e}")
        return None


# --------------------------------------------------------------------------
# Main loop
# --------------------------------------------------------------------------
def main():
    global VERBOSE

    parser = argparse.ArgumentParser(description="Student On Orbit Sensor System (Python)")
    parser.add_argument("-d", "--dir", default=DEFAULT_DATA_FOLDER,
                        help="data directory (default: %(default)s)")
    parser.add_argument("-c", "--config", default=None,
                        help="config file path (default: <dir>/sensors.config)")
    parser.add_argument("-s", "--state", default=None,
                        help="state file path (default: <dir>/sensors.state)")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="print status messages")
    args = parser.parse_args()

    VERBOSE = args.verbose

    rt_path = os.path.join(args.dir, RT_TELEM_NAME)
    wod_path = os.path.join(args.dir, WOD_SUBDIR, WOD_PREFIX)
    config_path = args.config or os.path.join(args.dir, CONFIG_FILE_NAME)
    state_path = args.state or os.path.join(args.dir, STATE_FILE_NAME)

    # Static config, read once. Currently only a startup default for the
    # sample period; the live value comes from state.
    config = load_config(config_path, CONFIG_DEFAULTS, VERBOSE)

    if SenseHat is None:
        print("ERROR: sense_hat library not available", file=sys.stderr)
        return 1
    try:
        sense = SenseHat()
        # Static colour-sensor setup. Kept here (not per-read) so we configure
        # once. If gain ever becomes flight-changeable, move into read_sensors.
        sense.colour.gain = 64
        sense.colour.integration_cycles = 64
    except Exception as e:
        print(f"ERROR: could not initialise Sense HAT: {e}", file=sys.stderr)
        return 1

    if VERBOSE:
        print(f"Sensors started. RT={rt_path} WOD={wod_path}")
        print(f"State={state_path} Config={config_path}")
        print(f"Record size: {TELEM_SIZE} bytes")


    # Load initial state so we have periods before the first loop pass.
    state = load_state(state_path, STATE_DEFAULTS, VERBOSE)

    now = time.time()
    last_wod = now
    last_sample = 0          # force a sample on the first eligible pass 
    last_state_check = now
    file_io_errors = 0
    last_packed = None       # most recent packed record, used by WOD

    signal.signal(signal.SIGTERM, _handle_sigterm)
    signal.signal(signal.SIGINT, _handle_sigterm)

    while _running:
        now = time.time()
        sample_period = state["period_to_sample_telem_in_seconds"]
        wod_period = state["period_to_store_wod_in_seconds"]
        wod_max_kb = state["wod_max_file_size_in_kb"]

        if sample_period > 0 and now >= CLOCK_2024_01_01:

            # --- WOD: append last sample on the WOD period, roll if too big ---
            if wod_period > 0 and last_packed is not None and \
                    (now - last_wod) >= wod_period:
                last_wod = now
                size = wod_append(wod_path, last_packed)
                if size < 0:
                    file_io_errors += 1
                elif size / 1024 > wod_max_kb:
                    wod_roll(wod_path)

            # --- Sample: read sensors, write RT ---
            if (now - last_sample) >= sample_period:
                last_sample = now
                # Re-read state each sample so ground commands take effect.
                state = load_state(state_path, STATE_DEFAULTS, False)
                last_state_check = now

                t = read_sensors(sense, state)
                last_packed = pack_telemetry(t)
                if not save_rt_telem(last_packed, rt_path):
                    file_io_errors += 1
                elif VERBOSE:
                    print(f"Wrote RT at {t['timestamp']} ({len(last_packed)} bytes)")
        elif sample_period > 0 and VERBOSE:
            print("clock not set, skipping telemetry")

        # Fallback state recheck so a change to the sample period (or to enable
        # sampling at all) is picked up even when the current period is long or
        # sampling is disabled.
        if (now - last_state_check) >= STATE_RECHECK_PERIOD:
            last_state_check = now
            state = load_state(state_path, STATE_DEFAULTS, VERBOSE)

        if file_io_errors > MAX_FILE_IO_ERRORS:
            print("ERROR: exceeded max file IO errors, exiting", file=sys.stderr)
            return 1

        time.sleep(1)
    if VERBOSE:
        print("SIGTERM received, exiting cleanly")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        pass
