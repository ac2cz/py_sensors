"""
sensors_state.py - load runtime state and static config, matching the
C load_sensors_state() / load_config() semantics.

State file: key=value pairs, re-read every cycle. iors_control rewrites it
(atomically) in response to ground commands. Missing keys keep their
defaults. Unknown keys are warned about but ignored.

Config file: static, read once at startup.
"""

# --- State defaults (mirror the C globals in sensors_state_file.c) ---
STATE_DEFAULTS = {
    "period_to_store_wod_in_seconds": 5 * 60,
    "wod_max_file_size_in_kb": 50,
    "sensor_log_level": 1,                 # INFO_LOG
    "period_to_sample_telem_in_seconds": 60,
    "imu_enabled": 0,
    "temp_humidity_enabled": 0,
    "pressure_enabled": 0,
    "color_enabled": 0,
}

# Keys that are integers (all of them, in this build)
def load_state(path, defaults, verbose=False):
    """
    Read key=value pairs from the state file and return a dict.
    Starts from a copy of defaults; only keys present in the file override.
    Matches C load_sensors_state: whole-file read each call, atoi-style parse,
    unknown keys warned but ignored, missing file leaves all defaults.
    """
    state = dict(defaults)
    try:
        with open(path, "r") as f:
            for line in f:
                if "=" not in line:
                    continue                      # ignore lines with no pair
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.strip()
                if key == "":
                    continue
                if key in state:
                    try:
                        state[key] = int(value)   # atoi: ints only
                    except ValueError:
                        if verbose:
                            print(f"Bad int for {key!r}: {value!r}")
                else:
                    # C does error_print but keeps going
                    if verbose:
                        print(f"Unknown key in state file: {key}")
                if verbose:
                    print(f" {key} = {value}")
    except FileNotFoundError:
        if verbose:
            print(f"Could not load state file: {path} (using defaults)")
    return state


CONFIG_DEFAULTS = {
    "period_to_sample_telem_in_seconds": 60,
}

def load_config(path, defaults, verbose=False):
    """
    Read the static config file once at startup. On this Sense HAT build the
    only relevant key is period_to_sample_telem_in_seconds (mic/CW serial
    device keys from the C version don't apply).

    Unlike the C version, a missing or unreadable config file is NOT fatal:
    we fall back to defaults and keep running, so the payload always produces
    telemetry. (The C exit(1) was overly strict for an unattended system.)
    """
    config = dict(defaults)
    try:
        with open(path, "r") as f:
            for line in f:
                if "=" not in line:
                    continue
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.strip()
                if key in config:
                    try:
                        config[key] = int(value)
                    except ValueError:
                        if verbose:
                            print(f"Bad int for {key!r}: {value!r}, keeping default")
                if verbose:
                    print(f" {key} = {value}")
    except OSError as e:
        if verbose:
            print(f"Could not load config file {path}: {e} — using defaults")
    return config
