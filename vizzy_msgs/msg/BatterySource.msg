# Generic per-source battery information.
#
# Fields a given source does not provide are left at their default and flagged
# via the has_* booleans (e.g. the Segway UI battery reports voltage but no
# percentage). Consumers should check the has_* flags before trusting
# voltage / current / percentage.

# Charge-level classification (mirrors vizzy_msgs/BatteryState).
uint8 CHARGED=0
uint8 GOOD=1
uint8 MEDIUM=2
uint8 LOW_BATTERY=3
uint8 LEVEL_UNKNOWN=4

# Charging classification (mirrors vizzy_msgs/BatteryChargingState).
uint8 NOT_CHARGING=0
uint8 CHARGING=1
uint8 CHARGING_UNKNOWN=2

string  name              # "kokam" | "segway_ui" | "segway_powerbase"

float32 voltage           # V,     valid if has_voltage
float32 current           # A,     valid if has_current (unsigned magnitude on Kokam)
float32 percentage        # 0-100, valid if has_percentage
float32 charge_slope      # V/s,   valid if has_voltage (0.0 otherwise)

uint8   level             # one of CHARGED..LEVEL_UNKNOWN
uint8   charging_state    # one of NOT_CHARGING..CHARGING_UNKNOWN

bool    is_full           # at/above this source's full threshold
bool    is_low            # at/below this source's low threshold

bool    has_voltage
bool    has_current
bool    has_percentage
