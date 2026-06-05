# Aggregated status of all of Vizzy's independent battery sources, published by
# the battery monitor. All sources charge through a single connector, so
# `charging` is global; `all_full` gates undocking (leave only when the last
# source finishes charging).

# system_status values.
uint8 OK=0
uint8 WARN=1
uint8 CRITICAL=2

std_msgs/Header header

BatterySource kokam
BatterySource segway_ui
BatterySource segway_powerbase

uint8    system_status    # OK / WARN / CRITICAL (worst across sources)
bool     any_low          # any source is_low  -> should dock
bool     all_full         # all sources is_full -> safe to undock
bool     charging         # docked & actively charging (global, from Kokam)
string[] alerts           # human-readable issues, e.g. "segway_ui low (9.0 V)"
