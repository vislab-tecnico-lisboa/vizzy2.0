# System recovery/health state, broadcast by the recovery_supervisor node.
# Behavior nodes (patrol, etc.) subscribe and must stop commanding the base
# whenever state != OK.

uint8 OK=0          # nominal operation
uint8 RECOVERING=1  # an automatic recovery is in progress (e.g. relocalizing); the
                    # supervisor is driving the base,  consumers must stay idle
uint8 FAULT=2       # unrecoverable; robot halted, awaiting human intervention
                    # (cleared via the /clear_fault service)

std_msgs/Header header
uint8  state
string reason       # human-readable explanation, e.g. "localization diverged"
