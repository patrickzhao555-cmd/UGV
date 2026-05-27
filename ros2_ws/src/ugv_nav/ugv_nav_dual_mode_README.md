# Jetson Chassis Controller

`ugv_nav_dual_mode.py` is a safe high-level chassis test node. It defaults to
`idle`, which publishes STOP only.

When a test mode is explicitly selected, it still uses one command contract:

```json
{"command_type":"velocity","v_mps":0.20,"omega_radps":0.0}
```

## Modes

- `idle`: STOP only.
- `straight_test`: drive forward while holding the start heading.
- `pivot_test`: pivot to a bounded relative angle.

The Teensy remains the only motor velocity PID layer. Jetson heading correction
is done by changing `omega_radps`; navigation must not publish raw PWM and must
not run motor PID.
