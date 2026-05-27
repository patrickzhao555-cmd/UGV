# Navigation Cleanup Placeholder

The previous `ugv_nav_dual_mode.py` runtime was removed on
`cleanup/two-side-pid-runtime`.

The file now exists only as a safe ROS entrypoint that publishes STOP. New Jetson
navigation should be rebuilt with one command contract:

```json
{"command_type":"velocity","v_mps":0.20,"omega_radps":0.0}
```

Navigation must not publish raw PWM and must not run motor PID.
