# H0 Nucleo Firmware

## Phase 1: dry-run observation firmware

This firmware phase must not energize either IBT-2 motor driver.

Supported live behavior:

- read AS5600 joint angle
- read both motor encoders
- receive Protocol-v1 commands
- ARM and capture motor encoder baselines
- DISABLE
- CLEAR_FAULT
- reject MOVE
- stream Protocol-v1 telemetry

Driver outputs are force disabled at startup and continuously throughout the main loop.

There is intentionally no driver-enable function in the Phase-1 board interface.

Motor motion will only be added after the complete sensing, coordinate, protocol, baseline and safety pipeline has been verified on the commissioned H0 mechanism.
