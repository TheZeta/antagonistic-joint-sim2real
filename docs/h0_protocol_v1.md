# H0 Host-Device Protocol v1

## Safety principle

The protocol does not expose a direct driver-enable command.

Motor-driver enable is controlled only by the device-side safety supervisor.

The host may request:

- ARM
- MOVE
- DISABLE
- CLEAR_FAULT

A fault always implies driver disable.

## Motor coordinate convention

Raw motor encoders decrease during WIND.

The protocol does not expose this sign convention to command logic.

Normalized winding counts are defined as:

W = C_baseline - C_raw

Therefore:

- W > 0 means WIND
- W < 0 means UNWIND

MOVE targets are absolute normalized winding positions relative to the motor encoder baselines captured during successful ARM.

They are not incremental motion requests.
