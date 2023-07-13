# Description

Simple X27.168 (_or equivalent_) gauge cluster stepper motor demo. This is ideally to be used with 4 half-bridges, or a driver suited for the bipolar stepper motor.

On startup (_reset_) the stepper motor should sweep from 0 to 600 steps and back to 0. This allows it to "home" as the X27 does not have enough torque to cause damage to the physicals stops in the molded plastic housing. This overdrives the needle beyond its maximum and correctly sets it back to the minimum as a "homing" procedure should it ever get out of sync.

It's important to ensure the motor gets **12V-DC** supply from the driver or half-bridge circuit allowing high-speed movement without missing steps.

## Wiring

| Pin    | Motor Controller / H-Bridge |
| ------ | --------------------------- |
| `PA13` | `A1`                        |
| `PA12` | `B1`                        |
| `PA11` | `A2`                        |
| `PA10` | `B2`                        |