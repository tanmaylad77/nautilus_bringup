# NAUTILUS Hardware Bring-Up Firmware

PlatformIO Arduino firmware for staged validation of the NAUTILUS wind-powered fire-risk sensing PCB.

## Safety Defaults

On every reset the firmware:

- disables `3V3_SENSOR`, `3V3_RADIO`, and `5V`
- drives BQ25186 `/CE` high, disabling charge
- forces all boost/buck PWM outputs low
- keeps converter PWM disabled until an explicit command is received
- caps PWM duty-cycle commands to 30%

Always start bench supplies with low current limits and verify rails with a multimeter before enabling the next subsystem.

## Build And Monitor

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

The BoM lists `IC1` as JLCPCB part `C2913202`, which resolves to `ESP32-S3-WROOM-1-N16R8`.
PlatformIO uses the standard `4d_systems_esp32s3_gen4_r8n16` board target because it matches the
module memory configuration: 16 MB flash and 8 MB OPI PSRAM, with native USB CDC serial enabled.

## Commands

- `help`
- `pins`
- `power sensor on|off`
- `power radio on|off`
- `power 5v on|off`
- `scan`
- `scan all`
- `ina init`
- `ina read`
- `bq regs`
- `bq profile liion`
- `bq current <mA>`
- `bq charge on|off`
- `bq status`
- `pwm boost <duty_percent>`
- `pwm buck <duty_percent>`
- `pwm enable boost|buck`
- `pwm disable boost|buck|all`
- `sensors init`
- `sensors read`
- `log on|off`
- `faults clear`

## Expected I2C Devices

| Device | Function | Address |
| --- | --- | --- |
| INA228 #1 | Boost/input current and voltage monitor | `0x40` |
| INA228 #2 | Buck/output current and voltage monitor | `0x41` |
| SCD30 | CO2, temperature, humidity sensor | `0x61` |
| SPS30 | Particulate matter sensor | `0x69` |
| BQ25186 | Battery charger | `0x6A` |

## Suggested Bring-Up Flow

### Battery Connector Powered From Current-Limited Bench Supply

1. Confirm no smoke or heating manually.
2. Flash the firmware over USB.
3. Run `pins`.
4. Run `scan` and verify all expected I2C devices are found.
5. Run `power sensor on`, `power radio on`, and `power 5v on`.
6. Verify `3V3_SENSOR`, `3V3_RADIO`, and `5V` rails by multimeter.
7. Run `sensors init`, wait a few seconds, then run `sensors read`.

### Motor Input Powered From Isolated 3 V Current-Limited Supply

1. Run `ina init`.
2. Run `pwm boost 5`.
3. Run `pwm enable boost` and verify the boost output by multimeter.
4. Increase duty only within the firmware cap and bench current limit.
5. Run `pwm disable boost`.
6. Run `pwm buck 5`.
7. Run `pwm enable buck` and verify buck output by multimeter.
8. Run `ina read` and compare against supply/multimeter readings.
9. Run `pwm disable all`.

### Battery Attached

1. Run `bq profile liion`.
2. Run `bq regs`.
3. Run `bq current 10`.
4. Run `bq charge on`.
5. Verify charging externally and with `bq status`.
6. Run `bq charge off` before ending the test.

## Implementation Notes

- BQ25186 access is implemented with local register helpers.
- INA228 access is implemented with local register helpers using a 50 mOhm shunt value.
- SCD30 and SPS30 access uses direct Sensirion I2C command framing and CRC checks, avoiding library API differences during bring-up.
- LoRa pins are defined in code, but LoRa radio validation is intentionally out of scope for this firmware.
