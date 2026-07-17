# NAUTILUS Hardware Bring-Up Firmware

PlatformIO Arduino firmware for staged validation of the NAUTILUS wind-powered fire-risk sensing PCB.

## Safety Defaults

On every reset the firmware:

- disables `3V3_SENSOR`, `3V3_RADIO`, and `5V`
- drives BQ25186 `/CE` high, disabling charge
- forces all boost/buck PWM outputs low
- keeps converter PWM disabled until an explicit command is received
- caps boost PWM duty-cycle commands to 85% and buck PWM duty-cycle commands to 95%

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
- `pwm boost <duty_percent>|enable|disable`
- `pwm buck <duty_percent>|enable|disable`
- `pwm enable boost|buck`
- `pwm disable boost|buck|all`
- `dual start`
- `dual stop`
- `dual status`
- `sensors init`
- `sensors read`
- `log on|off`
- `faults clear`

## Expected I2C Devices

| Device | Function | Address |
| --- | --- | --- |
| INA228 #1 | Boost/input current and voltage monitor | `0x44` |
| INA228 #2 | Buck/output current and voltage monitor | `0x40` |
| SCD30 | CO2, temperature, humidity sensor | `0x61` |
| SPS30 | Particulate matter sensor | `0x69` |
| BQ25186 | Battery charger | `0x6A` |

`0x44` corresponds to the INA228 address-pin state `A1=VS, A0=GND`, matching the
boost/input monitor on the as-built board. `0x40` is `A1=GND, A0=GND`.

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
- Converter PWM is built in asynchronous bring-up mode: boost drives `BOOST_LO_PWM` and holds `BOOST_HI_PWM` low;
  buck drives `BUCK_HI_PWM` and holds `BUCK_LO_PWM` low. Define `ENABLE_SYNC_COMPLEMENTARY_PWM` only after the
  individual switch nodes have been validated. The synchronous path uses active-high UCC27282 gate-driver inputs
  and 1 us MCPWM deadtime.
- `dual start` is a controlled two-stage test mode. It estimates boost duty from `D = 1 - Vin / 8 V`, then clamps
  boost duty to an 85% test limit and ramps from 5%. It trims buck duty slowly toward a 5 V output using the
  buck/output INA228, and disables both stages on INA read failure, input-current limit, or buck output overvoltage.
  The startup input-voltage sanity window is 0.8 V to 6.5 V to allow the boost input node to float high before load.
- LoRa pins are defined in code, but LoRa radio validation is intentionally out of scope for this firmware.
