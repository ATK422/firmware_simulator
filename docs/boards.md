# Board files

A board file (`boards/<dir>/<board>.toml`) says which external chips sit on which bus and pins of the RP2040, and
what the physical world around them starts at. `run --board`, robot files and `sim_test` all read them; the code
is `sim_host::board` (`Board::load`, `Board::apply`, which returns the board's named [inputs](#inputs)).

- [Which board file for which image](#which-board-file-for-which-image)
- [Generated files and overlays](#generated-files-and-overlays)
- [Top level](#top-level)
- [Device tables](#device-tables): [`[can]` and `[[spi]]`](#can-and-spi-the-mcp2517fd), [`[[i2c]]`](#i2c),
  [`[[analog]]`](#analog-adc-inputs-and-muxes), [`[[gpio]]`](#gpio-straps-and-pulls), [`[[flash]]`](#flash),
  [`[[ws2812]]`](#ws2812), [`[[dshot_esc]]`](#dshot_esc), [`[[dynamixel]]`](#dynamixel),
  [`[[hiwonder]]`](#hiwonder), [`[[soft_i2c]]`](#soft_i2c), [`[[drv8243]]`](#drv8243), [`[w5500]`](#w5500),
  [`[transducer]`](#transducer)
- [Inputs](#inputs)

## Which board file for which image

These are the pairings the smoke test (`crates/sim_test/tests/smoke.rs`) boots, with the mock bridge client ID
each image connects as:

| titan_firmware_rs image | Board file | Client |
|---|---|---|
| `template`, `power_board` | `mk2/power_board` | 1 |
| `esc_board` | `mk2/esc_board` (ID strap GPIO 29 low: board 0; high: board 1, client 3) | 2 |
| `camera_cage_bb` | `mk2/camera_cage_bb` | 4 |
| `actuator_board` | `mk2/actuator_board` | 5 |
| `led_board`, `led_board2` | `talos/led_board` | 6 |
| `sbc_board` | `talos/sbc_board` | 7 |
| `ivc_board`, `ivc_board2`, `ivc_dac` | `talos/ivc_board` | 8 |
| `poac_board` | `talos/poac_board` | 1 |
| `smart_battery` | `mk2/sbh_mcu` (serial number 21 in flash) | 21 |
| `hbridge_board` | `lilboat/hbridge_board` | 2 |
| `po_half_board` | `lilboat/po_half` | 1 |
| `puddles_backplane` | `mk2/puddles_backplane` (Ethernet, no CAN) | |

titan_firmware's C apps for Mercury (no titan_firmware_rs image yet) go on `mercury/po_half` (client 1) and
`mercury/esc_board` (ID strap GPIO 29: clients 2 and 3), as `robots/mercury.toml` puts them.

## Generated files and overlays

The files are generated from titan_firmware's board headers (`lib/titan_boards/include/boards/**/*.h`) by

```bash
python3 scripts/gen_boards.py           # rewrites boards/
python3 scripts/gen_boards.py --check   # fails if boards/ is out of date (CI runs this)
```

The generator keeps what the headers say: the MCP2517FD wiring and CAN bus of boards built on the CAN block, the
debug UART, the board I2C bus, the flash size, and every other `*_PIN` define under `[pins]`. Everything else (the
sensors, the PIO devices, the straps, the values the inputs start at) is written by hand in an **overlay** with
the same path under `boards/overlays/`, which the generator appends verbatim after a
`# ---- boards/overlays/...` line. So:

* Never edit a generated file; edit or add `boards/overlays/<dir>/<board>.toml` and rerun the generator.
* A new board header in titan_firmware gets its file on the next run (headers that define `UWRT_BOARD_DEFINED`
  or `MCP2517FD_SPI`).
* The generator reads `external/titan_firmware`; with that submodule not checked out it finds no headers, writes
  nothing and `--check` passes without checking anything.

Unknown keys are ignored (except in the tables marked strict below), so `[debug_uart]`, `[i2c_bus]` and `[pins]`
from the generator are information for whoever writes the overlay, not something the emulator acts on.

## Top level

| Key | Default | Meaning |
|---|---|---|
| `name` | required | Board name, e.g. `"mk2/power_board"` |
| `mcu` | `"rp2040"` | Only `rp2040` is supported |
| `crystal_hz` | 12 MHz | XOSC frequency |
| `flash_size` | 2 MiB | QSPI flash size in bytes (`run --flash-size` overrides it) |
| `[pins]` | | Pin names from the header, for reference |

## Device tables

### `[can]` and `[[spi]]`: the MCP2517FD

```toml
[can]
bus = "internal"        # "internal": CAN FD 1/5 Mbit/s; "external": classic 250 kbit/s
client_id = 1           # for reference; the firmware picks its own

[[spi]]
bus = 0                 # SPI0 or SPI1
cs = 17

[spi.mcp2517fd]         # or [spi.mcp2518fd] (reports a device ID)
int_pin = 22
clkin_pin = 21
termination = true
```

| Key | Default | Meaning |
|---|---|---|
| `[can].bus` | | Which robot bus the board is on, and so its bit rates. A board without `[can]` gets no CAN bus |
| `[[spi]].bus`, `cs` | | SPI block (0 or 1) and chip select GPIO |
| `int_pin` | required | GPIO the controller's INT output drives |
| `clkin_pin` | | GPIO whose GPOUT clock feeds CLKIN (UWRT boards clock the MCP2517FD from the RP2040) |
| `crystal_hz` | | Frequency of a crystal on OSC1 instead |
| `termination` | `true` | Level of the controller's GPIO0 input (the CAN block's termination jumper) |

The model covers the SPI instruction set, RAM, FIFOs, the TEF, interrupt flags, nominal/data bit timing checks and
ISO 11898-1 fault confinement (see [faults.md](faults.md)).

### `[[i2c]]`

```toml
[[i2c]]
bus = 0
device = "mcp3426"
channels = [
    { name = "rail_3v3", volts = 3.3, divider_ohms = [10_000, 10_000] },
    { name = "rail_balanced", volts = 16.0, divider_ohms = [10_000, 1_000] },
]
```

| Key | Default | Meaning |
|---|---|---|
| `bus` | 0 | I2C0 or I2C1 |
| `device` | required | `sht41`, `mcp3426`, `mcp3427`, `mcp3428`, `ms5837_02ba`, `ms5837_30ba`, `bq40z80`, `ssd1306` |
| `name` | the device kind | Prefix of the device's input names |
| `address` | the chip's default | 7-bit address (SHT41 0x44, MCP342x 0x68, SSD1306 0x3C; the MS5837 is always 0x76 and the BQ40Z80 0x0B) |

Device-specific keys, with the inputs each device gets (`<name>` is the `name` key or the device kind):

| Device | Keys (default) | Inputs |
|---|---|---|
| `sht41` | `temperature_c` (25), `humidity_rh` (40) | `<name>.temperature_c`, `<name>.humidity_rh` |
| `mcp3426`, `mcp3427` (2 channels), `mcp3428` (4) | `channels = [{ volts, divider_ohms = [top, bottom], name }]` (unlisted channels read 0 V) | each channel's `name`, or `<name>.ch1` ... `<name>.ch4`: the voltage before the divider |
| `ms5837_02ba`, `ms5837_30ba` | `pressure_pa` (absolute), or `depth_m` below a 101 325 Pa surface in 997 kg/m³ water (0); `temperature_c` (20) | `<name>.pressure_pa` (absolute, Pa), `<name>.temperature_c` |
| `bq40z80` | `serial` (1), `voltage_mv` (16 400), `current_ma` (-2 500, negative is discharging), `soc_percent` (80), `temperature_c` (25), `side_port` (true) | `<name>.voltage_mv`, `<name>.current_ma`, `<name>.soc_percent`, `<name>.temperature_c` |
| `ssd1306` | `page_commands_in_any_mode` (false: page-mode positioning only in page mode, as the datasheet says) | none; `run --screenshot FILE` saves what it shows |

The I2C controller model does clock stretching, NAKs and aborts; an address no device answers is NAKed.

### `[[analog]]`: ADC inputs and muxes

```toml
[[analog]]                  # a voltage behind a divider
adc = 0                     # ADC input 0-3 = GPIO 26-29
name = "port_battery"
volts = 16.2
divider_ohms = [10_000, 1_000]

[[analog]]                  # a CD74HC4051 mux in front of the ADC input
adc = 0
name = "mux"
mux = { select_pins = [23, 28, 29], enable_pin = 22, inputs = [
    { volts = 3.3, name = "h0.nfault_v" },
    { volts = 1.2 },        # input "mux.in1"
] }
```

| Key | Meaning |
|---|---|
| `adc` | ADC input 0-3 (GPIO 26-29) |
| `name` | Input name (default `adc<N>`) |
| `volts`, `divider_ohms` | The voltage being measured and the `[top, bottom]` resistor divider in front of the pin (none: the pin sees `volts`) |
| `mux.select_pins` | GPIOs on the mux's S0-S2 |
| `mux.enable_pin` | GPIO on its active-low enable, if wired |
| `mux.inputs` | Up to 8 `{ volts, divider_ohms, name }`; unnamed ones are `<name>.in0` ... `<name>.in7` |

An entry needs `volts` or `mux`. The ADC samples the input at the exact time each conversion ends; the internal
temperature sensor is input 4. A hydrophone takes an ADC input through a board's `[transducer.receive]` instead.

### `[[gpio]]`: straps and pulls

```toml
[[gpio]]
pin = 29
level = "low"           # driven from outside: "high" or "low"
# pull = "up"           # or an external pull resistor: "up" or "down"
```

A `level` wins over a `pull`; a pin with neither floats. Robot files can replace a board file's entries for the
same pins per board (`gpio = [{ pin = 29, level = "high" }]`), which is how one ESC board file serves both ESC
boards.

### `[[flash]]`

Bytes programmed into flash before boot (serial numbers, per-board data), at XIP addresses:

```toml
[[flash]]
address = 0x101F_F000   # the last sector of a 2 MiB flash
bytes = [21]
```

The image is loaded after, so the bytes must not overlap it.

### `[[ws2812]]`

| Key | Default | Meaning |
|---|---|---|
| `pin` | | Data pin |
| `leds` | | Number of LEDs |
| `name` | `"ws2812"` | Name in the `device` trace |

Frames are decoded from the pin waveform (T0H/T1H thresholds, reset gap) and reported to the `device` trace;
pulses outside the datasheet's windows are counted as timing errors.

### `[[dshot_esc]]`

| Key | Default | Meaning |
|---|---|---|
| `name` | `esc1`, `esc2`, ... | Name in traces and in a vehicle's thruster wiring |
| `signal_pin` | | DShot signal pin |
| `telemetry_pin` | | UART pin for KISS telemetry (10 bytes at 115 200 baud), if wired |
| `max_rpm` | 4000 | Mechanical RPM at full throttle, the speed proportional to the throttle |
| `throttle_fit` | | A measured speed curve instead: `{ forward = [c0, c1, c2], reverse = [c0, c1, c2] }`, the throttle step (DShot value minus 48 ahead, minus 1048 astern) at which the motor settles at `rpm`, `c0 + c1 rpm + c2 rpm²`; below `c0` it stands still. The mk2 and Mercury ESC boards give it the bench fit of their APD 80F3 ESCs and T200 thrusters ([below](#the-esc-boards-speed-curve)) |
| `pole_pairs` | 7 | Motor pole pairs (for eRPM) |
| `signal_loss_ms` | 250 | The motor stops this long after the last valid frame and coasts down (0: never). Assumed from BLHeli_S/Bluejay behaviour, not measured on the UWRT ESCs |

The model decodes DShot150-1200 frames (rate measured from the frame), answers bidirectional DShot (a line idling
high) with its eRPM 30 µs after each frame, sends telemetry when asked, and turns a motor that follows the throttle
with a 60 ms first-order lag (3D mode: 48-1047 forward, 1049-2047 reverse). After a signal loss it follows the
next valid frame (there is no arming sequence). A vehicle reads each ESC's RPM ([vehicles.md](vehicles.md)).

#### The ESC boards' speed curve

The Talos (mk2) and Mercury ESC boards drive APD 80F3 ESCs, patched by titan_firmware for 3D DShot
(`talos/ESCBoard/esc_firmware_patching`: 48-1047 ahead and 1049-2047 astern, both on the ESC's 0-1000 throttle
scale), with Blue Robotics T200 thrusters. Their `throttle_fit` is the quadratic fit of throttle against RPM that
the boards' own feed-forward uses (`ThrusterController.c`, `esc_core::controller::feed_forward`), which
`generateFF.m` made from riptide's `calibrate_thruster.py` bench runs (throttle ±20 to ±725, the RPM the ESC
reported and the force on a gauge; the same runs gave riptide's `force_to_rpm_coefficents`). So the firmware's
feed-forward lands each motor next to its target, ahead and astern, and the controller's P and I terms take out the
rest, as on the bench.

| Throttle step | Ahead | Astern |
|---|---|---|
| 13 / 15 (the fit's offset) | starts turning | starts turning |
| 400 | 2047 RPM | 2072 RPM |
| 725 (the boards' hard limit) | 3068 RPM | 3100 RPM |
| 999 (extrapolated) | 3770 RPM | 3804 RPM |

The fit was measured up to the hard limit. It is indexed by the DShot direction, as the calibration ran in raw
throttle; astern turns slightly faster and (by riptide's curve) pushes harder per RPM, which suggests the bench
thruster had a counter-rotating propeller, so for the others the two directions may be swapped (a 1 % difference in
speed). Not modelled: the ESCs' configured ramp of 25 % per 100 ms (`Talos_Config.cfg`; how the APD firmware
applies it to DShot is not known), the patch's slow-down before a direction change (the lag passes through zero
instead), and the current drawn (`current_per_krpm_a`, linear, in the telemetry).

### `[[dynamixel]]`

| Key | Default | Meaning |
|---|---|---|
| `pin` | | Half-duplex TTL bus pin |
| `id` | | Servo ID |
| `baud` | 57 600 | Bus baud rate |
| `position` | | Where the horn is at power-up (pulses, 4096 per turn) |
| `operating_mode` | | Operating mode register (3 position, 4 extended position) |

An XL430-W250 speaking protocol 2.0 (PING, READ, WRITE, REG_WRITE, ACTION, FACTORY_RESET, REBOOT, SYNC_READ,
SYNC_WRITE); the horn moves at the profile velocity while torque is on.

### `[[hiwonder]]`

`pin` (half-duplex serial pin, 115 200 8N1) and `id`. An LX/HTS bus servo: position 0-1000 over 240°, the move
commands, and the read commands.

### `[[soft_i2c]]`

An I2C/SMBus device on two bit-banged (or PIO) pins: `sda`, `scl`, plus any `[[i2c]]` device keys (`device`,
`name`, ...). The smart battery's BQ40Z80 gauge is one:

```toml
[[soft_i2c]]
sda = 7
scl = 6
device = "bq40z80"
name = "gauge"
serial = 21             # must match the board's serial number in [[flash]]
```

Give the pins pull-ups with `[[gpio]]` entries, as the board does.

### `[[drv8243]]`

DRV8243 H-bridge drivers sharing an nSLEEP line (strict: unknown keys are errors):

| Key | Meaning |
|---|---|
| `nsleep_pin` | The shared nSLEEP GPIO |
| `drvoff_pin` | The DRVOFF GPIO, if wired |
| `bridges` | One `{ nfault_pin = N }` or `{ nfault_input = "name" }` per driver (nFAULT on a GPIO, or on a mux input of an `[[analog]]` entry: 0 V pulled low, 3.3 V released), with `fitted = false` for a driver that is not populated (never answers) |

The model runs the wake-up handshake the firmware uses to find the drivers.

### `[w5500]`

The Puddles backplane's Ethernet controller (strict): `bus` (SPI), `cs`, `int_pin`. Socket 0 in MACRAW mode, the
receive interrupt and the PHY link; see [running.md](running.md#puddles-ethernet).

### `[transducer]`

The board's acoustic transducer and how it is wired, used when the board is placed in an acoustic environment:
see [acoustics.md](acoustics.md#transducers).

## Inputs

`Board::apply` returns the board's physical inputs as named, shared values (`sim_host::board::Inputs`). The CLI
lists them when it loads a board and sets them with `--input NAME=VALUE`; tests change them while the board runs
(`Bench::set`, [testing.md](testing.md#inputs)); a vehicle drives pressure inputs ([vehicles.md](vehicles.md)).
Setting a name the board does not have is an error (and a panic in tests), which catches typos.

The inputs of the shipped boards:

| Board | Inputs |
|---|---|
| `mk2/power_board` | `port_battery`, `stbd_battery` (V), `rail_3v3`, `rail_balanced` (V), `sht41.temperature_c`, `sht41.humidity_rh` |
| `talos/poac_board` | `rail_3v3`, `rail_balanced`, `sht41.temperature_c`, `sht41.humidity_rh` |
| `mk2/camera_cage_bb` | `depth.pressure_pa`, `depth.temperature_c`, `sht41.temperature_c`, `sht41.humidity_rh` |
| `mk2/esc_board` | `esc_vcc` (V; the ESCs count as powered above 9 V) |
| `mk2/sbh_mcu` | `gauge.voltage_mv`, `gauge.current_ma`, `gauge.soc_percent`, `gauge.temperature_c`, `sht41.temperature_c`, `sht41.humidity_rh` |
| `mk2/puddles_backplane` | `depth.pressure_pa`, `depth.temperature_c`, `rail_3v3`, `rail_balanced` |
| `lilboat/po_half` | `water_depth.*`, `reghousing_depth.*` (MS5837), `sht41.*`, `rail_3v3`, `rail_balanced`, `tank_pressure_v`, `regulated_pressure_v` |
| `lilboat/hbridge_board` | `h0.nfault_v`, `h1.nfault_v`, `h3.nfault_v` (driven by the DRV8243 model) |

Each snapshot of a bench or robot gets inputs of its own (`Inputs::deep_copy`), so copies can run side by side
with different values.
