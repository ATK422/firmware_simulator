# Vehicles: thrusters move the robot, sensors see it

A robot file's `[[vehicle]]` puts its boards in a simulated vehicle (`sim_physics`). The firmware's DShot frames
spin the ESC models' motors, the propeller speeds push a rigid body through the water, and the boards' depth
sensors and acoustic transducers follow where it goes. Thruster commands therefore go through the real firmware
end to end, and a depth controller can be closed through the firmware's own depth readings.

> **Every vehicle parameter is assumed.** There is no CAD model, weighing or tank data behind `vehicles/*.toml`
> yet: Talos is a plausible 32 kg open frame, Puddles uses BlueROV2 Heavy numbers from the literature, both with
> T200 thrust curves at 16 V. The thruster layout and the wiring of thrusters to ESCs are assumed too. Replace them
> when there is data; the tests check behaviour derived from whatever the files say.

- [The model](#the-model)
- [Vehicle files](#vehicle-files)
- [In a robot file](#in-a-robot-file)
- [External vehicles](#external-vehicles)
- [Running it and the CSV log](#running-it-and-the-csv-log)
- [Tests](#tests)
- [Limitations](#limitations)

## The model

* A rigid body in six degrees of freedom (Fossen's equations for a diagonal mass matrix): mass and inertia, added
  mass, linear and quadratic drag per axis on the velocity through the water (with an optional uniform current),
  gravity at the centre of gravity and buoyancy at a centre of buoyancy above it, so it rights itself in roll and
  pitch. The Munk moment can be scaled down (`munk_moment_scale`).
* The surface: the hull is a prism of `height_m` around the centre of buoyancy, so buoyancy fades over the last
  half-height and a positively buoyant vehicle floats instead of flying; drag scales with the wetted fraction; a
  thruster out of the water pushes nothing (faded over 5 cm).
* The bottom: in a pool or lake environment the vehicle rests on its bottom (a spring-damper contact with friction).
* Thrusters: position and direction on the hull, thrust `k rpm |rpm|` with separate forward and reverse
  coefficients from one datasheet point each. An `inverted` thruster (counter-rotating propeller) pushes along its
  direction when its motor turns backwards.
* Integrated with RK4 at 1 ms on the robot's virtual clock after every 100 µs slice, so runs are deterministic.

Frames: world north-east-down (`z` is depth), body forward-starboard-down, both from the centre of gravity.

## Vehicle files

`vehicles/talos.toml`, `vehicles/talos_nereus.toml`, `vehicles/puddles.toml` and `vehicles/tempest.toml` (strict:
unknown keys are errors).
Each value is commented in the files with where it came from. `talos_nereus.toml` is Talos as nereus's robot pack and
riptide's `talos.yaml` describe it: the real thrusters (VUS, VUP, HUS, HUP, HLS, HLP, VLS, VLP, in the controller's
order), nereus's pool-identified body (diagonal terms only), and a thrust curve through riptide's
`force_to_rpm_coefficents` at its 24 N limit (24 N at 3171 RPM ahead, 2932 RPM astern), so the thrust for a speed
is the force the controller chose that speed for; `robots/talos_c.toml`, `talos_c_nereus.toml` and
`talos_nereus.toml` use it.
`tempest.toml` is Tempest, the AUV before Talos, from riptide's `tempest.yaml` (mass, inertia, volume, the
controller's damping model, the eight thrusters' poses); its added mass, height and thrust curve are assumed, and
the horizontal offset of its centre of buoyancy is dropped (it would float pitched up about 50 degrees).
`robots/tempest.toml` leaves its thrusters unwired ([robots.md](robots.md#mercury-and-tempest)).

| Key | Meaning |
|---|---|
| `name` | Vehicle name |
| `mass_kg` | Mass |
| `volume_m3` | Water displaced fully under water (buoyancy `rho g V`; Talos is 2 N positive, Puddles 1 N) |
| `inertia_kg_m2` | Moments of inertia about the body axes through the centre of gravity (roll, pitch, yaw) |
| `cob_m` | Centre of buoyancy from the centre of gravity, body frame (negative z is above) |
| `height_m` | Vertical extent of the hull around the centre of buoyancy (for the waterline) |
| `added_mass` | Diagonal added mass and inertia: surge, sway, heave (kg), roll, pitch, yaw (kg m²) |
| `linear_drag` | N/(m/s) per axis, N m/(rad/s) for the rotations |
| `quadratic_drag` | N/(m/s)², N m/(rad/s)² |
| `munk_moment_scale` | Share of the potential-flow Munk moment that acts (1 by default; both shipped files use 0.25, because open frames shed the flow and the full value pitched them over at speed) |
| `thruster_model` | Default thrust curve: `{ forward_n, reverse_n, at_rpm }` (T200 at 16 V, fitted to Blue Robotics' data: 50.8 N / 40.5 N at 3533 RPM) |
| `[[thruster]]` | `name`, `position_m`, `direction` (the way it pushes at positive RPM, normalised), `inverted` (false), `curve` (its own curve) |

## In a robot file

```toml
[[vehicle]]
name = "talos"
file = "../vehicles/talos.toml"     # relative to the robot file
position_m = [5.0, 6.0, 0.0]        # centre of gravity: north, east, depth
# rpy_deg = [0.0, 0.0, 90.0]        # starting roll, pitch, yaw
floating = true                     # start at the depth where it floats, instead of position_m's depth
# water = { density_kg_m3 = 1025.0, current_mps = [0.1, 0.0, 0.0] }
# log_ms = 10                       # keep a state log
thrusters = [
    { thruster = "fwd_stbd", board = "esc0", esc = "esc1" },   # vehicle file thruster, robot board, ESC name
    # ... one per thruster
]

[[vehicle.sensor]]                  # a pressure input on the hull
board = "camera_cage"
input = "depth.pressure_pa"
mount_m = [0.30, 0.0, 0.10]         # body frame: forward, starboard, down

# [[vehicle.transducer]]            # a board's transducer on the hull (needs an [environment])
# board = "ivc"
# mount_m = [-0.2, 0.0, 0.15]
```

| Key | Meaning |
|---|---|
| `file`, `position_m`, `rpy_deg`, `floating` | The vehicle file and its starting pose |
| `water` | `density_kg_m3` (997, fresh water and what titan_firmware's depth driver assumes; sea water is about 1025), `gravity_mps2`, `surface_pa` (101 325), `current_mps` (world frame), `bottom_m` (none; in a tank or lake environment, its depth) |
| `thrusters` | Each vehicle thruster's ESC: the board's name in the robot file and the ESC's `name` in its board file's `[[dshot_esc]]` (`esc1`-`esc4` on the ESC board, `t1`-`t8` on the Puddles backplane) |
| `[[vehicle.sensor]]` | A board input that takes the absolute water pressure at `mount_m` (an MS5837's `<name>.pressure_pa`): `surface_pa + rho g depth` of the mount, the air pressure above the surface |
| `[[vehicle.transducer]]` | A board whose hydrophone and projector follow the hull at `mount_m`; such a board takes no `position_m` or `trajectory` |
| `log_ms` | Keep a state log every this many milliseconds |
| `external` | Moved by another simulator instead of its thrusters ([below](#external-vehicles)) |

After every slice the vehicle is stepped to the robot's time with each thruster's RPM read from its ESC's motor
model, then the sensor inputs and transducer positions are updated. Wiring mistakes (an unknown thruster, board,
ESC or input) are errors when the robot loads.

The shipped wiring (assumed): on Talos, ESC board 0 (`esc0`) drives the four 45° horizontal thrusters and ESC
board 1 (`esc1`) the four vertical ones; thrusters 1 and 4 of each board are `inverted`, matching the ESC boards'
default invert mask, so in RPM mode a positive target pushes every thruster along its direction (raw throttle
turns those four backwards). Positive vertical thrust dives. The camera cage's depth sensor is 30 cm ahead of and
10 cm below the centre of gravity. On Puddles, the backplane's `t1`-`t4` are horizontal and `t5`-`t8` vertical, none
inverted, with the depth sensor on the aft end cap. Both start afloat; the depth sensors zero at boot, as on the
dock.

## External vehicles

With `external = true` (or `robot --external-vehicle NAME`) the vehicle's own physics stop: another simulator owns
its motion, and posts where the vehicle is to the dashboard, `POST /api/vehicle` ([tools.md](tools.md#api)). That is
how [nereus](nereus.md) moves Talos while the emulated ESC boards push it. The vehicle stays where it was last put
(the robot file's pose until the first post); its pressure sensors take the pressure at their mounts at once, and
its transducers follow from the next slice, so IVC boards on it move too. Its thrusters still turn their ESC's
propeller speed into thrust (the curve alone: the other simulator handles the surface), which `GET /api/thrusters`
reports per ESC and per thruster, for the other simulator to apply. The posted pose is that of whatever point the
other simulator reports (nereus: `base_link`), so in such a robot file the sensor mounts are measured from that
point. An external vehicle needs `--dashboard`, since nothing else can move it.

```bash
firmware_simulator robot robots/talos_nereus.toml --dashboard 127.0.0.1:8080 --realtime --time 600s
curl -s -X POST -d '{"position_m": [2.0, 1.0, 1.5], "rpy_deg": [0, 0, 90]}' http://127.0.0.1:8080/api/vehicle
curl -s http://127.0.0.1:8080/api/thrusters
```

In code: `VehicleSim::external`, `VehicleSim::set_pose`.

## Running it and the CSV log

```bash
firmware_simulator robot robots/puddles.toml --eth-mock-bridge backplane --time 10s --vehicle-csv puddles.csv
firmware_simulator robot robots/talos.toml --mock-bridge internal:1 --time 2s --vehicle-csv talos.csv
#   vehicle talos: at [5.00, 6.00, 0.25] m, roll 0.0 pitch 0.0 yaw 0.0 deg, 0.00 m/s
```

The run ends with each vehicle's pose and speed. `--vehicle-csv FILE` writes the state every 10 ms (with several
vehicles, put `NAME` in the file name):

```
time_s,north_m,east_m,depth_m,roll_deg,pitch_deg,yaw_deg,u_mps,v_mps,w_mps,p_dps,q_dps,r_dps,fwd_stbd_n,...,heave_aft_port_n
0.0000,5.00000,6.00000,0.25215,0.000,0.000,0.000,0.00000,0.00000,0.00000,0.000,0.000,0.000,0.000,...,0.000
```

`u, v, w` are body-frame velocities, `p, q, r` body rates, and one column per thruster gives its thrust in
newtons. With only a mock bridge nothing commands the thrusters, so the vehicle floats; send thruster commands
from a test (below) or run the real stack over `--udp`/`--can` (to verify locally). The Puddles backplane applies
thruster commands only 5 s after boot, and the ESC boards hold neutral for their first 5 s.

In code: `Robot::vehicle(name)` returns the `VehicleSim` (its `vehicle.state`, `vehicle.imu()`, `log`), and
`VehicleSim::keep_log` starts a log.

## Tests

| Test | What it shows |
|---|---|
| `crates/sim_physics` unit tests | Each behaviour on its own: neutral buoyancy holds, rising and sinking at the drag-limited speed, surge terminal velocity, yaw from differential thrust, righting in roll and pitch, the surface, the bottom, currents, the Munk moment, determinism |
| `crates/sim_physics/tests/vehicles.rs` | The shipped files imply a plausible AUV (top speed, dive speed, spin rate, roll period); prints them with `--nocapture` |
| `crates/sim_host/tests/vehicle.rs` | Robot-file wiring and its errors, sensors and transducers following the vehicle, snapshots carrying their own vehicle, IVC boards linking from a drifting vehicle |
| `crates/sim_test/tests/vehicle.rs` | Puddles dives on UDP thruster commands while its own depth sensor follows (within 3 cm), the kill switch stops the thrusters, a depth controller in the test closes the loop through the firmware and holds 1 m, runs reproduce exactly |
| `crates/sim_test/tests/vehicle_talos.rs` | Talos's two ESC boards in RPM mode spin each motor to its target, the vehicle dives and turns as predicted, the camera cage reports the depth (about 40 s) |
| `crates/sim_test/tests/vehicle_nereus.rs` | The controller's raw CAN thruster frames spin the Rust ESC boards to their targets ahead and astern (24 N ahead stops at the hard limit's 3068 RPM); the thrust reported for nereus is what riptide's curve chose each speed for; an external vehicle stays where it is posted and the camera cage reports that depth |

## Limitations

Diagonal added mass and drag (no cross-coupling, lift or fins); added mass does not shrink out of the water; a
flat surface (no waves); no walls (a vehicle can leave a tank sideways); no propeller reaction torque; thrust does
not depend on inflow speed; the water density is a vehicle setting, not taken from the acoustic environment.
The ESC model stops a motor 250 ms after its last DShot frame (`signal_loss_ms`), an assumed value. The ESC boards'
speed for a throttle is their bench fit ([boards.md](boards.md#the-esc-boards-speed-curve)); the Puddles backplane's
ESCs keep the linear default (4000 RPM at full throttle), as nothing is known about them.

**Two sources disagree on the T200's thrust.** Blue Robotics' data (`vehicles/talos.toml`, `puddles.toml`) gives
40.7 N at 3153 RPM ahead; riptide's bench fit, which `talos_nereus.toml` follows, 24 N at 3171 RPM (59 % of it).
The bench fit's full-throttle speed (3770 RPM, extrapolated) is also above the datasheet's top speed at 16 V (3533
RPM, with Blue Robotics' ESC). Either the bench's force gauge or the RPM the APD ESCs report (7 pole pairs) is off,
or the thrusters or the supply differ from the datasheet's; a check on a bench would settle which. The emulator follows the firmware's own data for
the ESC and riptide's for the thrust in the files that run the riptide controller.

Snapshots of a robot copy its vehicles and give the copy its own sensor inputs, so `sim_test::sweep` runs
vehicle scenarios side by side (`vehicle_talos.rs` sweeps three depths, `vehicle.rs` three Puddles dive
throttles).
