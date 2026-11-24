# GrowBot wheels with SM-S4303R servos (Pico 2 W)

This port adapts the BTS7960 DC-motor driver for two SpringRC SM-S4303R
continuous-rotation servos. It is a drop-in `PicoRobotics.py` for GrowBot's
Pico firmware, not a driver for the original mower motors. The original
[automower port](../automower/) remains available for BTS7960 motors. The servo
port's commands are wheel velocities: port 1 is left, port 3 is right, `90` is stop,
and distance from `90` requests speed in either direction.

## Wiring and power

| Connection | Left | Right |
| --- | --- | --- |
| Servo signal | Pico GP2 | Pico GP4 |
| Servo power | Regulated servo supply | Same supply |
| Servo ground | Pico and supply ground | Pico and supply ground |

Disconnect the BTS7960 inputs; they are not used by this driver. Supply the
servos from a properly regulated 4.8-6 V supply sized for both servos' startup
and stall currents. Do not connect a mower battery directly to the servos or
power the motors from a GPIO pin. Power the Pico within its own specified
limits and connect all grounds together. Check the servo's signal-level
compatibility with the Pico's 3.3 V output. GP2 and GP4 use separate PWM slices;
other pins sharing those slices cannot use a different PWM frequency.

## Install

Flash the normal GrowBot Pico build first, then from this directory run:

```bash
mpremote cp PicoRobotics.py :PicoRobotics.py
mpremote reset
```

The driver sends a neutral pulse as it initializes and ignores movement commands
for 15 seconds to suppress the firmware's boot calibration. It is not a hardware
interlock: a reset, loss of signal or power, software hang, or uncalibrated
neutral can still cause motion. Use an independent physical power cutoff.

## Calibration and first test

1. Securely raise both wheels off the ground. Keep people clear and have the
   independent power cutoff ready.
2. Power up and wait out the 15-second lockout. Both wheels must remain still.
   The SM-S4303R nominal rest point is 1500 us, but each unit can differ. Adjust
   its center potentiometer or its own neutral entry in `SERVO_US` until a
   continuous neutral pulse stops it. Repeat this check after changes or reset.
3. Use the GrowBot tester to send small movements around `90`, with its
   inversion checkboxes unchecked. Check forward, back and both turns. The
   driver's default left inversion is calibrated for one mounting; adjust
   `INVERT` if your mounting differs. Then check the app separately, as it
   may use different command signs.
4. Test `90,90`, `release`, stopping the command stream, and disconnecting the
   link. Both wheels must stop. Repeat the test with only one wheel's command
   stream interrupted. Test behavior on loss of Pico signal and servo power
   separately; it depends on the servo and is not guaranteed by this driver.
5. Only test on the ground on a flat, clear, enclosed surface after verifying
   the above. Neutral is **not an active brake** and is not a slope-holding
   mechanism. The servo can be damaged by stalling or back-driving it.

Use the app's wheels mode (`?wheels=1`); the normal walking gait is not a
velocity command. `GAIT_ROLL` remains off because it cannot distinguish a gait
from steering. Gestures can command wheel motion, so test these with wheels
raised too.

### App driving and diagnosis

Open `https://growbot.dev/v3/next.html?wheels=1&id=YOUR_PAIRING_CODE&mode=legs`.
In the repository's app snapshot, `wheels=1` selects the wheeled drive lane and
also enables the body/legs mode. `mode=wheels` is not a separate recognized
mode: with `wheels=1` or an `id` it also resolves to legs mode. Hosted app
versions can change independently of this snapshot.

The app's `runDrive` applies the phone's `L_SIGN` and `R_SIGN` calibration.
The stock body defaults to `-1` and `+1`, and saved browser values can override
them; `SWAP` can also exchange the two output channels. These settings are
separate from the Pico's `INVERT` values. Direct-pose controls and the desktop
tester do not necessarily apply the same sign mapping as an app drive. For
example, if raw 114/114 drives both wheels forward on your robot, stock signs
turn an app forward request into roughly 66/114 (a pivot) and a left request
into roughly 114/114 (forward). These signs can vary by app lane and saved
calibration. First observe both the wire values and the actual wheel directions.

With the wheels raised and an independent cutoff ready, set `DEBUG_WHEELS =
True` in `PicoRobotics.py`, copy it to the Pico, and reset. After the 15-second
lockout, use `mpremote connect COM27 repl` to see sampled lines such as:

```
wheel wire lane=pose L=66 R=114 at_ms=12345
wheels mixed deg L=66.0 R=114.0; pulse us L=1366.7 R=1633.3
```

The `wheel wire` line is the complete pair delivered to the driver, after app
signs and swap: `pose` is a direct pose, while `act` is a gesture-engine output.
The `wheels mixed` line is a separate, rate-limited sample after stale inputs
expire; the samples need not represent the same frame. The port's relay
firmware passes the two values to the driver together; single-wheel writes
from other firmware can still produce a half-updated pair. Pulse widths
are requested from PWM after deadband, slew, gain and inversion; they are not
measured wheel speed. These lines are sampled every 500 ms per lane and may
miss a short command; the mixed pulse log also does not show subsequent timer
ramp updates. For each accepted `act` message, `wheel act` logs its queued
duration, number of pending keyframes and the first eight validated pending
`(left, right, duration_ms)` keyframes. An appended plan can leave earlier
keyframes in that list. Rejected acts are logged separately.
The wire and act logs require this port's `relay_chip.py` as
`main.py`. A quiet stream or a lockout produces no mixed lines. Normal forward should turn both wheels
forward, while a pivot should turn them in opposite physical directions. Tune
the Pico `INVERT` settings against the **app's** forward and pivot commands and
check the physical left/right mapping. Recheck direct-pose controls afterward:
they may use different conventions. Turn debug off after diagnosis; serial
printing can affect timing.

An accepted `wheel act` line confirms the Pico received the plan, not that the
wheels completed it. A `wheel act interrupted by pose` line means a direct
pose cancelled a running gesture; `wheel stop` reports a received stop. No
`wheel act` line does not prove a relay failure: the app may have sent a pose,
not sent a movement, or lost its connection. Compare with the app's status
and any displayed motion or sensor errors. The app snapshot's wheel-drive lane
requires fresh phone tilt data and can also defer movement for duty limits.

At the stock gain and pulse calibration, an app command 24 degrees from neutral
requests approximately 133 us away from neutral, whereas a 90-degree
command is capped at 225 us. The app can use smaller commands and ramps them in
and out; actual servo speed depends on load and each servo's deadband. If
direction is correct but motion remains too weak, try raising `SPEED_GAIN`
gradually (for example 1.5) with wheels raised; `MAX_SPEED` still caps pulse
displacement. Re-test neutral, timeout and direction under load. Increasing
gain cannot fix incorrect signs and may make motion unexpectedly faster.

## Tuning

| Setting | Default | Meaning |
| --- | --- | --- |
| `SERVO_PINS` | left GP2, right GP4 | Servo signal pins |
| `SERVO_US` | `(1000, 1500, 2000)` per wheel | Low, neutral and high pulse widths in microseconds |
| `INVERT` | left `True`, right `False` | Reverse the pulse direction for that wheel; verify for your mounting |
| `PWM_FREQ` | 50 Hz | Servo control frame frequency |
| `DEADBAND_DEG` | 4 | Commands close to 90 send neutral |
| `MAX_SPEED` | 0.45 | Cap on the fraction of the configured pulse range |
| `SPEED_GAIN` | 1.0 | Scale command magnitude before applying the cap |
| `SLEW_PER_S` | 6.0 | Signed command change per second; zero disables slew. The timer continues the ramp between incoming commands. |
| `DEADMAN_MS` | 300 | Per-wheel command timeout, checked every 20 ms |
| `SWEEP_MS` | 20 | Soft-timer interval for ramp and timeout checks |
| `BOOT_LOCKOUT_MS` | 15000 | Ignore boot-calibration movement; zero disables |
| `GAIT_ROLL` | `False` | Leave off for wheels mode |
| `DEBUG_WHEELS` | `True` | Sample input angles and requested output pulses on the serial console; disable after diagnosis |

For example, the default 1500 us center and 1000-2000 us limits produce a
maximum command range of 1275-1725 us. Command magnitude is proportional to
the pulse offset until `MAX_SPEED` is reached; actual speed need not be linear
or equal between wheels, and this driver has no encoder feedback or stall
detection. `MAX_SPEED` is not a motor PWM duty cycle or a guaranteed RPM limit.

The timer continues the slew ramp after an isolated command, making short
gestures potentially stronger than on earlier versions. It does not refresh
the dead-man: after 300 ms without a new wheel input, the timer sends neutral.
The timeout is a soft-timer safeguard and can be delayed by MicroPython or a
blocked interpreter. The gesture engine suppresses repeated identical poses, so
a held gesture can time out early; wheel-drive mode regularly refreshes its
commands. Never disable the timeout to make a gesture hold, and do not rely on
software alone to stop a moving platform.

Servo details: [Pololu SM-S4303R product information](https://www.pololu.com/product/1248).

## Desktop tester and mood indicator

The desktop tester connects as an app client through the GrowBot relay. It uses
the Pico's pairing code; it does not talk directly over USB or act as a relay.
Do not run the GrowBot app and tester as competing controllers. The relay has
no authentication: treat the pairing code like a password.

The custom mood buttons also require this port's
[`relay_chip.py`](relay_chip.py), installed as `main.py` on the Pico. It is a
port-specific copy of the shared firmware with the mood handler added; the
shared [`firmware/relay_chip.py`](../../firmware/relay_chip.py) does not handle
mood messages. From this directory, after flashing the normal GrowBot firmware:

```powershell
mpremote cp relay_chip.py :main.py
mpremote reset
```

The existing app does **not** currently send mood messages. The
tester sends the custom JSON `{"t":"state","rid":1,"mood":"happy"}`; supported
values are `happy`, `sad`, `neutral`, and `off`. The Pico returns an `ack` with
the same request ID and `ok` or an error. Without the updated firmware, no
acknowledgement arrives; with a driver lacking mood support, the error is
`state_unavailable`. Mood does not control movement.

Connect the first WS2812-compatible NeoPixel's data input to GP5, with each
LED's data output connected to the next LED's input. Use a shared ground and a
supply sized for the entire chain. For 5 V NeoPixels, use a suitable 3.3 V
to 5 V logic-level shifter and follow the LED manufacturer's power and wiring
recommendations. GP5 is no longer an H-bridge output. Set `MOOD_LED_COUNT` in
the servo driver to the number of LEDs in the chain (default: 5); every LED
shows the same mood color. Default low-brightness colors are green for happy,
blue for sad, white for neutral and black for off; edit `MOOD_COLORS` to adjust
them. The chain is initialized on the first mood command; its state before then
is not guaranteed.

On the desktop, from this directory:

```bash
python -m pip install -r requirements-tester.txt
python servo_tester.py
```

Tkinter must be available in the desktop Python installation. Enter the code
printed by the Pico and connect. Wait for **Pico awake**, then use the buttons
to send a timed step (100-2000 ms) in the selected direction and speed. The
tester streams poses every 100 ms and sends neutral plus `stop` afterward.
STOP and disconnect also attempt neutral and stop. These are software
commands, not a physical emergency stop. A dropped or delayed connection can
prevent them reaching the Pico; its per-wheel timeout is an additional software
safeguard. Raise the wheels, verify the direction checkboxes against the
physical mounting, and keep an independent power cutoff ready.

Speed is the selected displacement in degrees from `90`, not RPM or a
percentage; the driver's `MAX_SPEED` cap still applies. The tester's inversion
checkboxes are independent of the driver's `INVERT` settings and the app's
saved calibration. With the driver's left inversion enabled, leave both
tester inversion boxes unchecked unless your particular mounting requires
otherwise. The Wi-Fi status button requests existing Pico link
information; responses and custom state acknowledgements appear in the log.
