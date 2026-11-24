"""GrowBot wheeled driver for two SM-S4303R continuous-rotation servos.

Copy this file to the Pico as PicoRobotics.py. Port 1 is the left servo on GP2;
port 3 is the right servo on GP4. servoWrite(port, 0..180) means reverse through
stop to forward, and release(port) sends a neutral pulse.

The servos need a regulated supply and a common ground with the Pico. Neutral is
not an active brake; signal loss and power loss do not guarantee a safe stop.
Calibrate each rest point with the wheels raised before driving. See README.md.
"""

import machine, utime
from machine import Pin, PWM

# ---- wiring and per-servo calibration -----------------------------------
SERVO_PINS = {1: 2, 3: 4}         # left GP2, right GP4
INVERT     = {1: True, 3: False}  # calibrated for this mounting; verify on blocks
MOOD_LED_PIN = 5                  # WS2812-compatible NeoPixel data
MOOD_LED_COUNT = 5                # number of chained LEDs
MOOD_COLORS = {"happy": (0, 32, 0), "sad": (0, 0, 32),
               "neutral": (16, 16, 16), "off": (0, 0, 0)}
# (low pulse, neutral pulse, high pulse), in microseconds. The rest point
# varies by servo and can also be adjusted using its potentiometer.
SERVO_US   = {1: (1000, 1500, 2000),
              3: (1000, 1500, 2000)}

# ---- tuning --------------------------------------------------------------
PWM_FREQ          = 50      # 20 ms servo control frame
DEADBAND_DEG      = 4       # |deg-90| below this = neutral (no drive)
MAX_SPEED         = 0.45    # fraction of pulse range; start low, raise slowly
SPEED_GAIN        = 1.0     # scales command before the MAX_SPEED limit
SLEW_PER_S        = 6.0     # ramp rate; softens hard reversals. 0 = off.
DEADMAN_MS        = 300     # neutral a wheel if ITS OWN input goes quiet
SWEEP_MS          = 20      # ramp and timeout check interval (soft timer)
BOOT_LOCKOUT_MS   = 15000   # ignore commands after power-up to suppress the
                            # boot calibration lurch. 0 = off.
DEBUG_WHEELS      = True   # print sampled received commands and output pulses
DEBUG_INTERVAL_MS = 500     # minimum interval between diagnostic lines

# ---- GAIT_ROLL: make the autonomous walk lane actually roll --------------
# The brain's walk policy emits MIRRORED leg angles (l = 90 - x, r = 90 + x).
# Two servos read that as a gait. Two wheels read it as "spin one way, then the
# other", so the mower just twitches on the spot and never goes anywhere.
#
# We decompose each pose into throttle (both wheels together) and yaw (the
# difference). For teleop and same-sign gestures this is an exact identity and
# changes nothing. When we see the signature of a gait we convert how hard she is
# gaiting into forward roll, and damp the yaw so it wiggles rather than
# counter-rotates.
#
# The gait signature is OSCILLATION, not just "yaw is big". A steady teleop turn
# also has big yaw and near-zero throttle, and must stay a turn: if we keyed off
# magnitude alone, telling the mower to spin on the spot would drive it forward
# instead. So we require yaw to keep CROSSING ZERO (GAIT_CROSS_MIN), which a
# held turn never does.
#
# Result: when GrowBot decides to walk, the mower rolls forward with a slight
# waddle, at a speed proportional to how hard she is trying.
#
# ⚠️ OFF BY DEFAULT, AND HERE IS THE HONEST REASON. The firmware gives this driver
# only servoWrite() and release(). There is no mode bit, so nothing here can tell
# an autonomous gait from a human working the turn stick: both are a mirrored yaw
# oscillation with near-zero net throttle. Measured with GAIT_ROLL on, a ±25 deg
# teleop steering wiggle injected +0.255 mean forward drive, so the mower rolls
# off while the operator believes they are turning on the spot. The injection is
# also forward-only, because a mirrored stream carries no direction information,
# so an autonomous gait can only ever roll TOWARD whatever is in front of it.
#
# Turn this on only for untended autonomous runs: blade removed, boundary wire
# live, and after you have watched it on blocks. For teleop, leave it off.
GAIT_ROLL      = False
GAIT_ROLL_GAIN = 1.0     # forward speed per unit of gait amplitude
GAIT_YAW_KEEP  = 0.35    # how much of the original wiggle to keep (0 = dead straight)
GAIT_MIN_ENV   = 0.08    # gait must be at least this strong before we roll
GAIT_THR_MAX   = 0.25    # ...and net throttle below this (i.e. not a teleop command)
GAIT_CROSS_MIN = 1.2     # ...and yaw must be oscillating this fast, in Hz
GAIT_ENV_ALPHA = 0.10    # envelope smoothing (lower = slower to react)
GAIT_X_DECAY   = 0.96    # zero-cross decay per pose; makes xrate read out in Hz
ROLL_MAX       = 0.60    # clamp on the injected command (also limited by MAX_SPEED)

_DEADBAND = DEADBAND_DEG / 90.0


def _clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


class _Motor:
    def __init__(self, pin, invert, pulse_us):
        self.low_us, self.neutral_us, self.high_us = pulse_us
        if not (0 < self.low_us < self.neutral_us < self.high_us < 1000000 // PWM_FREQ):
            raise ValueError("invalid servo pulse calibration")
        if not (0.0 <= MAX_SPEED <= 1.0 and SPEED_GAIN >= 0.0):
            raise ValueError("invalid servo speed limit or gain")
        self.pwm = PWM(Pin(pin))
        self.pwm.freq(PWM_FREQ)
        self.invert = invert
        self.cur = 0.0                 # current (slew-limited) signed command
        self.target = 0.0
        self.busy = False              # prevent timer re-entry during a write
        self.last = utime.ticks_ms()
        self.last_cmd = self.last      # last time a command arrived (dead-man)
        self.stop()

    def stop(self):
        self.busy = True
        try:
            self._stop()
        finally:
            self.busy = False

    def _stop(self):
        self.target = 0.0
        self.cur = 0.0
        self.pulse_ns = self.neutral_us * 1000
        self.pwm.duty_ns(self.pulse_ns)

    def release(self):
        self.stop()

    def _apply(self, s):
        s = -s if self.invert else s
        s = _clamp(s * SPEED_GAIN, -MAX_SPEED, MAX_SPEED)
        span = self.high_us - self.neutral_us if s >= 0 else self.neutral_us - self.low_us
        self.pulse_ns = int((self.neutral_us + s * span) * 1000)
        self.pwm.duty_ns(self.pulse_ns)

    def drive(self, target):
        self.busy = True
        try:
            now = utime.ticks_ms()
            self.last_cmd = now        # feed the dead-man
            target = _clamp(target, -1.0, 1.0)

            if abs(target) < _DEADBAND:
                self.last = now
                self._stop()
                return
            self.target = target
            self._advance(now)
        finally:
            self.busy = False

    def _advance(self, now):
        dt = utime.ticks_diff(now, self.last) / 1000.0
        self.last = now

        # slew-limit toward target so a full reversal ramps instead of slamming.
        # dt==0 (two commands in the same ms) must NOT snap to target, and dt is
        # capped so the first command after an idle gap still ramps.
        if SLEW_PER_S > 0:
            if dt > 0:
                step = SLEW_PER_S * (0.05 if dt > 0.05 else dt)
                self.cur += _clamp(self.target - self.cur, -step, step)
        else:
            self.cur = self.target
        self._apply(self.cur)


class _FakeI2C:
    """robot-server.py's release() pokes a PCA9685 register; decode it back to a port.
    Kept so this driver also works under the older LAN firmware, not just relay_chip."""
    def __init__(self, owner):
        self._o = owner
    def writeto_mem(self, addr, reg, data):
        off = reg - 0x0B
        if off >= 0 and off % 4 == 0:
            self._o.release(off // 4 + 1)


class _WheelBoard:
    L_PORT, R_PORT = 1, 3

    def __init__(self):
        self.motors = {p: _Motor(pin, INVERT[p], SERVO_US[p])
                       for p, pin in SERVO_PINS.items()}
        self.i2c = _FakeI2C(self)
        self._mood_led = None
        now = utime.ticks_ms()
        self.raw = {self.L_PORT: 0.0, self.R_PORT: 0.0}   # latest signed cmd per side
        self.raw_ts = {self.L_PORT: now, self.R_PORT: now}  # when each side last spoke
        self._dirty_l = False                              # both sides fresh -> mix
        self._dirty_r = False
        self._seen_l = False                               # has this side ever spoken?
        self._seen_r = False
        self.env = 0.0                                     # gait amplitude envelope
        self.xrate = 0.0                                   # yaw zero-crossing rate
        self._ysign = 0                                    # last yaw sign seen
        self._boot = now
        self._debug_last = now
        # Sweep from a timer so a dead command stream cannot leave a wheel running.
        # rp2 = SOFT TIMERS ONLY: the id must be -1. Timer(0) raises ValueError here.
        self._wd = machine.Timer(-1)
        self._wd.init(period=SWEEP_MS, mode=machine.Timer.PERIODIC, callback=self._sweep)

    def _sweep(self, t):
        now = utime.ticks_ms()
        for m in self.motors.values():
            if not m.busy and m.target != 0.0:
                if utime.ticks_diff(now, m.last_cmd) > DEADMAN_MS:
                    m.stop()
                elif m.cur != m.target:
                    m._advance(now)

    def _locked_out(self):
        if not BOOT_LOCKOUT_MS:
            return False
        return utime.ticks_diff(utime.ticks_ms(), self._boot) < BOOT_LOCKOUT_MS

    def _mix(self):
        """Latest L/R pose -> actual wheel commands. Identity unless GAIT_ROLL fires."""
        # PER-SIDE DEAD-MAN. The timer sweep below only catches a stream that has
        # died completely, because _mix drives BOTH motors and so refreshes both
        # dead-man clocks on any single write. If one side's pose stream stops
        # while the other keeps arriving, that stale side would otherwise hold its
        # last command forever (measured: still driving at 0.35 two seconds after
        # its stream died). So expire the INPUT, per side, before mixing.
        now = utime.ticks_ms()
        for p in (self.L_PORT, self.R_PORT):
            if self.raw[p] != 0.0 and utime.ticks_diff(now, self.raw_ts[p]) > DEADMAN_MS:
                self.raw[p] = 0.0
        l = self.raw[self.L_PORT]
        r = self.raw[self.R_PORT]
        thr = (l + r) / 2.0            # common mode: both wheels together
        yaw = (l - r) / 2.0            # differential: spin

        if GAIT_ROLL:
            self.env += (abs(yaw) - self.env) * GAIT_ENV_ALPHA
            # zero-crossing rate: high for an oscillating gait, zero for a held turn
            sign = 1 if yaw > _DEADBAND else (-1 if yaw < -_DEADBAND else 0)
            self.xrate *= GAIT_X_DECAY
            if sign and self._ysign and sign != self._ysign:
                self.xrate += 1.0
            if sign:
                self._ysign = sign
            if (self.xrate > GAIT_CROSS_MIN and self.env > GAIT_MIN_ENV
                    and abs(thr) < GAIT_THR_MAX):
                thr += _clamp(GAIT_ROLL_GAIN * self.env, 0.0, ROLL_MAX)
                yaw *= GAIT_YAW_KEEP

        self.motors[self.L_PORT].drive(_clamp(thr + yaw, -1.0, 1.0))
        self.motors[self.R_PORT].drive(_clamp(thr - yaw, -1.0, 1.0))
        if DEBUG_WHEELS and utime.ticks_diff(now, self._debug_last) >= DEBUG_INTERVAL_MS:
            self._debug_last = now
            left = self.motors[self.L_PORT]
            right = self.motors[self.R_PORT]
            print("wheels mixed deg L=%.1f R=%.1f; pulse us L=%.1f R=%.1f"
                  % (90 + l * 90, 90 + r * 90,
                     left.pulse_ns / 1000, right.pulse_ns / 1000))

    def servoWritePair(self, left, right):
        if self._locked_out():
            for m in self.motors.values():
                m.stop()
            return
        now = utime.ticks_ms()
        for port, deg in ((self.L_PORT, left), (self.R_PORT, right)):
            self.raw[port] = (_clamp(deg, 0, 180) - 90) / 90.0
            self.raw_ts[port] = now
        self._seen_l = self._seen_r = True
        self._dirty_l = self._dirty_r = False
        self._mix()

    def servoWrite(self, port, deg):
        if port not in self.raw:
            return
        if self._locked_out():
            for m in self.motors.values():
                m.stop()
            return
        deg = 0 if deg < 0 else 180 if deg > 180 else deg
        self.raw[port] = (deg - 90) / 90.0      # 90 = stop; 0..180 -> -1..+1
        self.raw_ts[port] = utime.ticks_ms()
        # Mix ONCE PER POSE, when both sides are fresh. The firmware writes L then
        # R back to back, so mixing on every write applied (new L, old R) to the
        # hardware, and the R write landed in the same millisecond where dt==0 left
        # self.cur untouched -- so the half-stale output stood for the whole frame
        # and the right wheel ran a pose behind, twitching the yaw on every step.
        if port == self.L_PORT:
            self._dirty_l = True
            self._seen_l = True
            other, seen_other = self.R_PORT, self._seen_r
        else:
            self._dirty_r = True
            self._seen_r = True
            other, seen_other = self.L_PORT, self._seen_l
        # Fall through anyway if the OTHER side has gone quiet past the dead-man, so
        # a one-sided stream still reaches the wheels (with the dead side expired to
        # neutral by _mix) instead of freezing the mixer. Requires that side to have
        # spoken at least once: at boot neither has, and without this the very first
        # write would look "stale" and mix a half pose.
        stale = seen_other and \
            utime.ticks_diff(self.raw_ts[port], self.raw_ts[other]) > DEADMAN_MS
        if self._dirty_l and self._dirty_r:
            self._dirty_l = False
            self._dirty_r = False
            self._mix()
        elif stale:
            self._dirty_l = False
            self._dirty_r = False
            self._mix()
            # Keep this fresh side available for the other side's return.
            if port == self.L_PORT:
                self._dirty_l = True
            else:
                self._dirty_r = True

    def release(self, port):
        m = self.motors.get(port)
        if m:
            self.raw[port] = 0.0
            self.env = 0.0
            self.xrate = 0.0
            self._ysign = 0
            m.release()

    def set_mood(self, mood):
        if not isinstance(mood, str) or mood not in MOOD_COLORS:
            raise ValueError("unsupported mood")
        if self._mood_led is None:
            if not isinstance(MOOD_LED_COUNT, int) or isinstance(MOOD_LED_COUNT, bool) or MOOD_LED_COUNT < 1:
                raise ValueError("MOOD_LED_COUNT must be a positive integer")
            import neopixel
            self._mood_led = neopixel.NeoPixel(Pin(MOOD_LED_PIN), MOOD_LED_COUNT)
        for i in range(MOOD_LED_COUNT):
            self._mood_led[i] = MOOD_COLORS[mood]
        self._mood_led.write()


def KitronikPicoRobotics():
    """Factory, name-compatible with the servo driver the firmware imports."""
    print("PicoRobotics: SM-S4303R wheels on", SERVO_PINS)
    if BOOT_LOCKOUT_MS:
        print("  boot lockout %d ms: neutral, calibration stretch suppressed"
              % BOOT_LOCKOUT_MS)
    return _WheelBoard()
