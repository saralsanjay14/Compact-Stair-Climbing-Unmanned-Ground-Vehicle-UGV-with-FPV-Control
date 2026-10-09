"""
Stair-climbing UGV controller (ESP32, MicroPython)
TN-IMPACT 2026, TN126203

Hardware (see wiring diagram):
  3x TB6612FNG on shared lines (A = left motors, B = right motors)
  MPU6050 on I2C, HC-SR04 ultrasonic, RF receiver on UART2,
  battery divider (R1 30k / R2 10k) on ADC, 12 V 3S Li-ion pack.

RF command protocol (1 byte per command, adapt to your RF module):
  'F' forward   'B' backward   'L' spin left   'R' spin right   'S' stop
  '1'..'9'      set speed level (1 = slowest, 9 = full)
"""

from machine import Pin, PWM, I2C, UART, ADC, time_pulse_us
from time import ticks_ms, ticks_diff, sleep_ms, sleep_us
import math

# ---------------- Pin configuration ----------------
PIN_PWMA, PIN_AIN1, PIN_AIN2 = 25, 26, 27     # left side (all 3 drivers)
PIN_PWMB, PIN_BIN1, PIN_BIN2 = 13, 32, 33     # right side (all 3 drivers)
PIN_STBY = 14
PIN_SDA, PIN_SCL = 21, 22
PIN_TRIG, PIN_ECHO = 5, 18
PIN_RF_RX, PIN_RF_TX = 16, 17
PIN_VBAT = 34

# ---------------- Tunable settings ----------------
PWM_FREQ = 5000
RF_BAUD = 9600
COMMAND_TIMEOUT_MS = 500        # fail-safe: stop if no command received
DEFAULT_LEVEL = 6               # default speed level (1-9)
RAMP_STEP = 0.08                # max speed change per loop (smooths current)
LOOP_MS = 20

DIVIDER_RATIO = (30000 + 10000) / 10000   # R1 30k, R2 10k -> Vbat = Vadc * 4
VBAT_WARN = 10.5                # 3S pack: ~3.5 V per cell
VBAT_CUTOFF = 9.6               # 3S pack: ~3.2 V per cell, stop driving

CLIMB_PITCH_DEG = 10            # above this nose-up angle, add torque boost
CLIMB_BOOST = 0.25              # extra duty fraction while climbing
TIP_LIMIT_DEG = 50              # stop if roll or pitch exceeds this

OBSTACLE_CM = 15
OBSTACLE_STOP = False           # keep False: stair risers look like obstacles


# ---------------- Motor side (one TB6612 channel, 3 motors) ----------------
class Side:
    def __init__(self, pwm_pin, in1, in2):
        self.pwm = PWM(Pin(pwm_pin), freq=PWM_FREQ, duty_u16=0)
        self.in1 = Pin(in1, Pin.OUT, value=0)
        self.in2 = Pin(in2, Pin.OUT, value=0)

    def drive(self, speed):
        """speed in -1.0 .. 1.0"""
        speed = max(-1.0, min(1.0, speed))
        if speed > 0:
            self.in1.value(1)
            self.in2.value(0)
        elif speed < 0:
            self.in1.value(0)
            self.in2.value(1)
        else:
            self.in1.value(0)
            self.in2.value(0)
        self.pwm.duty_u16(int(abs(speed) * 65535))


class Drive:
    def __init__(self):
        self.stby = Pin(PIN_STBY, Pin.OUT, value=0)
        self.left = Side(PIN_PWMA, PIN_AIN1, PIN_AIN2)
        self.right = Side(PIN_PWMB, PIN_BIN1, PIN_BIN2)
        self.cur_l = 0.0
        self.cur_r = 0.0

    def enable(self, on):
        self.stby.value(1 if on else 0)

    @staticmethod
    def _ramp(cur, target):
        if target > cur + RAMP_STEP:
            return cur + RAMP_STEP
        if target < cur - RAMP_STEP:
            return cur - RAMP_STEP
        return target

    def update(self, target_l, target_r):
        self.cur_l = self._ramp(self.cur_l, target_l)
        self.cur_r = self._ramp(self.cur_r, target_r)
        self.left.drive(self.cur_l)
        self.right.drive(self.cur_r)

    def stop_now(self):
        self.cur_l = self.cur_r = 0.0
        self.left.drive(0)
        self.right.drive(0)


# ---------------- MPU6050 (accelerometer tilt) ----------------
class MPU6050:
    ADDR = 0x68

    def __init__(self, i2c):
        self.i2c = i2c
        self.ok = False
        try:
            self.i2c.writeto_mem(self.ADDR, 0x6B, b"\x00")   # wake up
            self.ok = True
        except OSError:
            print("MPU6050 not found")

    def tilt(self):
        """Returns (pitch, roll) in degrees, or (0, 0) if unavailable."""
        if not self.ok:
            return 0.0, 0.0
        try:
            d = self.i2c.readfrom_mem(self.ADDR, 0x3B, 6)
        except OSError:
            return 0.0, 0.0
        ax = self._s16(d[0], d[1])
        ay = self._s16(d[2], d[3])
        az = self._s16(d[4], d[5])
        pitch = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
        roll = math.degrees(math.atan2(ay, az))
        return pitch, roll

    @staticmethod
    def _s16(hi, lo):
        v = (hi << 8) | lo
        return v - 65536 if v & 0x8000 else v


# ---------------- HC-SR04 ----------------
class Ultrasonic:
    def __init__(self):
        self.trig = Pin(PIN_TRIG, Pin.OUT, value=0)
        self.echo = Pin(PIN_ECHO, Pin.IN)

    def distance_cm(self):
        self.trig.value(0)
        sleep_us(2)
        self.trig.value(1)
        sleep_us(10)
        self.trig.value(0)
        t = time_pulse_us(self.echo, 1, 30000)   # 30 ms timeout
        if t < 0:
            return None
        return t * 0.0343 / 2


# ---------------- Battery monitor ----------------
class Battery:
    def __init__(self):
        self.adc = ADC(Pin(PIN_VBAT))
        self.adc.atten(ADC.ATTN_11DB)

    def volts(self):
        n = 16
        total = 0
        for _ in range(n):
            try:
                total += self.adc.read_uv() / 1_000_000   # calibrated
            except AttributeError:
                total += self.adc.read() / 4095 * 3.3     # rough fallback
        return (total / n) * DIVIDER_RATIO


# ---------------- Main program ----------------
def main():
    drive = Drive()
    imu = MPU6050(I2C(0, sda=Pin(PIN_SDA), scl=Pin(PIN_SCL), freq=400000))
    sonar = Ultrasonic()
    batt = Battery()
    rf = UART(2, baudrate=RF_BAUD, rx=PIN_RF_RX, tx=PIN_RF_TX)

    level = DEFAULT_LEVEL
    cmd = "S"
    last_cmd = ticks_ms()
    last_report = ticks_ms()
    locked_out = False
    vbat = batt.volts()
    dist = None
    n = 0

    drive.enable(True)
    print("UGV ready. Battery: %.2f V" % vbat)

    while True:
        # 1. Read RF commands
        if rf.any():
            for ch in rf.read():
                c = chr(ch)
                if c in "FBLRS":
                    cmd = c
                    last_cmd = ticks_ms()
                elif c in "123456789":
                    level = int(c)
                    last_cmd = ticks_ms()

        # 2. Sensors (slower ones every few loops)
        pitch, roll = imu.tilt()
        n += 1
        if n % 5 == 0:
            dist = sonar.distance_cm()
        if n % 25 == 0:
            vbat = batt.volts()

        # 3. Safety checks
        failsafe = ticks_diff(ticks_ms(), last_cmd) > COMMAND_TIMEOUT_MS
        tipped = abs(roll) > TIP_LIMIT_DEG or abs(pitch) > TIP_LIMIT_DEG
        if vbat < VBAT_CUTOFF:
            locked_out = True
        blocked = (OBSTACLE_STOP and cmd == "F"
                   and dist is not None and dist < OBSTACLE_CM)

        # 4. Work out target speeds
        if failsafe or tipped or locked_out or blocked or cmd == "S":
            tl = tr = 0.0
        else:
            base = level / 9
            if pitch > CLIMB_PITCH_DEG and cmd == "F":
                base = min(1.0, base + CLIMB_BOOST)    # extra torque on stairs
            if cmd == "F":
                tl, tr = base, base
            elif cmd == "B":
                tl, tr = -base, -base
            elif cmd == "L":
                tl, tr = -base, base
            else:
                tl, tr = base, -base

        # 5. Drive (ramped)
        if tl == 0.0 and tr == 0.0 and (failsafe or tipped or locked_out):
            drive.stop_now()
        else:
            drive.update(tl, tr)

        # 6. Telemetry once per second
        if ticks_diff(ticks_ms(), last_report) > 1000:
            last_report = ticks_ms()
            print("cmd=%s lvl=%d pitch=%.0f roll=%.0f dist=%s vbat=%.2f%s%s" % (
                cmd, level, pitch, roll,
                "-" if dist is None else "%.0f" % dist, vbat,
                " LOW" if vbat < VBAT_WARN else "",
                " LOCKOUT" if locked_out else ""))

        sleep_ms(LOOP_MS)


try:
    main()
except KeyboardInterrupt:
    pass
finally:
    # make sure motors are off if the program stops
    for p in (PIN_PWMA, PIN_PWMB):
        PWM(Pin(p)).deinit()
    Pin(PIN_STBY, Pin.OUT, value=0)
