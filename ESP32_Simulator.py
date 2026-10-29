"""
ESP32 emulator - universal sketch runner (Python + tkinter, no extra packages).

Input: any Arduino-style sketch in C++ (.ino, translated by esp32_cpp.py) or the same API in Python.

Emulated (behaviour of the real chip, not instruction level):
  * GPIO: 34-39 input-only, 6-11 flash (forbidden), strapping warnings, pull-ups, VIL/VIH thresholds,
    attachInterrupt (RISING/FALLING/CHANGE) executed on the interrupt/timer context
  * ADC1/ADC2 (12 bit, attenuation, noise, clipping), DAC 8 bit, touch pads
  * LEDC PWM (80 MHz timer limit, 16 channels), MCPWM (timer/operator/comparator/generator, ISR on empty)
  * Hardware timers, esp_timer, FreeRTOS tasks/queues/semaphores on 2 cores, 1 kHz tick
  * UART0-2 (bit-exact TX waveform on the pins), I2C with virtual register devices, SPI
  * Emulated clock with adjustable speed: heavy sketches (e.g. 100 kHz PWM ISR) run in slow motion
    so Python can keep up; "Auto" lowers the speed automatically when the emulation lags.

Any GPIO voltage can be shown on the scope; inputs can be driven with a slider.
"""
import bisect
import heapq
import math
import operator
import os
import queue
import random
import re
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, filedialog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import esp32_cpp  # noqa: E402

try:
    import numpy as np
except ImportError:  # optional: only used for exact dense-waveform rendering
    np = None

if sys.platform == "win32":  # 1 ms timer resolution instead of the default ~15.6 ms
    try:
        import ctypes
        ctypes.windll.winmm.timeBeginPeriod(1)
    except Exception:
        pass

VDD = 3.3
INPUT, OUTPUT, INPUT_PULLUP, INPUT_PULLDOWN, OUTPUT_OPEN_DRAIN = 0x01, 0x03, 0x05, 0x09, 0x13
LOW, HIGH = 0, 1
RISING, FALLING, CHANGE = 1, 2, 3
ATTEN = {0: 1.1, 1: 1.5, 2: 2.2, 3: 3.9}
ADC1 = {36: 0, 37: 1, 38: 2, 39: 3, 32: 4, 33: 5, 34: 6, 35: 7}
ADC2 = {4: 0, 0: 1, 2: 2, 15: 3, 13: 4, 12: 5, 14: 6, 27: 7, 25: 8, 26: 9}
TOUCH = {4: 0, 0: 1, 2: 2, 15: 3, 13: 4, 12: 5, 14: 6, 27: 7, 33: 8, 32: 9}
DAC_PINS = (25, 26)
VALID_GPIO = [g for g in range(0, 40) if g not in (20, 24, 28, 29, 30, 31)]
FLASH_PINS = range(6, 12)
INPUT_ONLY = range(34, 40)
STRAPPING = {0: "boot mode", 2: "boot mode", 5: "SDIO timing", 12: "flash voltage (MTDI)", 15: "boot log"}
MAX_HISTORY = 300000


class StopEmulation(BaseException):
    pass


class EspError(Exception):
    pass


# --------------------------------------------------------------------------------------------------
# runtime helpers used by translated code
# --------------------------------------------------------------------------------------------------
class _Cell:
    """Box for variables whose address is taken (&x)."""
    __slots__ = ("v",)

    def __init__(self, v=None):
        self.v = v

    def __getitem__(self, i):
        return self.v

    def __setitem__(self, i, x):
        self.v = x


class _Dummy:
    """Permissive stand-in for library objects that are not emulated."""

    def __init__(self, name="?"):
        object.__setattr__(self, "_n", name)

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        d = _Dummy(f"{self._n}.{k}")
        object.__setattr__(self, k, d)
        return d

    def __call__(self, *a, **k):
        return 1

    def __bool__(self):
        return True

    def __int__(self):
        return 0

    __index__ = __int__

    def __float__(self):
        return 0.0

    def __getitem__(self, i):
        return self

    def __setitem__(self, i, v):
        pass

    def __str__(self):
        return ""

    def __add__(self, o):
        return o

    __radd__ = __sub__ = __rsub__ = __mul__ = __rmul__ = __add__

    def __len__(self):
        return 0


def _idiv(a, b):
    q = abs(int(a)) // abs(int(b))
    return q if (a < 0) == (b < 0) else -q


def _imod(a, b):
    return int(a) - int(b) * _idiv(a, b)


def _wrap(x, bits):
    m = 1 << bits
    x &= m - 1
    return x - m if x >= m >> 1 else x


def _i8(x):
    return _wrap(int(x), 8)


def _i16(x):
    return _wrap(int(x), 16)


def _i32(x):
    return _wrap(int(x), 32)


def _i64(x):
    return _wrap(int(x), 64)


def _s(x):
    if isinstance(x, str):
        return x
    if isinstance(x, bool):
        return "1" if x else "0"
    if isinstance(x, float):
        return "%.2f" % x
    if x is None:
        return ""
    return str(x)


def _sf(x, d=2):
    return "%.*f" % (int(d), x)


def _sbase(x, base):
    x = int(x)
    if base == 16:
        return "%X" % (x & 0xFFFFFFFF if x < 0 else x)
    if base == 2:
        return bin(x & 0xFFFFFFFF if x < 0 else x)[2:]
    if base == 8:
        return "%o" % (x & 0xFFFFFFFF if x < 0 else x)
    return str(x)


def _stoi(s):
    m = re.match(r"\s*[-+]?\d+", s)
    return int(m.group()) if m else 0


def _stof(s):
    m = re.match(r"\s*[-+]?(\d+\.?\d*|\.\d+)([eE][-+]?\d+)?", s)
    return float(m.group()) if m else 0.0


def _charat(s, i):
    return ord(s[i]) if 0 <= i < len(s) else 0


def _cmp(a, b):
    return (a > b) - (a < b)


def _sremove(s, i, n=None):
    return s[:i] if n is None else s[:i] + s[i + n:]


def _ssetchar(s, i, c):
    return s[:i] + chr(c) + s[i + 1:] if 0 <= i < len(s) else s


def _cp(x):
    import copy
    return copy.copy(x)


def _pad(lst, n, d):
    return lst + [d] * (n - len(lst)) if len(lst) < n else lst


def _seti(o, i, v):
    o[i] = v
    return v


def _seta(o, k, v):
    setattr(o, k, v)
    return v


def _pinc(o, i, d, pre):
    o[i] += d
    return o[i] if pre else o[i] - d


def _pinca(o, k, d, pre):
    setattr(o, k, getattr(o, k) + d)
    return getattr(o, k) if pre else getattr(o, k) - d


def _post(old, new):
    return old


_FMT = re.compile(r"%([-+ #0]*)(\d+|\*)?(?:\.(\d+|\*))?(hh|h|ll|l|z|j|t|L)?([diouxXeEfFgGcsp%])")


def _cfmt(fmt="", *args):
    it = iter(args)

    def rep(m):
        flags, w, pr, _, conv = m.groups()
        if conv == "%":
            return "%"
        if w == "*":
            w = str(next(it, 0))
        if pr == "*":
            pr = str(next(it, 0))
        a = next(it, 0)
        spec = "%" + flags + (w or "") + ("." + pr if pr is not None else "")
        try:
            if conv in "di":
                return (spec + "d") % int(a)
            if conv in "ouxX":
                a = int(a)
                return (spec + ("d" if conv == "u" else conv)) % (a & 0xFFFFFFFF if a < 0 else a)
            if conv in "eEfFgG":
                return (spec + conv) % float(a)
            if conv == "c":
                return (spec + "s") % (chr(a) if isinstance(a, int) else str(a))
            if conv == "p":
                return "0x%08x" % (id(a) & 0xFFFFFFFF)
            return (spec + "s") % ("(null)" if a is None else _s(a))
        except (TypeError, ValueError):
            return str(a)
    return _FMT.sub(rep, fmt)


def _dtostrf(v, width, prec):
    return ("%*.*f" % (int(width), int(prec), v))


# --------------------------------------------------------------------------------------------------
# clock / event engine
# --------------------------------------------------------------------------------------------------
class Clock:
    def __init__(self):
        self.scale = 1.0
        self.base_r = time.perf_counter()
        self.base_e = 0.0
        self.frozen = None

    def now(self):
        if self.frozen is not None:
            return self.frozen
        return self.base_e + (time.perf_counter() - self.base_r) * self.scale

    def set_scale(self, s):
        e = self.now()
        self.base_e, self.base_r, self.scale = e, time.perf_counter(), s

    def freeze(self):
        self.frozen = self.now()


class Ev:
    __slots__ = ("t", "fn", "period", "dead")

    def __init__(self, t, fn, period):
        self.t, self.fn, self.period, self.dead = t, fn, period, False


class Engine(threading.Thread):
    """Runs timer callbacks / interrupts at emulated times (core 0 interrupt + timer context)."""

    def __init__(self, esp):
        super().__init__(daemon=True, name="timer/isr")
        self.esp = esp
        self.heap = []
        self.cond = threading.Condition()
        self.seq = 0
        self.lag = 0.0

    def schedule(self, t, fn, period=None):
        ev = Ev(t, fn, period)
        with self.cond:
            self.seq += 1
            heapq.heappush(self.heap, (t, self.seq, ev))
            self.cond.notify()
        return ev

    def call(self, fn):
        return self.schedule(self.esp.now(), fn)

    def cancel(self, ev):
        if ev:
            ev.dead = True

    def run(self):
        clk = self.esp.clock
        while not self.esp.stop:
            with self.cond:
                if not self.heap:
                    self.cond.wait(0.05)
                    continue
                t, _, ev = self.heap[0]
                if ev.dead:
                    heapq.heappop(self.heap)
                    continue
                now = clk.now()
                # batch events that are due within a few ms of real time: OS wake-ups cost ~1 ms each
                if t > now + min(0.004 * clk.scale, 0.0005):
                    self.cond.wait(max(min((t - now) / clk.scale, 0.02), 0.0005))
                    continue
                heapq.heappop(self.heap)
            self.lag = 0.97 * self.lag + 0.03 * max((now - t) / clk.scale, 0.0)
            try:
                ev.fn()
            except StopEmulation:
                return
            except EspError as e:
                self.esp.log(f"[timer/isr] ESP error: {e}")
            except Exception as e:
                self.esp.log(f"Guru Meditation Error: Core 0 panic'ed (ISR/timer): {type(e).__name__}: {e}")
            if ev.period and not ev.dead:
                ev.t += ev.period
                if ev.t < clk.now() - 0.5:
                    ev.t = clk.now()
                with self.cond:
                    self.seq += 1
                    heapq.heappush(self.heap, (ev.t, self.seq, ev))


# --------------------------------------------------------------------------------------------------
# pins
# --------------------------------------------------------------------------------------------------
_L0, _L1 = ("L", 0), ("L", 1)
_FIRST = operator.itemgetter(0)


class Pin:
    def __init__(self, n):
        self.n = n
        self.ext = None
        self.warned = False
        self.reset()

    def reset(self):
        self.mode, self.level, self.ledc, self.irq, self.owner = INPUT, 0, None, None, None
        self.times = [0.0]
        self.states = [("E", self.ext) if self.ext is not None else ("Z", 0.0)]

    def push(self, t, state):
        if t < self.times[-1]:
            t = self.times[-1]
        self.times.append(t)
        self.states.append(state)
        if len(self.times) > MAX_HISTORY:
            del self.times[:MAX_HISTORY // 3]
            del self.states[:MAX_HISTORY // 3]

    def state_at(self, t):
        return self.states[min(max(bisect.bisect_right(self.times, t) - 1, 0), len(self.states) - 1)]

    def voltage_at(self, t):
        s = self.state_at(t)
        k = s[0]
        if k == "L":
            return VDD if s[1] else 0.0
        if k == "P":
            return VDD if s[1] > 0 and ((t - s[3]) * s[1]) % 1.0 < s[2] else 0.0
        return s[1]


# --------------------------------------------------------------------------------------------------
# peripherals
# --------------------------------------------------------------------------------------------------
class I2CDevice:
    def __init__(self):
        self.regs = bytearray(256)
        self.ptr = 0

    def write(self, data):
        if data:
            self.ptr = data[0]
            for b in data[1:]:
                self.regs[self.ptr & 255] = b
                self.ptr += 1

    def read(self, n):
        out = []
        for _ in range(n):
            out.append(self.regs[self.ptr & 255])
            self.ptr += 1
        return out


class TwoWire:
    def __init__(self, esp, num=0):
        self.esp, self.num = esp, num
        self.addr, self.tx, self.rx = 0, [], []
        self.sda, self.scl = (21, 22) if num == 0 else (-1, -1)

    def begin(self, sda=-1, scl=-1, freq=100000):
        if isinstance(sda, int) and sda >= 0 and scl >= 0:
            self.sda, self.scl = sda, scl
        if self.sda < 0:
            raise EspError("Wire1.begin() needs SDA and SCL pins")
        for n in (self.sda, self.scl):
            p = self.esp.pin(n, out=True)
            p.owner = f"I2C{self.num}"
            p.mode = OUTPUT_OPEN_DRAIN
            p.push(self.esp.now(), ("A", VDD))  # idle high through the external pull-ups
        return True

    def setClock(self, f):
        pass

    def setPins(self, sda, scl):
        self.sda, self.scl = sda, scl
        return True

    def beginTransmission(self, addr):
        self.addr, self.tx = addr, []

    def write(self, data, n=None):
        if isinstance(data, (list, tuple, bytes, bytearray)):
            self.tx.extend(int(x) & 255 for x in data)
            return len(data)
        if isinstance(data, str):
            self.tx.extend(ord(c) & 255 for c in data)
            return len(data)
        self.tx.append(int(data) & 255)
        return 1

    def endTransmission(self, stop=True):
        self.esp.check()
        dev = self.esp.i2c.get(self.addr)
        if dev is None:
            return 2  # NACK on address
        dev.write(self.tx)
        return 0

    def requestFrom(self, addr, n=1, stop=True):
        dev = self.esp.i2c.get(addr)
        self.rx = dev.read(int(n)) if dev else []
        return len(self.rx)

    def available(self):
        return len(self.rx)

    def read(self):
        return self.rx.pop(0) if self.rx else -1

    def peek(self):
        return self.rx[0] if self.rx else -1

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        return lambda *a, **kw: 0


class SPIClass:
    def __init__(self, esp, bus=0):
        self.esp = esp

    def begin(self, sck=-1, miso=-1, mosi=-1, ss=-1):
        for n in (sck, mosi):
            if n >= 0:
                p = self.esp.pin(n, out=True)
                p.mode = OUTPUT
                p.push(self.esp.now(), ("L", 0))

    def transfer(self, data=0, *a):
        self.esp.check()
        if isinstance(data, list):
            return [0] * len(data)
        return 0

    transfer16 = transfer
    transfer32 = transfer

    def transferBytes(self, out, inp, n):
        return None

    def write(self, *a):
        return None

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        return lambda *a, **kw: 0


class HardwareSerial:
    TX_DEFAULT = {0: 1, 1: 10, 2: 17}

    def __init__(self, esp, num):
        self.esp, self.num = esp, num
        self.rx, self.rx_lock = [], threading.Lock()
        self.baud, self.tx_pin, self.tx_t, self.up = 115200, None, 0.0, False
        self.timeout = 1.0
        self.buf = ""

    def begin(self, baud=115200, config=0, rx=-1, tx=-1, *a):
        self.baud, self.up = baud, True
        t = tx if tx >= 0 else self.TX_DEFAULT[self.num]
        if t in FLASH_PINS:
            self.esp.log(f"[warn] Serial{self.num} default TX GPIO{t} is a flash pin - pass explicit pins")
            return
        p = self.esp.pin(t, out=True)
        p.mode, p.owner, self.tx_pin = OUTPUT, f"UART{self.num}", p
        p.level = 1
        p.push(self.esp.now(), ("L", 1))

    def end(self):
        self.up = False

    def _emit(self, s):
        if self.num:
            s = s.replace("\n", f"\n[UART{self.num}] ")
        self.esp.out(s)
        if self.tx_pin is not None and self.up:
            self._wave(s)

    def _wave(self, s):
        e = self.esp
        bit = 1.0 / self.baud
        for ch in s.encode("latin-1", "replace"):
            now = e.now()
            if self.tx_t - now > 0.05:  # TX FIFO full: write blocks like the real driver
                time.sleep(min((self.tx_t - now - 0.05) / e.clock.scale, 0.5))
            t = max(now, self.tx_t)
            level = 1
            bits = [0] + [(ch >> i) & 1 for i in range(8)] + [1]
            for b in bits:
                if b != level:
                    self.tx_pin.push(t, ("L", b))
                    level = b
                t += bit
            self.tx_t = t

    def print(self, x="", fmt=None):
        if isinstance(x, bool):
            s = "1" if x else "0"
        elif isinstance(x, float):
            s = "%.*f" % (2 if fmt is None else int(fmt), x)
        elif isinstance(x, int) and fmt in (2, 8, 16):
            s = _sbase(x, fmt)
        else:
            s = _s(x)
        self._emit(s)
        return len(s)

    def println(self, x="", fmt=None):
        n = self.print(x, fmt)
        self._emit("\n")
        return n + 2

    def printf(self, fmt, *a):
        s = _cfmt(fmt, *a)
        self._emit(s)
        return len(s)

    def write(self, x, n=None):
        if isinstance(x, (list, bytes, bytearray)):
            self._emit("".join(chr(b) for b in x))
            return len(x)
        self._emit(chr(x & 255) if isinstance(x, int) else str(x))
        return 1

    def flush(self):
        pass

    def availableForWrite(self):
        return 128

    def available(self):
        return len(self.rx)

    def feed(self, text):
        with self.rx_lock:
            self.rx.extend(ord(c) for c in text)

    def read(self):
        with self.rx_lock:
            return self.rx.pop(0) if self.rx else -1

    def peek(self):
        return self.rx[0] if self.rx else -1

    def setTimeout(self, ms):
        self.timeout = ms / 1000

    def _wait_for(self, pred):
        end = self.esp.now() + self.timeout
        while not pred() and self.esp.now() < end:
            self.esp.delay(1)

    def readStringUntil(self, term):
        self._wait_for(lambda: term in self.rx)
        out = []
        while self.rx:
            c = self.read()
            if c == term:
                break
            out.append(chr(c))
        return "".join(out)

    def readString(self):
        self._wait_for(lambda: False)
        out = "".join(chr(c) for c in self.rx)
        self.rx.clear()
        return out

    def parseInt(self):
        self._wait_for(lambda: bool(self.rx))
        return _stoi(self.readStringUntil(10))

    def parseFloat(self):
        self._wait_for(lambda: bool(self.rx))
        return _stof(self.readStringUntil(10))

    def __bool__(self):
        return True

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        return lambda *a, **kw: 0


class HwTimer:
    def __init__(self, esp, freq):
        self.esp, self.freq = esp, freq
        self.cb, self.alarm, self.reload, self.ev, self.t0 = None, 0, True, None, esp.now()

    def start(self):
        self.stop()
        if self.alarm > 0 and self.cb:
            period = self.alarm / self.freq
            self.ev = self.esp.engine.schedule(self.esp.now() + period, self.cb, period if self.reload else None)

    def stop(self):
        self.esp.engine.cancel(self.ev)
        self.ev = None


class EspTimer:
    def __init__(self, esp, args):
        self.esp = esp
        self.cb = getattr(args, "callback", None)
        self.arg = getattr(args, "arg", None)
        if isinstance(self.arg, _Dummy):
            self.arg = None
        self.ev = None

    def fire(self):
        if callable(self.cb):
            self.cb(self.arg)

    def start(self, us, periodic):
        self.stop()
        t = us / 1e6
        self.ev = self.esp.engine.schedule(self.esp.now() + t, self.fire, t if periodic else None)

    def stop(self):
        self.esp.engine.cancel(self.ev)
        self.ev = None


# ---- MCPWM -------------------------------------------------------------------------------------
class McTimer:
    def __init__(self, esp, cfg):
        self.esp = esp
        self.res = float(getattr(cfg, "resolution_hz", 1_000_000) or 1_000_000)
        self.period = int(getattr(cfg, "period_ticks", 1000) or 1000)
        self.mode = getattr(cfg, "count_mode", 1)
        if isinstance(self.mode, _Dummy):
            self.mode = 1
        fl = getattr(cfg, "flags", None)
        self.upd_on_empty = bool(getattr(fl, "update_period_on_empty", True)) if fl is not None else True
        self.pending = None
        self.opers, self.cb_empty, self.cb_full, self.ud = [], None, None, None
        self.ev, self.running, self.enabled = None, False, False

    def start(self):
        if not self.running:
            self.running = True
            self.ev = self.esp.engine.schedule(self.esp.now(), self.cycle, 1e-3)

    def stop(self):
        self.running = False
        self.esp.engine.cancel(self.ev)

    def cycle(self):
        e = self.esp
        t0 = e.clock.now() if self.ev is None else self.ev.t
        if self.pending is not None:
            self.period, self.pending = self.pending, None
        period = max(int(self.period), 2)
        tick = 1.0 / self.res
        for op in self.opers:
            for c in op.cmps:
                if c.pend is not None and c.on_tez:
                    c.value, c.pend = c.pend, None
        if self.cb_empty:
            self.cb_empty(self, None, self.ud)
        updown = self.mode == 3
        for op in self.opers:
            for c in op.cmps:
                if c.pend is not None and not c.on_tez:
                    c.value, c.pend = c.pend, None
            for g in op.gens:
                g.run_cycle(t0, tick, period, updown)
        if self.cb_full:
            self.cb_full(self, None, self.ud)
        # the engine re-queues this periodic event itself (no new event object / lock per cycle)
        self.ev.period = period * tick


class McOper:
    def __init__(self, timer=None):
        self.timer, self.cmps, self.gens = timer, [], []


class McCmp:
    def __init__(self, oper, cfg):
        fl = getattr(cfg, "flags", None)
        self.on_tez = bool(getattr(fl, "update_cmp_on_tez", True)) if fl is not None else True
        self.value, self.pend = 0, None


class McGen:
    def __init__(self, esp, oper, pin):
        self.esp, self.oper, self.pin = esp, oper, pin
        self.timer_actions, self.cmp_actions, self.level, self.force = [], [], 0, None

    def run_cycle(self, t0, tick, period, updown):
        if self.force is not None:
            return
        peak = period // 2 if updown else period
        ev = []
        for d, event, act in self.timer_actions:
            if event == 0:
                ev.append((0, act, d))
            else:
                ev.append((peak, act, d))
        for d, cmp_, act in self.cmp_actions:
            v = int(cmp_.value)
            if updown:
                if d == 0 and 0 <= v <= peak:
                    ev.append((v, act, d))
                if d == 1 and 0 <= v <= peak:
                    ev.append((period - v, act, d))
            elif d == 0 and 0 <= v < period:
                ev.append((v, act, d))
        if len(ev) > 1:
            ev.sort(key=_FIRST)
        level = self.level
        pin = self.pin
        times, states = pin.times, pin.states
        for k, act, _ in ev:
            new = 1 if act == 2 else 0 if act == 1 else (1 - level) if act == 3 else level
            if new != level:
                level = new
                t = t0 + k * tick
                last = times[-1]
                times.append(t if t >= last else last)
                states.append(_L1 if new else _L0)
        if level != self.level:
            self.level = pin.level = level
            if len(times) > MAX_HISTORY:
                del times[:MAX_HISTORY // 3]
                del states[:MAX_HISTORY // 3]


class FreeRtosHandle:
    def __init__(self, name):
        self.name = name


class Sema:
    def __init__(self, maxc=1, init=0, mutex=False):
        self.cv, self.count, self.max = threading.Condition(), init, maxc


# --------------------------------------------------------------------------------------------------
# the chip
# --------------------------------------------------------------------------------------------------
API_EXCLUDE = {"build_namespace", "reset_state", "pin", "check", "now", "drive", "run", "halt", "log", "out",
               "autoscale", "feed_serial", "set_scale", "measure"}


class ESP32:
    def __init__(self, out):
        self.out_fn = out
        self.i2c = {0x3C: I2CDevice()}
        self.clock = Clock()
        self.auto = True
        self.calm = 0
        self.cool_until = 0.0
        self.pins =  {n: Pin(n) for n in VALID_GPIO}
        self.stop = False
        self.reset_state()

    def reset_state(self):
        self.clock = Clock()
        self.stop = False
        self.channels, self.ledc_v3 = {}, False
        self.adc_atten, self.adc_bits = {1: 3, 2: 3}, 12
        self.threads, self.stubbed = [], set()
        self.engine = Engine(self)
        self.serials = [HardwareSerial(self, i) for i in range(3)]
        self.wires = [TwoWire(self, 0), TwoWire(self, 1)]
        self.spis = [SPIClass(self, 0), SPIClass(self, 1)]
        self.rand = random.Random(1234)
        for p in self.pins.values():
            p.reset()
            p.owner = None

    # ---- output ----------------------------------------------------------------------------------
    def out(self, s):
        self.out_fn(s)

    def log(self, s):
        self.out_fn(s + "\n")

    def now(self):
        return self.clock.now()

    def check(self):
        if self.stop:
            raise StopEmulation()

    def set_scale(self, s):
        self.clock.set_scale(max(1e-4, min(1.0, s)))

    def autoscale(self):
        if not self.auto or self.stop or self.clock.frozen is not None:
            return
        lag = self.engine.lag
        now = time.perf_counter()
        if now < self.cool_until:  # let the backlog drain after a reduction before judging again
            return
        if lag > 0.003:
            self.set_scale(self.clock.scale * 0.75)
            self.engine.lag = 0.0
            self.calm = 0
            self.cool_until = now + 1.0
        elif lag < 0.001:
            self.calm += 1
            if self.calm > 15 and self.clock.scale < 1.0:
                self.set_scale(self.clock.scale * 1.1)
                self.calm = 0
                self.cool_until = now + 1.0

    # ---- pin helpers -------------------------------------------------------------------------------
    def pin(self, n, out=False):
        self.check()
        n = int(n)
        if n not in self.pins:
            raise EspError(f"GPIO{n} does not exist on the ESP32")
        if n in FLASH_PINS:
            raise EspError(f"GPIO{n} is connected to the SPI flash - using it crashes the chip")
        if out and n in INPUT_ONLY:
            raise EspError(f"GPIO{n} is input-only")
        return self.pins[n]

    def _in_voltage(self, p):
        if p.ext is not None:
            return p.ext
        return VDD if p.mode == INPUT_PULLUP else 0.0

    def _digital(self, p):
        if p.mode & 0x02:
            if p.ledc is not None or p.owner:
                return 1 if p.voltage_at(self.now()) > VDD / 2 else 0
            return p.level
        v = self._in_voltage(p)
        if v >= 0.75 * VDD:
            return 1
        if v <= 0.25 * VDD:
            return 0
        return p.level

    def _notify(self, p, old, new):
        if p.irq and old != new:
            fn, mode = p.irq
            if mode == CHANGE or (mode == RISING and new) or (mode == FALLING and not new):
                self.engine.call(fn)

    # ---- GPIO -----------------------------------------------------------------------------------------
    def pinMode(self, n, mode):
        p = self.pin(n)
        if mode & 0x02 and n in INPUT_ONLY:
            raise EspError(f"GPIO{n} is input-only")
        if mode in (INPUT_PULLUP, INPUT_PULLDOWN) and n in INPUT_ONLY:
            self.log(f"[warn] GPIO{n} has no internal pull resistors")
        if n in STRAPPING and not p.warned:
            p.warned = True
            self.log(f"[warn] GPIO{n} is a strapping pin ({STRAPPING[n]})")
        old = self._digital(p)
        p.mode, p.ledc = mode, None
        if mode & 0x02:
            p.push(self.now(), ("L", p.level))
        else:
            p.push(self.now(), ("E", p.ext) if p.ext is not None else ("Z", VDD if mode == INPUT_PULLUP else 0.0))
        self._notify(p, old, self._digital(p))

    def digitalWrite(self, n, v):
        p = self.pin(n, out=True)
        old = p.level
        p.level = 1 if v else 0
        if not p.mode & 0x02:
            return
        p.ledc = None
        p.push(self.now(), ("L", p.level))
        self._notify(p, old, p.level)

    def digitalRead(self, n):
        return self._digital(self.pin(n))

    def gpio_set_direction(self, n, mode):
        self.pinMode(n, {0: INPUT, 1: INPUT, 2: OUTPUT, 3: OUTPUT}.get(mode, INPUT))
        return 0

    def gpio_set_level(self, n, v):
        self.digitalWrite(n, v)
        return 0

    def gpio_get_level(self, n):
        return self.digitalRead(n)

    def gpio_reset_pin(self, n):
        self.pinMode(n, INPUT)
        return 0

    def gpio_set_pull_mode(self, n, mode):
        if mode == 0:
            self.pinMode(n, INPUT_PULLUP)
        elif mode == 1:
            self.pinMode(n, INPUT_PULLDOWN)
        return 0

    def attachInterrupt(self, n, fn, mode):
        self.pin(n).irq = (fn, mode)

    def detachInterrupt(self, n):
        self.pin(n).irq = None

    def digitalPinToInterrupt(self, n):
        return n

    def shiftOut(self, data_pin, clock_pin, order, val):
        for i in range(8):
            bit = (val >> i) & 1 if order == 0 else (val >> (7 - i)) & 1
            self.digitalWrite(data_pin, bit)
            self.digitalWrite(clock_pin, 1)
            self.digitalWrite(clock_pin, 0)

    def pulseIn(self, n, state, timeout=1000000):
        return 0

    # ---- ADC / DAC / touch -------------------------------------------------------------------------------
    def analogReadResolution(self, bits):
        if not 9 <= bits <= 12:
            raise EspError("ADC resolution must be 9..12 bit")
        self.adc_bits = bits

    def analogSetAttenuation(self, att):
        self.adc_atten = {1: att, 2: att}

    def analogSetPinAttenuation(self, n, att):
        self.adc_atten[1 if n in ADC1 else 2] = att

    def analogSetWidth(self, bits):
        self.adc_bits = bits

    def _adc_volts(self, n):
        p = self.pin(n)
        if n not in ADC1 and n not in ADC2:
            raise EspError(f"GPIO{n} has no ADC")
        v = p.voltage_at(self.now()) if p.mode & 0x02 else self._in_voltage(p)
        return min(v, VDD) + self.rand.gauss(0, 0.002)

    def analogRead(self, n):
        fs = ATTEN[self.adc_atten[1 if n in ADC1 else 2]]
        raw = max(0, min(4095, int(self._adc_volts(n) / fs * 4095)))
        return raw >> (12 - self.adc_bits)

    def analogReadMilliVolts(self, n):
        fs = ATTEN[self.adc_atten[1 if n in ADC1 else 2]]
        return int(max(0.0, min(self._adc_volts(n), fs)) * 1000)

    def adc1_get_raw(self, ch):
        pin = next((p for p, c in ADC1.items() if c == ch), None)
        return self.analogRead(pin)

    def adc1_config_width(self, w):
        return 0

    def adc1_config_channel_atten(self, ch, att):
        self.adc_atten[1] = att
        return 0

    def dacWrite(self, n, val):
        if n not in DAC_PINS:
            raise EspError("DAC only on GPIO25 and GPIO26")
        p = self.pin(n, out=True)
        p.ledc, p.mode = None, OUTPUT
        p.push(self.now(), ("A", max(0, min(255, int(val))) / 255 * VDD))

    def touchRead(self, n):
        if n not in TOUCH:
            raise EspError(f"GPIO{n} has no touch sensor")
        p = self.pin(n)
        return 70 if p.ext is None else int(5 + 65 * (1 - p.ext / VDD))

    # ---- LEDC -----------------------------------------------------------------------------------------------
    def ledcSetup(self, ch, freq, res):
        if not 0 <= ch < 16:
            raise EspError("LEDC has 16 channels (0..15)")
        if not 1 <= res <= 20:
            raise EspError("LEDC resolution must be 1..20 bit")
        if freq * (1 << res) > 80_000_000:
            raise EspError(f"LEDC: freq*2^res = {freq * (1 << res):.0f} exceeds the 80 MHz timer clock "
                           f"(max freq at {res} bit = {80_000_000 // (1 << res)} Hz)")
        old = self.channels.get(ch, {})
        self.channels[ch] = dict(freq=freq, res=res, duty=0, pin=old.get("pin"))
        self._ledc_update(ch)
        return freq

    def ledcAttachPin(self, n, ch):
        p = self.pin(n, out=True)
        if ch not in self.channels:
            raise EspError(f"LEDC channel {ch} not configured (call ledcSetup first)")
        p.mode, p.ledc = OUTPUT, ch
        self.channels[ch]["pin"] = n
        self._ledc_update(ch)

    def ledcAttach(self, n, freq, res):
        self.ledc_v3 = True
        ch = next((c for c in range(16) if c not in self.channels or self.channels[c]["pin"] is None), None)
        if ch is None:
            raise EspError("no free LEDC channel")
        self.ledcSetup(ch, freq, res)
        self.ledcAttachPin(n, ch)
        return True

    def ledcAttachChannel(self, n, freq, res, ch):
        self.ledc_v3 = True
        self.ledcSetup(ch, freq, res)
        self.ledcAttachPin(n, ch)
        return True

    def _chan(self, x):
        if self.ledc_v3:
            p = self.pins.get(x)
            if p is not None and p.ledc is not None:
                return p.ledc
        if x in self.channels:
            return x
        ch = next((c for c, d in self.channels.items() if d["pin"] == x), None)
        if ch is None:
            raise EspError(f"LEDC channel/pin {x} not configured")
        return ch

    def ledcWrite(self, x, duty):
        self.check()
        ch = self._chan(x)
        c = self.channels[ch]
        c["duty"] = max(0, min(1 << c["res"], int(duty)))
        self._ledc_update(ch)
        return True

    def ledcRead(self, x):
        return self.channels[self._chan(x)]["duty"]

    def ledcChangeFrequency(self, x, freq, res):
        ch = self._chan(x)
        pin = self.channels[ch]["pin"]
        self.ledcSetup(ch, freq, res)
        self.channels[ch]["pin"] = pin
        return freq

    def ledcWriteTone(self, x, freq):
        ch = self._chan(x)
        c = self.channels[ch]
        c["freq"], c["duty"] = freq, 1 << (c["res"] - 1) if freq else 0
        self._ledc_update(ch)
        return freq

    def ledcDetachPin(self, n):
        p = self.pin(n)
        if p.ledc is not None:
            self.channels[p.ledc]["pin"] = None
        p.ledc = None
        p.push(self.now(), ("L", 0))

    ledcDetach = ledcDetachPin

    def analogWrite(self, n, val):
        p = self.pin(n, out=True)
        if p.ledc is None:
            self.ledcAttach(n, 1000, 8)
        self.ledcWrite(n, val)

    def tone(self, n, freq, dur=0):
        p = self.pin(n, out=True)
        if p.ledc is None:
            self.ledcAttach(n, max(int(freq), 1), 8)
        self.ledcChangeFrequency(n, max(int(freq), 1), 8)
        self.ledcWrite(n, 128)
        if dur:
            self.engine.schedule(self.now() + dur / 1000, lambda: self.noTone(n))

    def noTone(self, n):
        self.ledcWrite(n, 0)

    def _ledc_update(self, ch):
        c = self.channels[ch]
        if c["pin"] is not None:
            t = self.now()
            self.pins[c["pin"]].push(t, ("P", c["freq"], c["duty"] / (1 << c["res"]), t))

    # ---- time ---------------------------------------------------------------------------------------------------
    def millis(self):
        return int(self.now() * 1000) & 0xFFFFFFFF

    def micros(self):
        return int(self.now() * 1e6) & 0xFFFFFFFF

    def esp_timer_get_time(self):
        return int(self.now() * 1e6)

    def _wait(self, seconds):
        clk = self.clock
        end = clk.now() + seconds
        while True:
            self.check()
            rem = end - clk.now()
            if rem <= 0:
                return
            time.sleep(min(rem / clk.scale, 0.01) if rem / clk.scale > 0.0002 else 0)

    def delay(self, ms):
        self._wait(ms / 1000)

    def delayMicroseconds(self, us):
        self._wait(us / 1e6)

    def yield_(self):
        self.check()
        time.sleep(0)

    def esp_random(self):
        return self.rand.getrandbits(32)

    def random(self, a, b=None):
        if b is None:
            a, b = 0, a
        return self.rand.randrange(int(a), int(b)) if b > a else int(a)

    def randomSeed(self, s):
        self.rand.seed(s)

    # ---- hardware timers / esp_timer --------------------------------------------------------------------------
    def timerBegin(self, a=1000000, divider=None, countUp=True):
        freq = a if divider is None else 80_000_000 / max(int(divider), 2)
        return HwTimer(self, freq)

    def timerAttachInterrupt(self, t, fn, edge=True):
        t.cb = fn

    def timerAlarmWrite(self, t, ticks, reload):
        t.alarm, t.reload = ticks, bool(reload)

    def timerAlarmEnable(self, t):
        t.start()

    def timerAlarm(self, t, ticks, reload, count):
        t.alarm, t.reload = ticks, bool(reload)
        t.start()

    def timerAlarmDisable(self, t):
        t.stop()

    def timerEnd(self, t):
        t.stop()

    timerStop = timerAlarmDisable

    def timerRead(self, t):
        return int((self.now() - t.t0) * t.freq)

    def timerReadMicros(self, t):
        return int((self.now() - t.t0) * 1e6)

    def esp_timer_create(self, args, cell):
        h = EspTimer(self, args)
        if isinstance(cell, _Cell):
            cell.v = h
        return 0

    def esp_timer_start_periodic(self, h, us):
        h.start(us, True)
        return 0

    def esp_timer_start_once(self, h, us):
        h.start(us, False)
        return 0

    def esp_timer_restart(self, h, us):
        h.start(us, True)
        return 0

    def esp_timer_stop(self, h):
        h.stop()
        return 0

    def esp_timer_delete(self, h):
        h.stop()
        return 0

    # ---- MCPWM ----------------------------------------------------------------------------------------------------
    def mcpwm_new_timer(self, cfg, cell):
        cell.v = McTimer(self, cfg)
        return 0

    def mcpwm_new_operator(self, cfg, cell):
        cell.v = McOper()
        return 0

    def mcpwm_operator_connect_timer(self, op, timer):
        op.timer = timer
        timer.opers.append(op)
        return 0

    def mcpwm_new_comparator(self, op, cfg, cell):
        c = McCmp(op, cfg)
        op.cmps.append(c)
        cell.v = c
        return 0

    def mcpwm_new_generator(self, op, cfg, cell):
        n = getattr(cfg, "gen_gpio_num", -1)
        p = self.pin(n, out=True)
        p.mode, p.owner = OUTPUT, "MCPWM"
        p.level = 0
        p.push(self.now(), ("L", 0))
        g = McGen(self, op, p)
        op.gens.append(g)
        cell.v = g
        return 0

    def MCPWM_GEN_TIMER_EVENT_ACTION(self, d, event, action):
        return ("t", d, event, action)

    def MCPWM_GEN_COMPARE_EVENT_ACTION(self, d, cmp_, action):
        return ("c", d, cmp_, action)

    def mcpwm_generator_set_action_on_timer_event(self, g, ev):
        g.timer_actions.append((ev[1], ev[2], ev[3]))
        return 0

    def mcpwm_generator_set_action_on_compare_event(self, g, ev):
        g.cmp_actions.append((ev[1], ev[2], ev[3]))
        return 0

    def mcpwm_generator_set_force_level(self, g, level, hold):
        g.force = level if level >= 0 else None
        if level >= 0:
            g.level = level
            g.pin.push(self.now(), ("L", level))
        return 0

    def mcpwm_comparator_set_compare_value(self, c, v):
        c.pend = int(v)
        return 0

    def mcpwm_timer_set_period(self, t, ticks):
        t.pending = int(ticks)
        return 0

    def mcpwm_timer_register_event_callbacks(self, t, cbs, ud):
        t.cb_empty = getattr(cbs, "on_empty", None)
        t.cb_full = getattr(cbs, "on_full", None)
        t.cb_empty = t.cb_empty if callable(t.cb_empty) else None
        t.cb_full = t.cb_full if callable(t.cb_full) else None
        t.ud = ud
        return 0

    def mcpwm_timer_enable(self, t):
        t.enabled = True
        return 0

    def mcpwm_timer_disable(self, t):
        t.enabled = False
        return 0

    def mcpwm_timer_start_stop(self, t, cmd):
        if cmd in (2, 3, 4):
            t.start()
        else:
            t.stop()
        return 0

    # ---- FreeRTOS-ish ------------------------------------------------------------------------------------------------
    def xTaskCreatePinnedToCore(self, fn, name, stack, arg, prio, handle, core):
        if core not in (0, 1, 0x7FFFFFFF):
            raise EspError("ESP32 has cores 0 and 1")
        if stack < 512:
            raise EspError(f"task '{name}': stack size {stack} is too small")
        h = FreeRtosHandle(name)
        if isinstance(handle, _Cell):
            handle.v = h
        self._spawn(lambda: fn(arg), f"{name}@core{core if core < 2 else 1}")
        return 1

    def xTaskCreate(self, fn, name, stack, arg, prio, handle=None):
        return self.xTaskCreatePinnedToCore(fn, name, stack, arg, prio, handle, 0x7FFFFFFF)

    def vTaskDelete(self, h=None):
        if h is None or isinstance(h, FreeRtosHandle) and False:
            raise StopEmulation() if threading.current_thread().name.startswith("loopTask") else _TaskEnd()

    def vTaskDelay(self, ticks):
        self.delay(ticks)

    def vTaskDelayUntil(self, last, ticks):
        self.delay(ticks)

    def taskYIELD(self):
        self.yield_()

    def pdMS_TO_TICKS(self, ms):
        return int(ms)

    def xTaskGetTickCount(self):
        return self.millis()

    def xQueueCreate(self, n, size=0):
        return queue.Queue(int(n))

    def xQueueSend(self, q, item, timeout=0):
        val = item.v if isinstance(item, _Cell) else item
        try:
            if timeout:
                q.put(val, timeout=min(timeout, 1 << 20) / 1000)
            else:
                q.put_nowait(val)
            return 1
        except queue.Full:
            return 0

    xQueueSendToBack = xQueueSend

    def xQueueReceive(self, q, buf=None, timeout=0):
        c_style = isinstance(buf, (_Cell, list)) or buf is None and timeout == 0 and False
        if not c_style and buf is not None:
            timeout, buf = buf, None
        end = self.now() + min(timeout, 1 << 20) / 1000
        while True:
            self.check()
            try:
                val = q.get(timeout=0.005)
                if isinstance(buf, _Cell):
                    buf.v = val
                    return 1
                return val if buf is None else 1
            except queue.Empty:
                if self.now() >= end:
                    return 0 if c_style else None

    def uxQueueMessagesWaiting(self, q):
        return q.qsize()

    def xSemaphoreCreateMutex(self):
        return Sema(1, 1)

    def xSemaphoreCreateBinary(self):
        return Sema(1, 0)

    def xSemaphoreCreateCounting(self, maxc, init):
        return Sema(maxc, init)

    def xSemaphoreTake(self, s, timeout=0):
        end = self.now() + min(timeout, 1 << 20) / 1000
        while True:
            self.check()
            with s.cv:
                if s.count > 0:
                    s.count -= 1
                    return 1
            if self.now() >= end:
                return 0
            time.sleep(0.001)

    def xSemaphoreGive(self, s):
        with s.cv:
            s.count = min(s.count + 1, s.max)
        return 1

    xSemaphoreGiveFromISR = xSemaphoreGive

    def xSemaphoreTakeFromISR(self, s, *a):
        return self.xSemaphoreTake(s, 0)

    def ESP_ERROR_CHECK(self, err):
        if err:
            raise EspError(f"ESP_ERROR_CHECK failed: esp_err_t 0x{int(err):x}")

    def printf(self, fmt, *a):
        self.out(_cfmt(fmt, *a))
        return 0

    def ESP_LOGI(self, tag, fmt, *a):
        self.log(f"I ({self.millis()}) {tag}: {_cfmt(fmt, *a)}")

    def ESP_LOGW(self, tag, fmt, *a):
        self.log(f"W ({self.millis()}) {tag}: {_cfmt(fmt, *a)}")

    def ESP_LOGE(self, tag, fmt, *a):
        self.log(f"E ({self.millis()}) {tag}: {_cfmt(fmt, *a)}")

    def noInterrupts(self):
        pass

    def interrupts(self):
        pass

    def portENTER_CRITICAL(self, m=None):
        pass

    portEXIT_CRITICAL = portENTER_CRITICAL_ISR = portEXIT_CRITICAL_ISR = portENTER_CRITICAL

    def esp_get_free_heap_size(self):
        return 280000

    def malloc(self, n):
        return bytearray(int(n))

    calloc = lambda self, n, s=1: bytearray(int(n) * int(s))  # noqa: E731

    def free(self, p):
        pass

    def memset(self, arr, val, n):
        for i in range(min(int(n), len(arr))):
            arr[i] = val
        return arr

    def memcpy(self, dst, src, n):
        for i in range(int(n)):
            dst[i] = src[i]
        return dst

    memmove = memcpy

    def strcmp(self, a, b):
        return _cmp(a, b)

    def atoi(self, s):
        return _stoi(s)

    def atof(self, s):
        return _stof(s)

    def isDigit(self, c):
        return 48 <= c <= 57

    def isAlpha(self, c):
        return chr(c).isalpha()

    def isSpace(self, c):
        return chr(c).isspace()

    def isUpperCase(self, c):
        return chr(c).isupper()

    def isLowerCase(self, c):
        return chr(c).islower()

    def isAlphaNumeric(self, c):
        return chr(c).isalnum()

    def isnan(self, x):
        return x != x

    # ---- external world (GUI) -------------------------------------------------------------------------------------------
    def drive(self, n, volts):
        p = self.pins[n]
        old = self._digital(p)
        p.ext = None if volts is None else max(0.0, min(VDD, volts))
        if not p.mode & 0x02:
            p.push(self.now(), ("E", p.ext) if p.ext is not None else ("Z", VDD if p.mode == INPUT_PULLUP else 0.0))
            self._notify(p, old, self._digital(p))

    def feed_serial(self, num, text):
        self.serials[num].feed(text)

    # ---- measurements ----------------------------------------------------------------------------------------------------
    def measure(self, p, t1):
        """Frequency / duty of a digital waveform over the last transitions."""
        st = p.states
        tm = p.times
        i1 = bisect.bisect_right(tm, t1)
        i0 = max(i1 - 3000, 0)
        if i1 - i0 < 3:
            return None
        if st[i1 - 1][0] == "P":
            s = st[i1 - 1]
            return s[1], s[2]
        rises, highs, last_edge, hi = [], 0.0, None, None
        for k in range(i0, i1):
            s = st[k]
            if s[0] != "L":
                return None
            if s[1] and (hi is None or not hi):
                rises.append(tm[k])
            if hi:
                highs += tm[k] - last_edge
            last_edge, hi = tm[k], s[1]
        if len(rises) < 2:
            return None
        per = (rises[-1] - rises[0]) / (len(rises) - 1)
        span = max(last_edge - tm[i0], 1e-12)
        return (1 / per if per > 0 else 0.0), min(highs / span, 1.0)

    # ---- sketch execution ----------------------------------------------------------------------------------------------------
    def _spawn(self, body, name):
        def run():
            try:
                body()
            except (StopEmulation, _TaskEnd):
                pass
            except EspError as e:
                self.log(f"[{name}] ESP error: {e}")
            except Exception as e:
                tb = e.__traceback__
                while tb.tb_next:
                    tb = tb.tb_next
                self.log(f"Guru Meditation Error: Core panic'ed in {name}: {type(e).__name__}: {e} "
                         f"(line {tb.tb_lineno} of generated code, in {tb.tb_frame.f_code.co_name})")
        th =  threading.Thread(target=run, daemon=True, name=name)
        self.threads.append(th)
        th.start()

    def build_namespace(self):
        ns = {k: getattr(self, k) for k in dir(self)
              if not k.startswith("_") and callable(getattr(self, k)) and k not in API_EXCLUDE
              and k not in ("engine", "clock", "pins", "serials", "wires", "spis", "i2c", "threads")}
        ns["yield"] = self.yield_
        ns.update(
            INPUT=INPUT, OUTPUT=OUTPUT, INPUT_PULLUP=INPUT_PULLUP, INPUT_PULLDOWN=INPUT_PULLDOWN,
            OUTPUT_OPEN_DRAIN=OUTPUT_OPEN_DRAIN, LOW=LOW, HIGH=HIGH, RISING=RISING, FALLING=FALLING, CHANGE=CHANGE,
            Serial=self.serials[0], Serial1=self.serials[1], Serial2=self.serials[2], Wire=self.wires[0],
            Wire1=self.wires[1], SPI=self.spis[0], HSPI=self.spis[1], VSPI=self.spis[0],
            TwoWire=lambda n=0: self.wires[1 if n else 0], HardwareSerial=lambda n=1: self.serials[min(int(n), 2)],
            SPIClass=lambda n=0: self.spis[1 if n else 0],
            ADC_0db=0, ADC_2_5db=1, ADC_6db=2, ADC_11db=3, ADC_ATTEN_DB_0=0, ADC_ATTEN_DB_2_5=1,
            ADC_ATTEN_DB_6=2, ADC_ATTEN_DB_11=3, ADC_ATTEN_DB_12=3, ADC1_CHANNEL_0=0, ADC_WIDTH_BIT_12=3,
            PI=math.pi, M_PI=math.pi, HALF_PI=math.pi / 2, TWO_PI=2 * math.pi, DEG_TO_RAD=math.pi / 180,
            RAD_TO_DEG=180 / math.pi, M_E=math.e, math=math, time=time, INFINITY=math.inf, NAN=math.nan,
            DEC=10, HEX=16, OCT=8, BIN=2, MSBFIRST=1, LSBFIRST=0, SPI_MODE0=0, SPI_MODE1=1, SPI_MODE2=2,
            SPI_MODE3=3, portMAX_DELAY=1 << 30, pdTRUE=1, pdFALSE=0, pdPASS=1, pdFAIL=0, tskNO_AFFINITY=0x7FFFFFFF,
            ESP_OK=0, ESP_FAIL=-1, ESP_ERR_NO_MEM=0x101, ESP_ERR_INVALID_ARG=0x102, ESP_TIMER_TASK=0,
            ESP_TIMER_ISR=1, GPIO_MODE_INPUT=1, GPIO_MODE_OUTPUT=2, GPIO_MODE_INPUT_OUTPUT=3,
            GPIO_PULLUP_ONLY=0, GPIO_PULLDOWN_ONLY=1, GPIO_FLOATING=3, LED_BUILTIN=2, SDA=21, SCL=22, SS=5,
            MOSI=23, MISO=19, SCK=18, TX=1, RX=3, T0=4, T1=0, T2=2, T3=15, T4=13, T5=12, T6=14, T7=27, T8=33, T9=32,
            DAC1=25, DAC2=26, A0=36, A3=39, A4=32, A5=33, A6=34, A7=35, A10=4, A11=0, A12=2, A13=15, A14=13,
            A15=12, A16=14, A17=27, A18=25, A19=26,
            MCPWM_TIMER_DIRECTION_UP=0, MCPWM_TIMER_DIRECTION_DOWN=1, MCPWM_TIMER_COUNT_MODE_PAUSE=0,
            MCPWM_TIMER_COUNT_MODE_UP=1, MCPWM_TIMER_COUNT_MODE_DOWN=2, MCPWM_TIMER_COUNT_MODE_UP_DOWN=3,
            MCPWM_TIMER_EVENT_EMPTY=0, MCPWM_TIMER_EVENT_FULL=1, MCPWM_GEN_ACTION_KEEP=0, MCPWM_GEN_ACTION_LOW=1,
            MCPWM_GEN_ACTION_HIGH=2, MCPWM_GEN_ACTION_TOGGLE=3, MCPWM_TIMER_STOP_EMPTY=0, MCPWM_TIMER_STOP_FULL=1,
            MCPWM_TIMER_START_NO_STOP=2, MCPWM_TIMER_START_STOP_EMPTY=3, MCPWM_TIMER_START_STOP_FULL=4,
            MCPWM_TIMER_CLK_SRC_DEFAULT=0, MCPWM_TIMER_CLK_SRC_PLL160M=0, MCPWM_CAP_TIMER_CLK_SRC_DEFAULT=0,
            esp_timer_get_time_f=lambda: self.now() * 1e6, abs=abs, min=min, max=max, pow=_pow, sq=lambda x: x * x,
            round=lambda x: int(math.floor(x + 0.5)) if x >= 0 else -int(math.floor(-x + 0.5)),
            constrain=lambda x, a, b: max(a, min(b, x)), radians=math.radians, degrees=math.degrees,
            map=_map, lowByte=lambda x: x & 255, highByte=lambda x: (x >> 8) & 255,
            bit=lambda n: 1 << n, bitRead=lambda v, n: (v >> n) & 1, bitSet=lambda v, n: v | (1 << n),
            bitClear=lambda v, n: v & ~(1 << n), bitWrite=lambda v, n, b: (v | (1 << n)) if b else (v & ~(1 << n)),
            sizeof=lambda x: 4, dtostrf=_dtostrf, ESP=_EspObj(self), temperatureRead=lambda: 42.0,
            getCpuFrequencyMhz=lambda: 240, setCpuFrequencyMhz=lambda f: None, hallRead=lambda: 0,
            _Cell=_Cell, _Dummy=_Dummy, _idiv=_idiv, _imod=_imod, _i8=_i8, _i16=_i16, _i32=_i32, _i64=_i64,
            _s=_s, _sf=_sf, _sbase=_sbase, _stoi=_stoi, _stof=_stof, _charat=_charat, _cmp=_cmp,
            _sremove=_sremove, _ssetchar=_ssetchar, _cp=_cp, _pad=_pad, _seti=_seti, _seta=_seta, _pinc=_pinc,
            _pinca=_pinca, _post=_post, _cfmt=_cfmt, _dtostrf=_dtostrf)
        for name in ("sin", "cos", "tan", "asin", "acos", "atan", "atan2", "sqrt", "exp", "log", "log10", "fabs",
                     "floor", "ceil", "fmod", "sinh", "cosh", "tanh", "trunc", "hypot", "cbrt"):
            f = _safe(getattr(math, name, None) or (lambda x: abs(x) if name == "fabs" else x))
            ns[name] = f
            ns[name + "f"] = f
        ns["fabs"] = ns["fabsf"] = abs
        ns["pow"] = ns["powf"] = _pow
        ns["floor"] = ns["floorf"] = lambda x: float(math.floor(x))
        ns["ceil"] = ns["ceilf"] = lambda x: float(math.ceil(x))
        ns["cbrt"] = lambda x: math.copysign(abs(x) ** (1 / 3), x)
        return ns

    def run(self, source, force=None):
        self.reset_state()
        self.log("ets Jun  8 2016 00:22:57\n\nrst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)\n"
                 "configsip: 0, SPIWP:0xee\nclk_drv:0x00,q_drv:0x00,d_drv:0x00,cs0_drv:0x00,hd_drv:0x00,wp_drv:0x00\n"
                 "mode:DIO, clock div:2\nentry 0x400805f0\n[emulator] 2 cores @ 240 MHz, 3.3 V rail, tick 1 kHz")
        ns = self.build_namespace()
        is_cpp = esp32_cpp.looks_like_cpp(source) if force is None else force == "cpp"
        self.generated = source
        if is_cpp:
            try:
                res = esp32_cpp.transpile(source, known=ns.keys())
            except esp32_cpp.CppError as e:
                self.log(f"[compile error] {e}")
                return False
            self.generated = res.code
            if res.unknown_calls or res.unknown_ids:
                self.log("[translator] not emulated (stubbed): " + ", ".join(res.unknown_calls + res.unknown_ids))
            for w in res.warnings:
                self.log(f"[translator] {w}")
            for name in res.unknown_calls:
                ns.setdefault(name, self._stub(name))
            for name in res.unknown_ids:
                ns.setdefault(name, _Dummy(name) if name[:1].isupper() and not name.isupper() else 0)
            source = res.code
        try:
            code = compile(source, "<sketch>", "exec")
        except SyntaxError as e:
            self.log(f"[compile error] line {e.lineno}: {e.msg}")
            return False
        self.engine.start()

        def main():
            exec(code, ns)
            if "setup" in ns:
                ns["setup"]()
            elif "app_main" in ns:
                ns["app_main"]()
                return
            lp = ns.get("loop")
            while lp:
                lp()
                self.check()
                time.sleep(0)
        self._spawn(main, "loopTask@core1")
        return True

    def _stub(self, name):
        def stub(*a, **k):
            if name not in self.stubbed:
                self.stubbed.add(name)
                self.log(f"[stub] {name}() is not emulated - returns 0")
            return 0
        return stub

    def halt(self):
        self.clock.freeze()
        self.stop = True
        with self.engine.cond:
            self.engine.cond.notify_all()


class _TaskEnd(BaseException):
    pass


class _EspObj:
    def __init__(self, esp):
        self.esp = esp

    def restart(self):
        self.esp.log("[ESP.restart] software reset")
        raise StopEmulation()

    def getFreeHeap(self):
        return 280000

    def getChipModel(self):
        return "ESP32-D0WDQ6"

    def getCpuFreqMHz(self):
        return 240

    def getSketchSize(self):
        return 300000

    def getChipRevision(self):
        return 1

    def getEfuseMac(self):
        return 0x240AC4112233

    def __getattr__(self, k):
        if k.startswith("__"):
            raise AttributeError(k)
        return lambda *a, **kw: 0


def _safe(f):
    def g(*a):
        try:
            return f(*a)
        except (ValueError, OverflowError):
            return math.nan
    return g


def _pow(a, b):
    try:
        return float(a) ** b
    except (ZeroDivisionError, OverflowError):
        return math.inf
    except ValueError:
        return math.nan


def _map(x, a, b, c, d):
    if all(isinstance(v, int) for v in (x, a, b, c, d)):
        return _idiv((x - a) * (d - c), (b - a)) + c if b != a else c
    return (x - a) * (d - c) / (b - a) + c


# --------------------------------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------------------------------
EXAMPLE_CPP = r'''// Any Arduino/ESP32 C++ sketch can be pasted here (or opened with "Open...").
#define LED_PIN   13      // LEDC PWM
#define SQUARE    14      // 2 Hz square wave
#define BTN       32      // button to GND (use "Drive pin" slider: 0 V = pressed)
#define POT       33      // analog input

volatile uint32_t presses = 0;
uint32_t lastPrint = 0;

void IRAM_ATTR onButton() { presses++; }

void setup() {
  Serial.begin(115200);
  pinMode(BTN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN), onButton, FALLING);
  pinMode(SQUARE, OUTPUT);
  ledcAttach(LED_PIN, 1000, 10);              // 1 kHz, 10 bit
  analogSetAttenuation(ADC_11db);
  Wire.begin(21, 22);
  for (uint8_t a = 1; a < 127; a++) {          // I2C scan
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) Serial.printf("I2C device at 0x%02X\n", a);
  }
}

void loop() {
  float s = (sinf(millis() / 500.0f) + 1.0f) / 2.0f;
  ledcWrite(LED_PIN, (int)(s * 1023));
  dacWrite(25, (int)(127 + 127 * sinf(millis() / 200.0f)));
  digitalWrite(SQUARE, (millis() / 250) % 2);
  if (millis() - lastPrint > 1000) {
    lastPrint = millis();
    int raw = analogRead(POT);
    Serial.printf("ADC%d=%d (%.2f V)  presses=%u\n", POT, raw, raw * 3.9f / 4095.0f, presses);
  }
  delay(5);
}
'''

EXAMPLE_PY = '''# Same API in Python
def setup():
    Serial.begin(115200)
    pinMode(14, OUTPUT)

def loop():
    digitalWrite(14, (millis() // 250) % 2)
    delay(5)
'''

COLORS = ["#ffd400", "#00e5ff", "#ff4081", "#76ff03", "#ff9100", "#b388ff", "#ffffff", "#64ffda"]
SPEEDS = ["Auto", "1x", "0.1x", "0.01x", "0.001x", "0.0001x"]


def pin_label(n):
    tags = []
    if n in INPUT_ONLY:
        tags.append("in-only")
    if n in ADC1:
        tags.append(f"ADC1_{ADC1[n]}")
    if n in ADC2:
        tags.append(f"ADC2_{ADC2[n]}")
    if n in DAC_PINS:
        tags.append("DAC")
    if n in TOUCH:
        tags.append(f"T{TOUCH[n]}")
    if n in STRAPPING:
        tags.append("strap")
    if n in FLASH_PINS:
        tags.append("FLASH")
    return f"GPIO{n:<2} {' '.join(tags)}"


def fmt_div(s):
    for unit, k in (("s", 1.0), ("ms", 1e-3), ("us", 1e-6)):
        if s >= k * 0.9995:
            return f"{s / k:.3g} {unit}"
    return f"{s * 1e9:.3g} ns"


def fmt_hz(f):
    return f"{f / 1e6:.3f} MHz" if f >= 1e6 else f"{f / 1e3:.3f} kHz" if f >= 1e3 else f"{f:.2f} Hz"


class App:
    def __init__(self, root):
        self.root = root
        root.title("ESP32 Emulator")
        root.geometry("1400x850")
        self.q = queue.Queue()
        self.esp = ESP32(self.q.put)
        self.frozen = False
        self.frozen_t = 0.0
        self.driven = None
        sys.setswitchinterval(0.002)

        paned = ttk.PanedWindow(root, orient="horizontal")
        paned.pack(fill="both", expand=True)
        left, right = ttk.Frame(paned), ttk.Frame(paned)
        paned.add(left, weight=2)
        paned.add(right, weight=3)

        bar = ttk.Frame(left)
        bar.pack(fill="x")
        for text, cmd in (("Run", self.run), ("Stop", self.stop), ("Open...", self.open), ("Save...", self.save),
                          ("View Python", self.view_py)):
            ttk.Button(bar, text=text, command=cmd).pack(side="left")
        self.lang = ttk.Combobox(bar, width=8, state="readonly", values=["Auto", "C++", "Python"])
        self.lang.set("Auto")
        self.lang.pack(side="left", padx=4)
        self.status = ttk.Label(bar, text="stopped")
        self.status.pack(side="right")
        self.editor = tk.Text(left, font=("Consolas", 10), undo=True, wrap="none")
        self.editor.pack(fill="both", expand=True)
        self.editor.insert("1.0", EXAMPLE_CPP)
        self.console = tk.Text(left, height=12, font=("Consolas", 9), bg="#111", fg="#9f9", state="disabled")
        self.console.pack(fill="x")
        sf = ttk.Frame(left)
        sf.pack(fill="x")
        self.ser_n = ttk.Combobox(sf, width=8, state="readonly", values=["Serial", "Serial1", "Serial2"])
        self.ser_n.set("Serial")
        self.ser_n.pack(side="left")
        self.ser_in = ttk.Entry(sf)
        self.ser_in.pack(side="left", fill="x", expand=True)
        self.ser_in.bind("<Return>", self.send_serial)
        ttk.Button(sf, text="Send", command=self.send_serial).pack(side="left")

        top = ttk.Frame(right)
        top.pack(fill="x")
        lf = ttk.LabelFrame(top, text="Pins to display (Ctrl/Shift multi-select)")
        lf.pack(side="left", fill="y")
        self.lb = tk.Listbox(lf, selectmode="extended", height=14, exportselection=False, font=("Consolas", 9),
                             width=26)
        for n in VALID_GPIO:
            self.lb.insert("end", pin_label(n))
        sb = ttk.Scrollbar(lf, command=self.lb.yview)
        self.lb.config(yscrollcommand=sb.set)
        self.lb.pack(side="left")
        sb.pack(side="left", fill="y")
        for n in (13, 14, 25):
            self.lb.selection_set(VALID_GPIO.index(n))

        ctl = ttk.LabelFrame(top, text="Scope / environment")
        ctl.pack(side="left", fill="both", expand=True, padx=4)
        ttk.Label(ctl, text="Time/div").grid(row=0, column=0, sticky="w")
        self.tdiv = ttk.Combobox(ctl, width=9, state="readonly",
                                 values=["1 us", "10 us", "100 us", "1 ms", "10 ms", "100 ms", "500 ms", "1 s"])
        self.tdiv.set("100 ms")
        self.win_s = 1.0
        self.tdiv.bind("<<ComboboxSelected>>", self.tdiv_changed)
        self.tdiv.grid(row=0, column=1)
        self.freeze = tk.BooleanVar()
        ttk.Checkbutton(ctl, text="Freeze", variable=self.freeze, command=self.toggle_freeze).grid(row=0, column=2)
        ttk.Label(ctl, text="Speed").grid(row=1, column=0, sticky="w")
        self.speed = ttk.Combobox(ctl, width=9, state="readonly", values=SPEEDS)
        self.speed.set("Auto")
        self.speed.grid(row=1, column=1)
        self.speed.bind("<<ComboboxSelected>>", self.speed_changed)
        self.speed_lbl = ttk.Label(ctl, text="1x")
        self.speed_lbl.grid(row=1, column=2, sticky="w")
        ttk.Label(ctl, text="I2C devices").grid(row=2, column=0, sticky="w")
        self.i2c = ttk.Entry(ctl, width=14)
        self.i2c.insert(0, "0x3C")
        self.i2c.grid(row=2, column=1, columnspan=2, sticky="we")
        ttk.Label(ctl, text="Drive pin").grid(row=3, column=0, sticky="w")
        self.drv_pin = ttk.Combobox(ctl, width=9, state="readonly",
                                    values=[f"GPIO{n}" for n in VALID_GPIO if n not in FLASH_PINS])
        self.drv_pin.set("GPIO33")
        self.drv_pin.grid(row=3, column=1)
        self.drv_pin.bind("<<ComboboxSelected>>", self.pin_changed)
        self.drv_on = tk.BooleanVar()
        ttk.Checkbutton(ctl, text="apply", variable=self.drv_on, command=self.drive).grid(row=3, column=2)
        self.drv_v = tk.DoubleVar(value=0.0)
        ttk.Scale(ctl, from_=0, to=VDD, variable=self.drv_v, command=lambda _=None: self.drive(),
                  length=240).grid(row=4, column=0, columnspan=3, sticky="we")
        self.drv_lbl = ttk.Label(ctl, text="0.00 V")
        self.drv_lbl.grid(row=5, column=0, columnspan=3, sticky="w")
        self.legend = tk.Text(ctl, height=8, width=58, font=("Consolas", 9), state="disabled")
        self.legend.grid(row=6, column=0, columnspan=3, sticky="we")

        self.scope = tk.Canvas(right, bg="#0b1a0b", highlightthickness=0)
        self.scope.pack(fill="both", expand=True)
        self.scope.bind("<MouseWheel>", self.scope_zoom)
        self.scope.bind("<Button-4>", self.scope_zoom)
        self.scope.bind("<Button-5>", self.scope_zoom)
        self.scope.bind("<ButtonPress-1>", self.scope_press)
        self.scope.bind("<B1-Motion>", self.scope_drag)
        self.scope.bind("<Double-Button-1>", self.scope_reset)
        self.tick()

    # ---- actions --------------------------------------------------------------------------------------
    def apply_i2c(self):
        devs = {}
        for tok in re.split(r"[,\s]+", self.i2c.get().strip()):
            try:
                if tok:
                    devs[int(tok, 0)] = I2CDevice()
            except ValueError:
                self.q.put(f"[warn] bad I2C address '{tok}'\n")
        self.esp.i2c = devs

    def run(self):
        self.stop()
        self.esp = ESP32(self.q.put)
        self.apply_i2c()
        self.driven = None
        self.speed_changed()
        self.drive()
        self.frozen = False
        self.freeze.set(False)
        self.esp.run(self.editor.get("1.0", "end"), {"Auto": None, "C++": "cpp", "Python": "py"}[self.lang.get()])

    def stop(self):
        self.esp.halt()

    def speed_changed(self, _=None):
        v = self.speed.get()
        self.esp.auto = v == "Auto"
        if not self.esp.auto:
            self.esp.set_scale(float(v[:-1]))

    def view_py(self):
        w = tk.Toplevel(self.root)
        w.title("Generated Python")
        t = tk.Text(w, font=("Consolas", 9), width=110, height=45)
        t.pack(fill="both", expand=True)
        t.insert("1.0", getattr(self.esp, "generated", "(run first)"))

    def open(self):
        f = filedialog.askopenfilename(filetypes=[("Sketch", "*.ino *.cpp *.c *.py"), ("All", "*.*")])
        if f:
            self.editor.delete("1.0", "end")
            self.editor.insert("1.0", open(f, encoding="utf-8", errors="replace").read())

    def save(self):
        f = filedialog.asksaveasfilename(defaultextension=".ino")
        if f:
            open(f, "w", encoding="utf-8").write(self.editor.get("1.0", "end"))

    def send_serial(self, _=None):
        n = ["Serial", "Serial1", "Serial2"].index(self.ser_n.get())
        self.esp.feed_serial(n, self.ser_in.get() + "\n")
        self.ser_in.delete(0, "end")

    def toggle_freeze(self):
        self.frozen = self.freeze.get()
        self.frozen_t = self.esp.now()

    def pin_changed(self, _=None):
        if self.driven is not None:
            self.esp.drive(self.driven, None)
            self.driven = None
        self.drive()

    def drive(self):
        n = int(self.drv_pin.get()[4:])
        v = self.drv_v.get()
        self.drv_lbl.config(text=f"{v:.2f} V on {self.drv_pin.get()}")
        if self.drv_on.get():
            self.esp.drive(n, v)
            self.driven = n
        elif self.driven is not None:
            self.esp.drive(self.driven, None)
            self.driven = None

    def window(self):
        return self.win_s

    def tdiv_changed(self, _=None):
        val, unit = self.tdiv.get().split()
        self.win_s = float(val) * {"us": 1e-6, "ms": 1e-3, "s": 1.0}[unit] * 10
        self.draw()

    def _scope_geom(self):
        w = max(self.scope.winfo_width(), 200)
        ml, pw = 50, w - 50 - 10
        return ml, pw

    def _view(self):
        t_end = self.frozen_t if self.frozen else self.esp.now()
        return t_end, self.win_s

    def scope_zoom(self, ev):
        up = ev.num == 4 or getattr(ev, "delta", 0) > 0
        f = 0.8 if up else 1.25
        ml, pw = self._scope_geom()
        t_end, win = self._view()
        frac = min(max((ev.x - ml) / pw, 0.0), 1.0)
        new = min(max(win * f, 1e-7), 1000.0)
        if self.frozen:  # keep the time under the cursor fixed
            tc = t_end - win + frac * win
            self.frozen_t = tc + (1 - frac) * new
        self.win_s = new
        self.tdiv.set(fmt_div(new / 10))
        self.draw()

    def scope_press(self, ev):
        self._drag = (ev.x, self.frozen_t if self.frozen else self.esp.now())

    def scope_drag(self, ev):
        if not getattr(self, "_drag", None):
            return
        ml, pw = self._scope_geom()
        x0, t_start = self._drag
        if not self.frozen:
            self.frozen = True
            self.freeze.set(True)
        self.frozen_t = min(t_start - (ev.x - x0) / pw * self.win_s, self.esp.now())
        self.draw()

    def scope_reset(self, _=None):
        self.frozen = False
        self.freeze.set(False)
        self.draw()

    # ---- periodic update ---------------------------------------------------------------------------------
    def tick(self):
        text = []
        while not self.q.empty():
            text.append(self.q.get())
        if text:
            self.console.config(state="normal")
            self.console.insert("end", "".join(text))
            n = int(self.console.index("end-1c").split(".")[0])
            if n > 3000:
                self.console.delete("1.0", f"{n - 2000}.0")
            self.console.see("end")
            self.console.config(state="disabled")
        e = self.esp
        n = sum(t.is_alive() for t in e.threads)
        e.autoscale()
        self.speed_lbl.config(text=f"{e.clock.scale:g}x")
        self.status.config(text=f"running  t={e.now():.4f}s  speed {e.clock.scale:g}x  tasks={n}"
                           if n and not e.stop else "stopped")
        t_draw = time.perf_counter()
        self.draw()
        # keep the GUI to ~20% of the CPU time so the emulator thread isn't starved by heavy scope drawing
        d = time.perf_counter() - t_draw
        self.root.after(max(70, int(d * 6000)), self.tick)

    # ---- scope rendering ---------------------------------------------------------------------------------------
    def trace(self, p, t0, t1, ml, pw, mt, ph):
        times, states = p.times, p.states
        i0 = max(bisect.bisect_right(times, t0) - 1, 0)
        i1 = bisect.bisect_right(times, t1)
        span = t1 - t0

        def X(t):
            return ml + pw * (t - t0) / span

        def Y(v):
            return mt + ph * (1 - v / 3.5)

        def level(s):
            k = s[0]
            if k == "L":
                return VDD if s[1] else 0.0
            if k == "P":
                return VDD * s[2]
            return s[1]

        segs = max(i1 - i0, 1)
        self._band = None
        if segs > 3000 and np is not None:
            # dense waveform: exact per-pixel-column average (integral of the level) plus a min/max
            # peak-detect band, so no edge can be missed (no aliasing)
            cols = max(int(pw), 1)
            bw = span / cols
            T = np.array(times[i0:i1], dtype=float)
            L = np.array([level(s) for s in states[i0:i1]], dtype=float)
            n = len(T)
            if n == 0:
                return []
            T[0] = max(T[0], t0)
            E = np.append(T[1:], t1)
            C = np.concatenate(([0.0], np.cumsum(L * (E - T))))
            B = t0 + bw * np.arange(cols + 1)
            idx = np.clip(np.searchsorted(T, B, side="right") - 1, 0, n - 1)
            integ = C[idx] + L[idx] * (B - T[idx])
            avg = np.diff(integ) / bw
            s_idx = idx[:-1]
            e_idx = np.maximum(np.clip(np.searchsorted(T, B[1:], side="left") - 1, 0, n - 1), s_idx)
            ri = np.empty(2 * cols, dtype=np.intp)
            ri[0::2], ri[1::2] = s_idx, e_idx + 1
            Lp = np.append(L, L[-1])
            hi = np.maximum.reduceat(Lp, ri)[0::2]
            lo = np.minimum.reduceat(Lp, ri)[0::2]
            xs = ml + pw * (np.arange(cols) + 0.5) / cols
            ys = lambda v: mt + ph * (1 - v / 3.5)
            top, bot = ys(hi), ys(lo)
            band = np.empty(4 * cols)
            band[0:2 * cols:2], band[1:2 * cols:2] = xs, top
            band[2 * cols::2], band[2 * cols + 1::2] = xs[::-1], bot[::-1]
            self._band = band.tolist()
            pts = np.empty(2 * cols)
            pts[0::2], pts[1::2] = xs, ys(avg)
            return pts.tolist()
        if segs > 3000:  # no numpy: coarse per-column sampling
            cols = int(min(pw, 400))
            bw = span / cols
            S = 8
            n = len(states)
            br = bisect.bisect_right
            pts = []
            for b in range(cols):
                tb = t0 + b * bw
                tot = 0.0
                for j in range(S):
                    ts = tb + bw * (j + ((b * 0.6180339 + j * 0.3819660) % 1.0)) / S
                    tot += level(states[min(max(br(times, ts) - 1, 0), n - 1)])
                pts += [ml + pw * (b + 0.5) / cols, Y(tot / S)]
            return pts
        pts, cur = [], None

        def add(t, v):
            nonlocal cur
            if cur is None:
                pts.extend([X(t), Y(v)])
            elif v != cur:
                pts.extend([X(t), Y(cur), X(t), Y(v)])
            cur = v

        for k in range(i0, i1):
            ts = max(times[k], t0)
            te = min(times[k + 1], t1) if k + 1 < len(times) else t1
            s = states[k]
            if s[0] == "P" and s[1] > 0:
                per = 1.0 / s[1]
                if per * pw / span < 3 or (te - ts) / per > 1500:
                    add(ts, VDD * s[2])
                    continue
                c = s[3] + math.floor((ts - s[3]) / per) * per
                while c < te:
                    hi_end = c + s[2] * per
                    if s[2] > 0 and hi_end > ts:
                        add(max(c, ts), VDD)
                    if hi_end < te and c + per > ts:
                        add(max(hi_end, ts), 0.0)
                    elif s[2] == 0:
                        add(ts, 0.0)
                    c += per
            else:
                add(ts, level(s))
        if cur is not None:
            pts.extend([X(t1), Y(cur)])
        return pts

    def draw(self):
        c = self.scope
        c.delete("all")
        w, h = max(c.winfo_width(), 200), max(c.winfo_height(), 200)
        ml, mb, mt = 50, 20, 10
        pw, ph = w - ml - 10, h - mb - mt
        for i in range(11):
            x = ml + pw * i / 10
            c.create_line(x, mt, x, mt + ph, fill="#1d3a1d")
        for v in (0, 0.825, 1.65, 2.475, 3.3):
            y = mt + ph * (1 - v / 3.5)
            c.create_line(ml, y, ml + pw, y, fill="#1d3a1d")
            c.create_text(ml - 4, y, text=f"{v:.2f}V", fill="#7c7", anchor="e", font=("Consolas", 8))
        win = self.window()
        t_end = self.frozen_t if self.frozen else self.esp.now()
        t0 = t_end - win
        lines = []
        for k, i in enumerate(self.lb.curselection()):
            n = VALID_GPIO[i]
            col = COLORS[k % len(COLORS)]
            p = self.esp.pins[n]
            try:
                pts = self.trace(p, t0, t_end, ml, pw, mt, ph)
            except IndexError:  # history was trimmed by the emulator thread mid-draw; retry next frame
                pts = []
            if self._band:
                c.create_polygon(*self._band, fill=col, outline="", stipple="gray25")
            if len(pts) >= 4:
                c.create_line(*pts, fill=col, width=1.5)
            v = p.voltage_at(t_end)
            m = self.esp.measure(p, t_end)
            extra = f" {fmt_hz(m[0])} {m[1] * 100:.1f}%" if m and m[0] > 0 else ""
            mode = {INPUT: "IN", OUTPUT: "OUT", INPUT_PULLUP: "IN_PU", INPUT_PULLDOWN: "IN_PD",
                    OUTPUT_OPEN_DRAIN: "OD"}.get(p.mode, "?")
            own = f" [{p.owner}]" if p.owner else ""
            lines.append((col, f"GPIO{n:<2} {mode:5} {v:.3f}V{extra}{own}"))
        c.create_text(ml, h - 8, anchor="w", fill="#7c7", font=("Consolas", 8),
                      text=f"{fmt_div(win / 10)}/div   window {win:.6g} s   t={t_end:.4f} s (emulated)"
                           "   [wheel: zoom, drag: pan, double-click: live]")
        self.legend.config(state="normal")
        self.legend.delete("1.0", "end")
        for i, (col, text) in enumerate(lines):
            self.legend.tag_config(f"c{i}", foreground=col)
            self.legend.insert("end", text + "\n", f"c{i}")
        self.legend.config(state="disabled")


if __name__ == "__main__":
    r = tk.Tk()
    App(r)
    r.mainloop()
