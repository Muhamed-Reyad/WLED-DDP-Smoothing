# DDP Smoothing (Exponential Decay) — Feature Notes

This document summarizes the DDP smoothing feature that was added to this WLED
checkout (v16.0.1), the source changes that implement it, and the design reached
while integrating and building it for ESP32.

---

## 1. Goal

When receiving realtime DDP frames, the source frame rate may be visibly choppy
(twitchy motion is common at 20–40 fps), or the source may output discrete frames
that the strip snaps between. The feature lets the device *smooth* that motion:

- Incoming DDP pixels are intercepted **before** they are written to the strip and
  stored in a single RGBW **target** plane.
- `ddpSmoothLoop()` — driven from the main loop — runs on a **fixed 20 ms tick**
  (matching the source's ~50 fps capture cadence) and decays a separate RGBW
  **current** plane toward the target with one exponential blend step per tick:
  ```
  current' = current + (target - current) * speed / 256
  ```
- `speed` (the `ddpSmoothingSpeed` setting, 1–255, default 80) controls how much of
  the remaining gap is erased per tick. At `speed=80`, 80/256 = **31.25%** of the
  gap closes on every tick — i.e. roughly `0.6875^n` of the gap remains after *n*
  ticks: a classic exponential decay curve (start fast, settle quickly).

This is perceptually very different from a linear ramp: motion starts fast and
settles into the new value quickly, which reads as smooth for ambient/backlight
capture. Every source frame is matched frame-for-frame — there is no frame
dropping, no sub-frame interpolation, and no added output latency (the output is
as current as the latest received frame, which is the whole point).

## 2. How the flow works

DDP packets arrive through the async UDP callback (`ESPAsyncE131::parsePacket` →
`handleE131Packet` → `handleDDPPacket` in `wled00/e131.cpp`). Without smoothing, the
publish path is:

```
handleDDPPacket()
    └─ setRealtimePixel(...)   → strip._pixels[]
        └─ (on PUSH) e131NewData = true
            handleNotifications()  → strip.show() / trigger()
```

With smoothing enabled, `handleDDPPacket()` does **not** touch the strip. Instead:

```
handleDDPPacket()
    ├─ ddpSmoothStore(i, r, g, b, w)   → writes into the target plane
    └─ (on PUSH) ddpSmoothFrameDone()   → flags that a complete frame is available

main loop → ddpSmoothLoop() (called from handleNotifications())
    ├─ first frame: copies the target plane onto the strip directly (no black screen)
    └─ every 20 ms tick: one decay step  current' = current + (target-current)*speed/256
       written back into the current plane and onto the strip through setRealtimePixel()
```

Rendering uses the exact same per-LED addressing as the original direct path
(`setRealtimePixel(i, ...)`, i.e. index `i + arlsOffset`), which means:

- `arlsOffset` is honored identically.
- `useMainSegmentOnly` behaviour is unchanged.
- Out-of-sequence packet rejection, DMXAddress offset and sequence-number tracking
  keep working.

## 3. Timing model

Interpolation progress is driven **by a fixed 20 ms tick**, not by the observed
source interval and not by loop-iteration counters:

```
tick = 20 ms            (constant, DDP_SMOOTH_TICK_MS)
current' = current + (target - current) * speed / 256
```

- The decay is applied at most once per tick; a main-loop call that finds fewer than
  20 ms elapsed since the last tick does nothing.
- If the main loop cannot keep up with the tick cadence, whole ticks are skipped and
  counted (see stats in §9); the decay simply advances more slowly during that
  window — never a pop, never an accumulation error.
- Because the output is wall-clock-driven, it stays smooth even while the main loop
  is busy parsing UDP traffic.

The source fundamentally sends **discrete frames**. Widths/positions change abruptly
between frames; the exponential decay is what blurs those abrupt snaps into smooth
motion. The decay target plane is simply overwritten by each new frame — nothing has
to be "committed" or locked, and no history is kept.

## 4. Source changes

| File | Change |
|------|--------|
| `wled00/e131.cpp` | Smoothing engine: `ddpSmoothEnsureBuffers()`, `ddpSmoothStore()`, `ddpSmoothFrameDone()`, `ddpSmoothRenderDecay()`, `ddpSmoothLoop()`; two-plane (current/target) exponential-decay model; integration into `handleDDPPacket()` |
| `wled00/udp.cpp` | Call `ddpSmoothLoop()` from `handleNotifications()` (after the realtime-timeout check) |
| `wled00/wled.h` | New globals `ddpSmoothingEnabled`, `ddpSmoothingSpeed`; macros `DDP_SMOOTHING_DEFAULT_SPEED` (80), `DDP_SMOOTHING_MAX_SPEED` (255) |
| `wled00/fcn_declare.h` | Declares `bool ddpSmoothLoop()` |
| `wled00/cfg.cpp` | Persists `if_live["ddp-sm"]` (bool), `if_live["ddp-sms"]` (1–255) |
| `wled00/set.cpp` | Parses `SM` / `SMS` form fields |
| `wled00/xml.cpp` | Emits the settings-JS values for `SM` / `SMS` |
| `wled00/data/settings_sync.htm` | New "DDP smoothing" checkbox + "DDP smoothing speed" number input |

### 4.1 Configuration

- **Settings UI** (Settings → Sync → Realtime):
  - `DDP smoothing` checkbox
  - `DDP smoothing speed` (1–255, default 80) — fraction of the remaining gap erased
    per 20 ms tick; higher = faster response, lower = smoother/softer decay.
- **JSON config** (`/json/cfg`, `interfaces.live`):
  - `ddp-sm`  : bool
  - `ddp-sms` : int (1–255)
- The form key `SMS` was chosen because `DS` is already used by `serverDescription`
  ("Device Name") and `SM` for the smoothing toggle; `SMS` (smoothing speed) is free.

### 4.2 Behavioural notes

- Saving the setting takes effect immediately on the next received DDP frame; no
  reboot required.
- Disabling smoothing mid-stream returns to the stock direct-render path on the
  next packet.
- While `realtimeOverride` is active, smoothing is suspended (so usermod/effect
  override output is not stomped) and stock behaviour applies.
- When the source stops (realtime timeout), `exitRealtime()` returns the strip to
  normal effects; there is deliberately **no** extra fade-to-black path — the last
  decayed frame is simply taken over by the normal effect renderer.

## 5. Memory strategy & ESP8266 trade-offs

The engine uses two RGBW planes of the strip length, allocated with a single
contiguous `d_malloc()` on the **first received DDP packet after the feature is
enabled**:

```cpp
alloc = strip.getLengthTotal() * 4 * 2   // current + target plane, RGWB per LED
```

- Footprint while active ≈ **8 bytes per LED** (e.g. 222 LEDs ≈ 1.8 KB) — far
  smaller than the previous ring design's ~64 B/LED.
- If allocation fails, the feature silently degrades to the stock direct-render path
  (no crash, no further retry until the strip length changes).
- The planes are **kept allocated** while the session runs (reset each boot). This
  avoids free/realloc churn during enable/disable toggles — which would otherwise
  fragment the heap.

### CPU trade-off (ESP8266)

Decaying hundreds of pixels on every tick costs CPU, but stays cheap on the 80/160
MHz ESP8266 core because:

- The blend is **pure integer** per channel (`(target - current) * speed) >> 8` with
  an arithmetic shift) — no floating point in the render loop at all.
- The render loop runs in the main `loop()`, so it yields to other work.
- Skipped ticks (main loop too busy) degrade gracefully — motion just slows, never
  jerks or accumulates error.

## 6. Findings / edge cases

1. **`realtime.cpp` doesn't exist in 16.0.1.** Realtime is split across
   `wled00/e131.cpp` (protocol parsing) and `wled00/udp.cpp`
   (`handleNotifications()`, realtime lock/show). Plan accordingly when porting.
2. **DDP arrives on an async task.** `ESPAsyncE131::parsePacket()` is an `AsyncUDP`
   callback, so `handleDDPPacket` runs off the main loop. The async task only writes
   the target plane + `ddpSmoothGotFrame`; the main loop only reads the target plane
   and writes the current plane. Byte-at-a-time access may tear, but the worst case
   is one transient blend artifact corrected by the next tick — never a crash.
3. **The two painters never contend on the same buffer**, so no mutex is required —
   same concurrency model WLED already uses for `_pixels`.
4. **Sequence tracking must be preserved.** With smoothing active the push event is
   consumed by `ddpSmoothFrameDone()`, but `e131LastSequenceNumber[]` is still
   updated so the existing out-of-sequence rejection keeps working.
5. **First frame has no "previous" state.** The first *received* frame is copied onto
   the strip directly (no black-screen fade-in). Only subsequent frames are decayed
   toward, starting from that snapshot.
6. **Source end:** the normal realtime timeout (`realtimeTimeoutMs`, 2.5 s) still
   applies, so a dead source results in the expected exit back to normal effects.
7. **Settings key collision:** `DS` was rejected (used by Device Name) in favor of
   `SM` (toggle) + `SMS` (speed); both are free.
8. **Strip show cadence.** `strip.show()`/`trigger()` is reused exactly as in the
   stock DDP path (`useMainSegmentOnly ? trigger() : show()`), so the existing
   realtime render path remains intact. The extra show calls happen at most every
   20 ms (50 fps), well within the LED bus capability for typical strip lengths.
9. **Exponential decay stalls short of the target by <1 LSB** — `(target-current)*
   speed/256` truncates toward zero when the gap is tiny (e.g. gap of 1 with speed 80
   yields a zero step). This is exactly how the source's `blend8Log` behaves and is
   imperceptible (≤1/255 of a channel).
10. **Speed=255 ≈ but is not "instant".** 255/256 of the gap is erased per tick, so
    a full-scale jump needs a few ticks; use `speed=255` for near-direct behaviour.

## 7. Build & verification

- `npm ci && npm run build` — web UI rebuilt (`settings_sync.htm` change lands in
  `wled00/html_settings.h`; these generated headers must never be hand-edited).
- `npm test` — `node --test` passes.
- `pio run -e esp32dev` — **SUCCESS**:
  ```
  RAM:   [==        ]  25.0% (used 81856 bytes from 327680 bytes)
  Flash: [========  ]  82.7% (used 1300309 bytes from 1572864 bytes)
  ```
  Output: `build_output\release\WLED_16.0.1_ESP32.bin`

### 7.1 Environment workaround (not a repo change)

The Tasmota `platform-espressif32` distribution pins `tool-esptoolpy` to a Tasmota
"esptool" zip that ships only upstream sources — it lacks the `package-postinstall.py`
that PlatformIO runs after install. On this machine that aborted the ESP32 toolchain
install with `'package-postinstall.py' is not recognized...`. Fixed by adding a
no-op/idempotent `package-postinstall.py` to
`%USERPROFILE%\.platformio\packages\tool-esptoolpy\`. The platform invokes
`esptool.py` directly for builds/uploads, so no launcher executable is required.

## 8. Suggested follow-ups

- Test on a live DDP source (e.g. LedFx): A/B the smoothing speed (`SMS` 40 vs 80 vs
  160) and tune the default `ddpSmoothingSpeed`.
- Optionally add an `if_live["ddp-sm"]` toggle to the web Live tab (not just the
  settings page).
- Consider exposing the current speed value in `/json/info` alongside the existing
  diagnostics below.
- Run a `pio run -e nodemcuv2` pass to gauge ESP8266 flash/RAM impact with the
  feature compiled in.

## 9. Debugging the effective output rate

Enable a debug build (`-D WLED_DEBUG`) and feed your DDP source, then watch the
serial log. Roughly every **2 s** `ddpSmoothDebugStats()` prints:

```
DDP smooth: 98 frames rendered, 0 ticks skipped (SMS=80, tick=20ms)
```

- `frames rendered` — number of decay steps actually pushed to the strip in the last
  2 s window (~50 ticks/s × 2 s ≈ 100 at full cadence).
- `ticks skipped` — 20 ms ticks dropped because the main loop could not keep up.
  **Non-zero ⇒ the main loop is too busy to hit the 50 Hz show cadence.**
- `SMS` — the configured smoothing speed; `tick` — the fixed tick interval.

A cheaper always-on check that needs no debug build: read
`http://<ip>/json/state` → `leds["fps"]`, which WLED already computes from the real
show cadence.

Since v2, `/json/info` → `leds` also exposes live smoothing diagnostics (no debug
build needed):

```
"ddpSmoothing" ... /json/info → leds:
  ddpSmooth:   true/false   ← smoothing engine engaged right now (frame shown while DDP streaming)
  ddpRendered: N            ← decay frames actually shown since last poll (≈ rate/s)
  ddpSkipped:  M            ← 20 ms ticks dropped (main loop too busy)
```