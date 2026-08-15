# DDP Smoothing (Frame Interpolation) — Feature Notes

This document summarizes the DDP frame-interpolation ("smoothing") feature that was
added to this WLED checkout (v16.0.1), the source changes that implement it, and the
findings reached while designing, integrating, and building it for ESP32.

---

## 1. Goal

When receiving realtime DDP frames, the source frame rate may be visibly choppy
(twitchy motion is common at 20–40 fps). The feature lets the device *smooth* that
motion:

- Incoming DDP frames are intercepted **before** they are written to the strip.
- The pipeline is delayed by roughly **one source frame**.
- A configurable number of interpolated frames (`ddpSmoothingFrames`) is rendered
  between the currently displayed frame and the frame that just arrived.
- Each interpolated frame is a per-channel integer blend of
  `current` and `target`, with a **logarithmic easing curve** (`ease-out`):
  motion starts fast and decelerates when approaching the target:

  ```
  eased  = log2(1 + p * 127) / 7            (p = 0..1, pre-computed 256-entry LUT)
  channel_out = current + (target - current) * eased_step / 255
  ```

  The easing LUT is built once on first use (`ddpSmoothBuildEaseLUT()`) so the
  per-pixel hot path stays pure integer.

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
    ├─ ddpSmoothStore(i, r, g, b, w)   → writes into target buffer
    └─ (on PUSH) ddpSmoothFrameDone()   → arms the interpolator

main loop → ddpSmoothLoop() (called from handleNotifications())
    └─ renders interpolated frames into the strip at the observed source rate,
       writing through setRealtimePixel() so all existing address/mapping logic
       (arlsOffset, main-segment-only, led maps) is preserved unchanged.
```

Rendering uses the exact same per-LED addressing as the original direct path
(`setRealtimePixel(i, ...)`, i.e. index `i + arlsOffset`), which means:

- `arlsOffset` is honored identically.
- `useMainSegmentOnly` behaviour is unchanged.
- Out-of-sequence packet rejection, DMXAddress offset and sequence-number tracking
  keep working.

## 3. Interpolation timing model (robustness finding)

The original plan stepped an index counter per display tick. That is fragile: the
main loop's iteration rate varies wildly while it parses UDP traffic, so a
counter-based approach either misses steps or overshoots the target frame.

**Finding:** time-based progress is far more robust. When a PUSH frame arrives, the
observed inter-arrival interval of the source is used as the interpolation duration:

```
duration = now - lastFrameArrival          (clamped to [1, 5000] ms)
steps    = ddpSmoothingFrames + 1
```

Every main-loop call, progress is computed from wall clock:

```
seq  = elapsed * steps / duration          (quantized to `steps` levels)
step = seq * 255 / steps                   (0 = current, 255 = target)
```

Only a *changed* level causes a render, so:

- The output is locked to the source frame rate — exactly `steps` distinct frames
  per source frame.
- The final frame of each interpolation **always equals the target** exactly.
- If the main loop is too busy to render all levels, the display simply skips levels
  and still catches up to the exact target frame — no accumulation, no pop.
- The number of physically rendered frames is capped at the main-loop rate; faster
  loops show more of the interpolated levels.

The "one frame delay" is inherent: interpolation starts from what is currently on
screen and takes about one source-frame interval to reach the freshly received frame.

## 4. Source changes

| File | Change |
|------|--------|
| `wled00/e131.cpp` | Interpolation engine: `ddpSmoothEnsureBuffers()`, `ddpSmoothStore()`, `ddpSmoothFrameDone()`, `ddpSmoothRenderStep()`, `ddpSmoothLoop()`; integration into `handleDDPPacket()` |
| `wled00/udp.cpp` | Call `ddpSmoothLoop()` from `handleNotifications()` (after the realtime-timeout check) |
| `wled00/wled.h` | New globals `ddpSmoothingEnabled`, `ddpSmoothingFrames`; macros `DDP_SMOOTHING_DEFAULT_FRAMES` (6), `DDP_SMOOTHING_MAX_FRAMES` (30) |
| `wled00/fcn_declare.h` | Declares `bool ddpSmoothLoop()` |
| `wled00/cfg.cpp` | Persists `if_live["ddp-sm"]` (bool) and `if_live["ddp-smf"]` (1–30) |
| `wled00/set.cpp` | Parses `SM` / `SMF` form fields |
| `wled00/xml.cpp` | Emits the settings-JS values for `SM` / `SMF` |
| `wled00/data/settings_sync.htm` | New "DDP smoothing" checkbox + "DDP smoothing frames" number input |

### 4.1 Configuration

- **Settings UI** (Settings → Sync → Realtime):
  - `DDP smoothing` checkbox
  - `DDP smoothing frames` (1–30, default 6) — frames interpolated between incoming
    DDP frames; output delayed by ~1 frame.
- **JSON config** (`/json/cfg`, `interfaces.live`):
  - `ddp-sm` : bool
  - `ddp-smf` : int (1–30)
- The form keys `SM`/`SMF` were chosen because `DS` is already used by
  `serverDescription` ("Device Name").

### 4.2 Behavioural notes

- Saving the setting takes effect immediately on the next received DDP frame; no
  reboot required.
- Disabling smoothing mid-stream returns to the stock direct-render path on the
  next packet.
- While `realtimeOverride` is active, interpolation is suspended (so usermod/effect
  override output is not stomped) and stock behaviour applies.

## 5. Memory strategy & ESP8266 trade-offs

The original proposal suggested static double RGB buffers. Based on the project's
own guidance (keep allocation static to avoid fragmentation on ESP8266, but ESP8266
only has ~30–40 KB free), we chose **lazy, demand allocation** instead:

- Two RGBW planes (`current` + `target`) sized to `strip.getLengthTotal()` are
  allocated via `d_malloc()` on the **first received DDP packet after the feature is
  enabled**.
- Footprint while active ≈ **8 bytes per LED** (e.g. 300 LEDs ≈ 2.4 KB).
- If allocation fails, the feature silently degrades to the stock direct-render path
  (no crash, no further retry until the strip length changes).
- Buffers are **kept allocated** while the session runs (reset each boot). This
  avoids free/realloc churn during enable/disable toggles — which would otherwise
  fragment the heap — and a resize mid-interpolation would race between the async
  UDP task and the main loop. A resize is therefore only permitted while no
  interpolation is active.

### CPU trade-off (ESP8266)

Interpolating hundreds of pixels extra times at high frame rates costs CPU. To stay
affordable on the 80/160 MHz ESP8266 core:

- Pure integer LERP (32-bit) per channel — no floating point in the render loop.
  (The logarithmic easing LUT is the only float code, and runs exactly once on
  first use, then is only a table lookup.)
- The interp loop runs in the main `loop()`, so it yields to other work.
- The number of levels rendered is capped by the main-loop rate; on an ESP8266 under
  heavy UDP load the effective output rate automatically and gracefully trails off
  toward the source rate (the display stays smooth, just fewer in-between frames).
- The heavier ESP32 CPU comfortably supports the full interpolation rate.

## 6. Findings / edge cases

1. **`realtime.cpp` doesn't exist in 16.0.1.** Realtime is split across
   `wled00/e131.cpp` (protocol parsing) and `wled00/udp.cpp`
   (`handleNotifications()`, realtime lock/show). Plan accordingly when porting.
2. **DDP arrives on an async task.** `ESPAsyncE131::parsePacket()` is an `AsyncUDP`
   callback, so `handleDDPPacket` runs off the main loop. The interp reader/writer
   split is the same concurrency model WLED already uses for `_pixels` (byte atomic
   writes, no locking). Worst case is one transient blend artifact mid-render — never
   a crash.
3. **Sequence tracking must be preserved.** With smoothing active the push event is
   consumed by `ddpSmoothFrameDone()`, but `e131LastSequenceNumber[]` is still
   updated so the existing out-of-sequence rejection keeps working.
4. **First frame has no "previous" frame.** It is displayed immediately (single
   "frame" at step 255); interpolation starts from the second frame onward.
5. **Source pause:** the interpolation duration is clamped to 5 s, and the normal
   realtime timeout still applies, so a dead source results in the expected exit back
   to normal effects.
6. **Settings key collision:** `DS` was rejected (used by Device Name) in favor of
   `SM`/`SMF`.
7. **Wayback: the strip show cadence.** `strip.show()`/`trigger()` is reused exactly
   as in the stock DDP path (`useMainSegmentOnly ? trigger() : show()`), so the
   existing realtime render path (which skips effect servicing in non-main-segment
   realtime mode) remains intact.
8. **Physical refresh ceiling.** The requested output rate is
   `source_fps × (ddpSmoothingFrames + 1)`, but it cannot exceed what the LED bus
   can physically push (`≈ 1000 / (LEDs × 0.03 ms)` for WS2812 @800 kHz). Over that
   ceiling, interpolation levels are skipped (see stats below) and raising `SMF`
   has no visible effect.
9. **Logarithmic easing.** Interpolation progress `p` is remapped through a
   256-entry `log2(1 + p·127) / 7` LUT (ease-out). Fast start, gentle landing on
   the target — perceptually softer than linear steps at the same level count.

## 7. Build & verification

- `npm ci && npm run build` — web UI rebuilt (`settings_sync.htm` change lands in
  `wled00/html_settings.h`; these generated headers must never be hand-edited).
- `npm test` — 16/16 passing (`node --test`).
- `pio run -e esp32dev` — **SUCCESS**:
  ```
  RAM:   [==        ]  25.0% (used 81848 bytes from 327680 bytes)
  Flash: [========  ]  82.6% (used 1299725 bytes from 1572864 bytes)
  ```
  Output: `build_output\release\WLED_16.0.1_ESP32.bin`
- `e131.cpp` initially failed to compile because the static smoothing helpers were
  defined below their first use; fixed by adding forward declarations at the top of
  `e131.cpp`.
- Logarithmic easing (`logf` LUT build, one-time) adds ~52 B of flash; no new
  runtime float in the per-pixel path.

### 7.1 Environment workaround (not a repo change)

The Tasmota `platform-espressif32` distribution pins `tool-esptoolpy` to a Tasmota
"esptool" zip that ships only upstream sources — it lacks the `package-postinstall.py`
that PlatformIO runs after install. On this machine that aborted the ESP32 toolchain
install with `'package-postinstall.py' is not recognized...`. Fixed by adding a
no-op/idempotent `package-postinstall.py` to
`%USERPROFILE%\.platformio\packages\tool-esptoolpy\`. The platform invokes
`esptool.py` directly for builds/uploads, so no launcher executable is required.

## 8. Suggested follow-ups

- Test on a live DDP source (e.g. LedFx) and tune the default `ddpSmoothingFrames`.
- Optionally add an `if_live["ddp-sm"]` toggle to the web Live tab (not just the
  settings page).
- Consider exposing interpolation stats (current step / dropped levels) in
  `/json/info` for debugging. (Today the counters only print under `WLED_DEBUG`;
  WLED already reports achieved strip FPS in `/json/state` → `leds["fps"]`.)
- Run a `pio run -e nodemcuv2` pass to gauge ESP8266 flash/RAM impact with the
  feature compiled in.

## 9. Debugging the effective output rate

Enable a debug build (`-D WLED_DEBUG`) and feed your DDP source, then watch the
serial log. Roughly every **2 s** `ddpSmoothDebugStats()` prints:

```
DDP smooth: 42 frames rendered, 3 levels skipped (SMF=10, steps=11, dur=50ms)
```

- `frames rendered` — number of interpolated frames actually pushed to the strip
  in the last 2 s window.
- `levels skipped` — quantized interpolation levels that had to be dropped because
  the main loop or the LED bus could not keep up. **Non-zero ⇒ you are at/over the
  physical refresh ceiling** (see Finding 8), and raising `SMF` will not help.

A cheaper always-on check that needs no debug build: read
`http://<ip>/json/state` → `leds["fps"]`, which WLED already computes from the real
show cadence.

Since v2, `/json/info` → `leds` also exposes live smoothing diagnostics (no debug
build needed):

```
"DDP smoothing" ... /json/info → leds:
  ddpSmooth:   true/false   ← interpolation engine engaged right now
  ddpRendered: N            ← interpolated frames actually shown since last poll (≈ rate/s)
  ddpSkipped:  M            ← interpolation levels dropped (loop/bus too slow)
```

If `ddpSkipped > 0` persists while `ddpSmooth == true`, the strip is at its physical
refresh ceiling (Finding 8) — raising `ddpSmoothingFrames` won't smooth further.