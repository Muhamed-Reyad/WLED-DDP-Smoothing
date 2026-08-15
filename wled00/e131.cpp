#include "wled.h"

#define MAX_3_CH_LEDS_PER_UNIVERSE 170
#define MAX_4_CH_LEDS_PER_UNIVERSE 128
#define MAX_CHANNELS_PER_UNIVERSE 512

// forward declarations
static void handleDDPPacket(e131_packet_t* p, size_t packetLen);
static void handleArtnetPollReply(IPAddress ipAddress);
static void prepareArtnetPollReply(ArtPollReply *reply);
static void sendArtnetPollReply(ArtPollReply *reply, IPAddress ipAddress, uint16_t portAddress);
// DDP smoothing helpers (defined below handleDDPPacket)
static bool ddpSmoothEnsureBuffers();
static void ddpSmoothStore(uint16_t i, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
static void ddpSmoothFrameDone();
static void ddpSmoothDebugStats();


/*
 * E1.31 handler
 */

//DDP protocol support, called by handleE131Packet
//handles RGB data only
static void handleDDPPacket(e131_packet_t* p, size_t packetLen) {
  static bool ddpSeenPush = false;  // have we seen a push yet?
  int lastPushSeq = e131LastSequenceNumber[0];

  if (packetLen < DDP_HEADER_LEN) return; // too short to safely read any DDP header fields

  // reject unsupported color data types (only RGB and RGBW are supported)
  //uint8_t maskedType = p->dataType & 0x3F; // mask out custom and reserved flags, only type bits are relevant
  //if (maskedType != DDP_TYPE_RGB24 && maskedType != DDP_TYPE_RGBW32) return;

  // note: for maximum compatibility we do not reject unknonw or malformed data types but simply default to RGB24 and check there is enough data available in the packet to do so
  //       also we assume 8bit per channel and currently do not support other bit depths

  // reject control, status and config packets (not implemented)
  if (p->destination == DDP_ID_CONTROL || p->destination == DDP_ID_STATUS || p->destination == DDP_ID_CONFIG) return;

  // reject query and response packets (not implemented)
  if (p->flags & (DDP_FLAGS_QUERY | DDP_FLAGS_REPLY)) return;

  bool push = p->flags & DDP_FLAGS_PUSH; // push flag means "render now"
  if (!push && (p->flags & DDP_FLAGS_STORAGE)) return; // reject "from storage" flag but still let the push flag pass if set along with it

  //reject late packets belonging to previous frame (assuming 4 packets max. before push, if more are used and packets are very late, they are still accepted)
  if (e131SkipOutOfSequence && lastPushSeq) {
    int sn = p->sequenceNum & 0xF; // sequence number is 4 bits, 1-15, 0 means unused
    if (sn) {
      if (lastPushSeq > 5) {
        if (sn > (lastPushSeq -5) && sn < lastPushSeq) return;
      } else {
        if (sn > (10 + lastPushSeq) || sn < lastPushSeq) return;
      }
    }
  }

  unsigned ddpChannelsPerLed = 3; // default to RGB
  if ((p->dataType & 0b00111000)>>3 == 0b011) ddpChannelsPerLed = 4; // RGBW data type (see DDP protocol definition)

  uint32_t start =  htonl(p->channelOffset) / ddpChannelsPerLed;
  start += DMXAddress / ddpChannelsPerLed;
  uint16_t dataLen = htons(p->dataLen);
  unsigned stop = start + dataLen / ddpChannelsPerLed;
  uint8_t* data = p->data;
  unsigned c = 0;
  if (p->flags & DDP_FLAGS_TIME) c = 4; //packet has timecode flag, we do not support it, but data starts 4 bytes later

  // ensure the received packet is at least as long as the header claims
  if (packetLen < DDP_HEADER_LEN + c + dataLen) {
    DEBUG_PRINTLN(F("DDP packet incomplete"));
    return;
  }

  unsigned numLeds = stop - start; // stop >= start is guaranteed
  unsigned maxDataIndex = numLeds * ddpChannelsPerLed; // validate bounds before accessing data array
  if (maxDataIndex > dataLen) {
    DEBUG_PRINTLN(F("DDP packet data bounds exceeded, rejecting."));
    return;
  }

  if (realtimeMode != REALTIME_MODE_DDP) ddpSeenPush = false; // just starting, no push yet
  realtimeLock(realtimeTimeoutMs, REALTIME_MODE_DDP);

  // intercept the data before it reaches the strip when smoothing is enabled
  bool smoothing = ddpSmoothingEnabled && !realtimeOverride && ddpSmoothEnsureBuffers();

  if (!realtimeOverride) {
    if (smoothing) {
      for (unsigned i = start; i < stop; i++, c += ddpChannelsPerLed) {
        ddpSmoothStore(i, data[c], data[c+1], data[c+2], ddpChannelsPerLed >3 ? data[c+3] : 0);
      }
    } else {
      for (unsigned i = start; i < stop; i++, c += ddpChannelsPerLed) {
        setRealtimePixel(i, data[c], data[c+1], data[c+2], ddpChannelsPerLed >3 ? data[c+3] : 0);
      }
    }
  }

  ddpSeenPush |= push;
  if (!ddpSeenPush || push) { // if we've never seen a push, or this is one, render display
    if (smoothing) {
      ddpSmoothFrameDone();
    } else {
      e131NewData = true;
    }
    int sn = p->sequenceNum & 0xF;
    if (sn) e131LastSequenceNumber[0] = sn;
  }
}

/*
 * DDP frame interpolation ("smoothing").
 * ======================================
 * When ddpSmoothingEnabled is true, incoming DDP pixel data is not written to
 * the strip immediately. Every received pixel is stored in a single RGBW
 * "target" plane. ddpSmoothLoop() - driven from the main loop - runs on a fixed
 * 20 ms tick (matching the source's ~50 fps capture cadence) and decays a
 * separate RGBW "current" plane toward that target with one per-tick
 * exponential blend step (a blend8Log-style update):
 *     current' = current + (target - current) * speed / 256
 * The result is written back into the current plane and painted on the strip, so
 * the output keeps asymptotically approaching whatever the newest target is;
 * every source frame is matched frame-for-frame with no frame dropping, no
 * sub-frame interpolation and no output lag.
 *
 * speed (the ddpSmoothingSpeed setting, 1..255, default 80) controls how much of
 * the remaining gap is erased per tick: at speed=80, 80/256 = 31.25% of the gap
 * closes on every tick, i.e. roughly (0.6875)^n remains after n ticks - a
 * classic exponential decay curve. This is perceptually very different from a
 * linear ramp: motion starts fast and settles into the new value quickly, which
 * is what reads as smooth for ambient/backlight capture.
 *
 * Threading: handleDDPPacket() runs on the UDP async task and only touches the
 * target plane and the flag ddpSmoothGotFrame; ddpSmoothLoop() runs on the main
 * loop and only reads the target plane while writing the current plane and the
 * strip. Per-channel byte accesses may be torn across the two sides, but the
 * worst that produces is a one-tick transient blend artifact that the next tick
 * corrects - never a crash or persistent corruption. No mutex is therefore
 * needed; the two painters never write the same buffer.
 *
 * Memory: two RGBW planes of the strip length (~8 bytes per LED while active),
 * allocated lazily only while the feature is enabled. Allocation failure degrades
 * gracefully to the default direct-render behaviour.
 */
#define DDP_SMOOTH_TICK_MS 20  // fixed millisecond cadence of one blend tick (matches the source's 20 ms capture timer)

static uint8_t  *ddpSmoothCurr    = nullptr;   // displayed/decayed RGBW state (written by the main loop)
static uint8_t  *ddpSmoothTarget  = nullptr;   // latest received source frame, RGBW (written by the async task)
static uint16_t  ddpSmoothBufLen  = 0;         // number of LEDs one plane can hold
static uint16_t  ddpSmoothReqLen  = 0;         // strip length the buffers were sized for
static bool      ddpSmoothAllocFailed = false; // no more retries until length changes

static bool      ddpSmoothGotFrame  = false;   // at least one source frame has been received (written by the async task)
static bool      ddpSmoothHaveShown = false;   // at least one frame has been put on the strip
static uint32_t  ddpSmoothLastTick  = 0;       // millis() of the previous blend tick

// Diagnostic counters (always maintained, only printed under WLED_DEBUG).
static uint32_t  ddpSmoothRenderedFrames = 0; // frames actually shown
static uint32_t  ddpSmoothSkippedLevels  = 0; // 20 ms ticks dropped because the main loop could not keep up
static uint32_t  ddpSmoothStatsLastMs    = 0; // millis() of the last stats printout

static void ddpSmoothReset() {
  ddpSmoothHaveShown = false;
  ddpSmoothLastTick  = 0;
}

// (Re)allocate the current/target planes to match the current strip length.
// Both planes share one contiguous allocation so a single d_malloc/d_free pair
// is enough. Returns true if usable, false (degrading to direct render) otherwise.
static bool ddpSmoothEnsureBuffers() {
  uint16_t total = strip.getLengthTotal();
  if (total == 0) return false;
  if (ddpSmoothCurr && ddpSmoothBufLen >= total) {
    ddpSmoothReqLen = total;
    return true;
  }
  if (ddpSmoothAllocFailed && ddpSmoothReqLen >= total) return false; // do not hammer the heap
  size_t alloc = (size_t)total * 4 * 2; // RGBW bytes per LED, current + target plane
  uint8_t *newBuf = (uint8_t*) d_malloc(alloc);
  if (!newBuf) {
    ddpSmoothReqLen = total;
    ddpSmoothAllocFailed = true;
    return false;
  }
  memset(newBuf, 0, alloc);
  d_free(ddpSmoothCurr);
  ddpSmoothCurr   = newBuf;
  ddpSmoothTarget = newBuf + (size_t)total * 4;
  ddpSmoothBufLen = total;
  ddpSmoothReqLen = total;
  ddpSmoothAllocFailed = false;
  ddpSmoothReset();
  return true;
}

// Realtime listener: called from handleDDPPacket() for every received pixel while
// smoothing is enabled. Stores the data in the target plane, using the same LED
// addressing that setRealtimePixel() would use, so rendering can reproduce it.
static void ddpSmoothStore(uint16_t i, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
  if (!ddpSmoothTarget || i >= ddpSmoothBufLen) return;
  int pix = (int)i + arlsOffset; // same mapping as setRealtimePixel()
  if (pix < 0 || pix >= (int)ddpSmoothBufLen) return; // outside the visible strip
  uint16_t p = i * 4;
  ddpSmoothTarget[p+0] = r;
  ddpSmoothTarget[p+1] = g;
  ddpSmoothTarget[p+2] = b;
  ddpSmoothTarget[p+3] = w;
}

// Realtime listener: called from handleDDPPacket() when a push frame arrives.
// Marks that at least one complete source frame is available so the main loop
// can start rendering. There is nothing else to commit: every pixel is already
// in the target plane, which is simply overwritten by the following frames.
static void ddpSmoothFrameDone() {
  ddpSmoothGotFrame = true;
}

// Render one exponential-decay step: blend each channel of the current plane one
// tick toward the target plane and paint the result on the strip, also writing
// it back into the current plane so the decay continues across ticks. Pure
// integer math (a blend8Log-style update), safe on the 80/160 MHz ESP8266 core:
//     out = current + (target - current) * speed / 256
// (target - current) may be negative, so the product uses a signed 32-bit value
// and an arithmetic right shift before being added back; the final value is
// clamped to 0..255 for display and for the running state.
static void ddpSmoothRenderDecay() {
  if (!ddpSmoothCurr || !ddpSmoothTarget) return;
  uint32_t speed = ddpSmoothingSpeed;
  if (speed < 1)  speed = 1;
  if (speed > DDP_SMOOTHING_MAX_SPEED) speed = DDP_SMOOTHING_MAX_SPEED;
  for (uint16_t i = 0; i < ddpSmoothBufLen; i++) {
    uint16_t p = i * 4;
    uint8_t r = (uint8_t)(ddpSmoothCurr[p]   + (((int32_t)ddpSmoothTarget[p]   - ddpSmoothCurr[p])   * (int32_t)speed) >> 8);
    uint8_t g = (uint8_t)(ddpSmoothCurr[p+1] + (((int32_t)ddpSmoothTarget[p+1] - ddpSmoothCurr[p+1]) * (int32_t)speed) >> 8);
    uint8_t b = (uint8_t)(ddpSmoothCurr[p+2] + (((int32_t)ddpSmoothTarget[p+2] - ddpSmoothCurr[p+2]) * (int32_t)speed) >> 8);
    uint8_t w = (uint8_t)(ddpSmoothCurr[p+3] + (((int32_t)ddpSmoothTarget[p+3] - ddpSmoothCurr[p+3]) * (int32_t)speed) >> 8);
    ddpSmoothCurr[p]   = r;
    ddpSmoothCurr[p+1] = g;
    ddpSmoothCurr[p+2] = b;
    ddpSmoothCurr[p+3] = w;
    setRealtimePixel(i, r, g, b, w);
  }
  if (useMainSegmentOnly) strip.trigger();
  else                    strip.show();
}

// Main-loop driver (called from handleNotifications()). Runs the exponential
// decay at a fixed 20 ms tick. While no source frame has arrived yet it does
// nothing; the first received frame is copied onto the strip directly (so the
// screen is not black on startup), then every tick after that blends the current
// plane one step toward the target plane until a newer frame replaces it.
bool ddpSmoothLoop() {
  if (!ddpSmoothingEnabled || realtimeMode != REALTIME_MODE_DDP || realtimeOverride) {
    ddpSmoothReset();
    return false;
  }
  if (!ddpSmoothCurr || !ddpSmoothTarget) return false;

  uint32_t now = millis();

  if (!ddpSmoothHaveShown) {
    // first visibility: paint the first received frame directly to avoid a black screen
    if (!ddpSmoothGotFrame) return false;
    memcpy(ddpSmoothCurr, ddpSmoothTarget, (size_t)ddpSmoothBufLen * 4);
    ddpSmoothHaveShown = true;
    ddpSmoothLastTick = now;
    for (uint16_t i = 0; i < ddpSmoothBufLen; i++) {
      uint16_t p = i * 4;
      setRealtimePixel(i, ddpSmoothCurr[p], ddpSmoothCurr[p+1], ddpSmoothCurr[p+2], ddpSmoothCurr[p+3]);
    }
    if (useMainSegmentOnly) strip.trigger();
    else                    strip.show();
    ddpSmoothRenderedFrames++;
    return true;
  }

  if (now - ddpSmoothLastTick < DDP_SMOOTH_TICK_MS) return false; // not a tick boundary yet
  uint32_t elapsed = now - ddpSmoothLastTick;
  if (elapsed > DDP_SMOOTH_TICK_MS && elapsed < 10000u) { // loop could not keep up with the tick cadence
    ddpSmoothSkippedLevels += elapsed / DDP_SMOOTH_TICK_MS - 1;
  }
  ddpSmoothLastTick = now;
  ddpSmoothRenderDecay();
  ddpSmoothRenderedFrames++;
  ddpSmoothDebugStats();
  return true;
}

// Periodically print smoothing statistics (compiled out unless WLED_DEBUG).
// Informs how many frames were shown and how many 20 ms ticks had to be skipped
// because the main loop / LED bus could not keep up with the tick cadence.
static void ddpSmoothDebugStats() {
  uint32_t now = millis();
  if (now - ddpSmoothStatsLastMs < 2000) return;
  ddpSmoothStatsLastMs = now;
  #ifdef WLED_DEBUG
  DEBUG_PRINTF_P(PSTR("DDP smooth: %lu frames rendered, %lu ticks skipped (SMS=%u, tick=%ums)\n"),
    (unsigned long)ddpSmoothRenderedFrames, (unsigned long)ddpSmoothSkippedLevels,
    ddpSmoothingSpeed, DDP_SMOOTH_TICK_MS);
  ddpSmoothRenderedFrames = 0;
  ddpSmoothSkippedLevels  = 0;
  #endif
}

// Report smoothing diagnostic counters (readable from /json/info).
// enabled is the persisted toggle setting; active is whether smoothing is
// engaged at this instant (a frame has been shown while DDP streaming with no
// override). The rendered/skipped counters count since the previous call and are
// reset each time, so the web UI sees a live rate rather than monotonic totals.
void ddpSmoothGetStats(bool& enabled, bool& active, uint32_t& rendered, uint32_t& skipped) {
  enabled = ddpSmoothingEnabled;
  active  = ddpSmoothingEnabled && ddpSmoothHaveShown && (realtimeMode == REALTIME_MODE_DDP);
  uint32_t r = ddpSmoothRenderedFrames;
  uint32_t s = ddpSmoothSkippedLevels;
  ddpSmoothRenderedFrames = 0;
  ddpSmoothSkippedLevels  = 0;
  rendered = r;
  skipped  = s;
}

//E1.31 and Art-Net protocol support
void handleE131Packet(e131_packet_t* p, IPAddress clientIP, byte protocol, size_t packetLen){

  int uni = 0, dmxChannels = 0;
  uint8_t* e131_data = nullptr;
  int seq = 0, mde = REALTIME_MODE_E131;

  if (protocol == P_ARTNET)
  {
    if (packetLen < 10) return; // need at least art_opcode (offset 8, 2 bytes)
    if (p->art_opcode == ARTNET_OPCODE_OPPOLL) {
      handleArtnetPollReply(clientIP);
      return;
    }
    uni = p->art_universe;
    dmxChannels = htons(p->art_length);
    e131_data = p->art_data;
    seq = p->art_sequence_number;
    mde = REALTIME_MODE_ARTNET;
  } else if (protocol == P_E131) {
    // Ignore PREVIEW data (E1.31: 6.2.6)
    if ((p->options & 0x80) != 0) return;
    dmxChannels = htons(p->property_value_count) - 1; // on malformed packets, this can become negative, checked below
    // DMX level data is zero start code. Ignore everything else. (E1.11: 8.5)
    if (dmxChannels <= 0 || p->property_values[0] != 0) return;
    uni = htons(p->universe);
    e131_data = p->property_values;
    seq = p->sequence_number;
    if (e131Priority != 0) {
      if (p->priority < e131Priority ) return;
      // track highest priority & skip all lower priorities
      if (p->priority >= highPriority.get()) highPriority.set(p->priority);
      if (p->priority < highPriority.get()) return;
    }
  } else { //DDP
    realtimeIP = clientIP;
    handleDDPPacket(p, packetLen);
    return;
  }

  #ifdef WLED_ENABLE_DMX
  // does not act on out-of-order packets yet
  if (e131ProxyUniverse > 0 && uni == e131ProxyUniverse) {
    for (uint16_t i = 1; i <= dmxChannels; i++)
      dmx.write(i, e131_data[i]);
    dmx.update();
  }
  #endif

  // only listen for universes we're handling & allocated memory
  if (uni < e131Universe || uni >= (e131Universe + E131_MAX_UNIVERSE_COUNT)) return;

  unsigned previousUniverses = uni - e131Universe;

  if (e131SkipOutOfSequence)
    if (seq < e131LastSequenceNumber[previousUniverses] && seq > 20 && e131LastSequenceNumber[previousUniverses] < 250){
      DEBUG_PRINTF_P(PSTR("skipping E1.31 frame (last seq=%d, current seq=%d, universe=%d)\n"), e131LastSequenceNumber[previousUniverses], seq, uni);
      return;
    }
  e131LastSequenceNumber[previousUniverses] = seq;

  // update status info
  realtimeIP = clientIP;

  handleDMXData(uni, dmxChannels, e131_data, mde, previousUniverses);
}

void handleDMXData(uint16_t uni, uint16_t dmxChannels, uint8_t* e131_data, uint8_t mde, uint8_t previousUniverses) {
  byte wChannel = 0;
  unsigned totalLen = strip.getLengthTotal();
  unsigned availDMXLen = 0;
  unsigned dataOffset = DMXAddress;

  // For legacy DMX start address 0 the available DMX length offset is 0
  const unsigned dmxLenOffset = (DMXAddress == 0) ? 0 : 1;

  // Check if DMX start address fits in available channels
  if (dmxChannels >= DMXAddress) {
    availDMXLen = (dmxChannels - DMXAddress) + dmxLenOffset;
  }

  // DMX data in Art-Net packet starts at index 0, for E1.31 at index 1
  if (mde == REALTIME_MODE_ARTNET && dataOffset > 0) {
    dataOffset--;
  }

  switch (DMXMode) {
    case DMX_MODE_DISABLED:
      return;  // nothing to do
      break;

    case DMX_MODE_SINGLE_RGB:   // 3 channel: [R,G,B]
      if (uni != e131Universe) return;
      if (availDMXLen < 3) return;

      realtimeLock(realtimeTimeoutMs, mde);

      if (realtimeOverride) return;

      wChannel = (availDMXLen > 3) ? e131_data[dataOffset+3] : 0;
      for (unsigned i = 0; i < totalLen; i++)
        setRealtimePixel(i, e131_data[dataOffset+0], e131_data[dataOffset+1], e131_data[dataOffset+2], wChannel);
      break;

    case DMX_MODE_SINGLE_DRGB:  // 4 channel: [Dimmer,R,G,B]
      if (uni != e131Universe) return;
      if (availDMXLen < 4) return;

      realtimeLock(realtimeTimeoutMs, mde);
      if (realtimeOverride) return;
      wChannel = (availDMXLen > 4) ? e131_data[dataOffset+4] : 0;

      if (bri != e131_data[dataOffset+0]) {
        bri = e131_data[dataOffset+0];
        strip.setBrightness(bri, true);
      }

      for (unsigned i = 0; i < totalLen; i++)
        setRealtimePixel(i, e131_data[dataOffset+1], e131_data[dataOffset+2], e131_data[dataOffset+3], wChannel);
      break;

    case DMX_MODE_PRESET:       // 2 channel: [Dimmer,Preset]
      {
        if (uni != e131Universe || availDMXLen < 2) return;

        // limit max. selectable preset to 250, even though DMX max. val is 255
        int dmxValPreset = (e131_data[dataOffset+1] > 250 ? 250 : e131_data[dataOffset+1]);
        
        // only apply preset if value changed 
        if (dmxValPreset != 0 && dmxValPreset != currentPreset &&  
            // only apply preset if not in playlist, or playlist changed
            (currentPlaylist < 0 || dmxValPreset != currentPlaylist)) { 
          presetCycCurr = dmxValPreset;
          applyPreset(dmxValPreset, CALL_MODE_NOTIFICATION);
        }

        // only change brightness if value changed
        if (bri != e131_data[dataOffset]) {                                        
          bri = e131_data[dataOffset];
          strip.setBrightness(bri, false);
          stateUpdated(CALL_MODE_WS_SEND);
        }
        return;
        break;
      }

    case DMX_MODE_EFFECT:           // 15 channels [bri,effectCurrent,effectSpeed,effectIntensity,effectPalette,effectOption,R,G,B,R2,G2,B2,R3,G3,B3]
    case DMX_MODE_EFFECT_W:         // 18 channels, same as above but with extra +3 white channels [..,W,W2,W3]
    case DMX_MODE_EFFECT_SEGMENT:   // 15 channels per segment;
    case DMX_MODE_EFFECT_SEGMENT_W: // 18 Channels per segment;
      {
        if (uni != e131Universe) return;
        bool isSegmentMode = DMXMode == DMX_MODE_EFFECT_SEGMENT || DMXMode == DMX_MODE_EFFECT_SEGMENT_W;
        unsigned dmxEffectChannels = (DMXMode == DMX_MODE_EFFECT || DMXMode == DMX_MODE_EFFECT_SEGMENT) ? 15 : 18;
        for (unsigned id = 0; id < strip.getSegmentsNum(); id++) {
          Segment& seg = strip.getSegment(id);
          if (isSegmentMode)
            dataOffset = DMXAddress + id * (dmxEffectChannels + DMXSegmentSpacing);
          else
            dataOffset = DMXAddress;
          // Modify address for Art-Net data
          if (mde == REALTIME_MODE_ARTNET && dataOffset > 0)
            dataOffset--;
          // Skip out of universe addresses
          if (dataOffset > dmxChannels - dmxEffectChannels + 1)
            return;

          if (e131_data[dataOffset+1] < strip.getModeCount())
            if (e131_data[dataOffset+1] != seg.mode)      seg.setMode(   e131_data[dataOffset+1]);
          if (e131_data[dataOffset+2]   != seg.speed)     seg.speed     = e131_data[dataOffset+2];      
          if (e131_data[dataOffset+3]   != seg.intensity) seg.intensity = e131_data[dataOffset+3];
          if (e131_data[dataOffset+4]   != seg.palette)   seg.setPalette(e131_data[dataOffset+4]);

          if (bool(e131_data[dataOffset+5] & 0b00000010) != seg.reverse_y) { seg.reverse_y = bool(e131_data[dataOffset+5] & 0b00000010); }
          if (bool(e131_data[dataOffset+5] & 0b00000100) != seg.mirror_y)  { seg.mirror_y  = bool(e131_data[dataOffset+5] & 0b00000100); }
          if (bool(e131_data[dataOffset+5] & 0b00001000) != seg.transpose) { seg.transpose = bool(e131_data[dataOffset+5] & 0b00001000); }
          if ((e131_data[dataOffset+5] & 0b00110000) >> 4 != seg.map1D2D) {
            seg.map1D2D = (e131_data[dataOffset+5] & 0b00110000) >> 4;
          }
          // To maintain backwards compatibility with prior e1.31 values, reverse is fixed to mask 0x01000000
          if ((e131_data[dataOffset+5] & 0b01000000) != seg.reverse) { seg.reverse = bool(e131_data[dataOffset+5] & 0b01000000); }
          // To maintain backwards compatibility with prior e1.31 values, mirror is fixed to mask 0x10000000
          if ((e131_data[dataOffset+5] & 0b10000000) != seg.mirror) { seg.mirror = bool(e131_data[dataOffset+5] & 0b10000000); }

          uint32_t colors[3];
          byte whites[3] = {0,0,0};
          if (dmxEffectChannels == 18) {
            whites[0] = e131_data[dataOffset+15];
            whites[1] = e131_data[dataOffset+16];
            whites[2] = e131_data[dataOffset+17];
          }
          colors[0] = RGBW32(e131_data[dataOffset+ 6], e131_data[dataOffset+ 7], e131_data[dataOffset+ 8], whites[0]);
          colors[1] = RGBW32(e131_data[dataOffset+ 9], e131_data[dataOffset+10], e131_data[dataOffset+11], whites[1]);
          colors[2] = RGBW32(e131_data[dataOffset+12], e131_data[dataOffset+13], e131_data[dataOffset+14], whites[2]);
          if (colors[0] != seg.colors[0]) seg.setColor(0, colors[0]);
          if (colors[1] != seg.colors[1]) seg.setColor(1, colors[1]);
          if (colors[2] != seg.colors[2]) seg.setColor(2, colors[2]);

          // Set segment opacity or global brightness
          if (isSegmentMode) {
            if (e131_data[dataOffset] != seg.opacity) seg.setOpacity(e131_data[dataOffset]);
          } else if ( id == strip.getSegmentsNum()-1U ) {
            if (bri != e131_data[dataOffset]) {
              bri = e131_data[dataOffset];
              strip.setBrightness(bri, true);
            }
          }
        }
        return;
        break;
      }
      
    case DMX_MODE_MULTIPLE_DRGB:
    case DMX_MODE_MULTIPLE_RGB:
    case DMX_MODE_MULTIPLE_RGBW:
      {
        const bool is4Chan = (DMXMode == DMX_MODE_MULTIPLE_RGBW);
        const unsigned dmxChannelsPerLed = is4Chan ? 4 : 3;
        const unsigned ledsPerUniverse = is4Chan ? MAX_4_CH_LEDS_PER_UNIVERSE : MAX_3_CH_LEDS_PER_UNIVERSE;
        uint8_t stripBrightness = bri;
        unsigned previousLeds, dmxOffset, ledsTotal;

        if (previousUniverses == 0) {
          if (availDMXLen < 1) return;
          dmxOffset = dataOffset;
          previousLeds = 0;
          // First DMX address is dimmer in DMX_MODE_MULTIPLE_DRGB mode.
          if (DMXMode == DMX_MODE_MULTIPLE_DRGB) {
            stripBrightness = e131_data[dmxOffset++];
            ledsTotal = (availDMXLen - 1) / dmxChannelsPerLed;
          } else {
            ledsTotal = availDMXLen / dmxChannelsPerLed;
          }
        } else {
          // All subsequent universes start at the first channel.
          dmxOffset = (mde == REALTIME_MODE_ARTNET) ? 0 : 1;
          const unsigned dimmerOffset = (DMXMode == DMX_MODE_MULTIPLE_DRGB) ? 1 : 0;
          unsigned ledsInFirstUniverse = (((MAX_CHANNELS_PER_UNIVERSE - DMXAddress) + dmxLenOffset) - dimmerOffset) / dmxChannelsPerLed;
          previousLeds = ledsInFirstUniverse + (previousUniverses - 1) * ledsPerUniverse;
          ledsTotal = previousLeds + (dmxChannels / dmxChannelsPerLed);
        }

        // All LEDs already have values
        if (previousLeds >= totalLen) {
          return;
        }

        realtimeLock(realtimeTimeoutMs, mde);
        if (realtimeOverride) return;

        if (ledsTotal > totalLen) {
          ledsTotal = totalLen;
        }

        if (DMXMode == DMX_MODE_MULTIPLE_DRGB && previousUniverses == 0) {
          if (bri != stripBrightness) {
            bri = stripBrightness;
            strip.setBrightness(bri, true);
          }
        }

        for (unsigned i = previousLeds; i < ledsTotal; i++) {
          setRealtimePixel(i, e131_data[dmxOffset], e131_data[dmxOffset+1], e131_data[dmxOffset+2], is4Chan ? e131_data[dmxOffset+3] : 0);
          dmxOffset += dmxChannelsPerLed;
        }
        break;
      }
    default:
      DEBUG_PRINTLN(F("unknown E1.31 DMX mode"));
      return;  // nothing to do
      break;
  }

  e131NewData = true;
}

static void handleArtnetPollReply(IPAddress ipAddress) {
  ArtPollReply artnetPollReply;
  prepareArtnetPollReply(&artnetPollReply);

  unsigned startUniverse = e131Universe;
  unsigned endUniverse = e131Universe;

  switch (DMXMode) {
    case DMX_MODE_DISABLED:
      break;

    case DMX_MODE_SINGLE_RGB:
    case DMX_MODE_SINGLE_DRGB:
    case DMX_MODE_PRESET:
    case DMX_MODE_EFFECT:
    case DMX_MODE_EFFECT_W:
    case DMX_MODE_EFFECT_SEGMENT:
    case DMX_MODE_EFFECT_SEGMENT_W:
      break;  // 1 universe is enough

    case DMX_MODE_MULTIPLE_DRGB:
    case DMX_MODE_MULTIPLE_RGB:
    case DMX_MODE_MULTIPLE_RGBW:
      {
        bool is4Chan = (DMXMode == DMX_MODE_MULTIPLE_RGBW);
        const unsigned dmxChannelsPerLed = is4Chan ? 4 : 3;
        const unsigned dimmerOffset = (DMXMode == DMX_MODE_MULTIPLE_DRGB) ? 1 : 0;
        const unsigned dmxLenOffset = (DMXAddress == 0) ? 0 : 1; // For legacy DMX start address 0
        const unsigned ledsInFirstUniverse = (((MAX_CHANNELS_PER_UNIVERSE - DMXAddress) + dmxLenOffset) - dimmerOffset) / dmxChannelsPerLed;
        const unsigned totalLen = strip.getLengthTotal();

        if (totalLen > ledsInFirstUniverse) {
          const unsigned ledsPerUniverse = is4Chan ? MAX_4_CH_LEDS_PER_UNIVERSE : MAX_3_CH_LEDS_PER_UNIVERSE;
          const unsigned remainLED = totalLen - ledsInFirstUniverse;

          endUniverse += (remainLED / ledsPerUniverse);

          if ((remainLED % ledsPerUniverse) > 0) {
            endUniverse++;
          }

          if ((endUniverse - startUniverse) > E131_MAX_UNIVERSE_COUNT) {
            endUniverse = startUniverse + E131_MAX_UNIVERSE_COUNT - 1;
          }
        }
        break;
      }
    default:
      DEBUG_PRINTLN(F("unknown E1.31 DMX mode"));
      return;  // nothing to do
      break;
  }

  if (DMXMode != DMX_MODE_DISABLED) {
    for (unsigned i = startUniverse; i <= endUniverse; ++i) {
      sendArtnetPollReply(&artnetPollReply, ipAddress, i);
    }
  }

  #ifdef WLED_ENABLE_DMX
    if (e131ProxyUniverse > 0 && (DMXMode == DMX_MODE_DISABLED || (e131ProxyUniverse < startUniverse || e131ProxyUniverse > endUniverse))) {
      sendArtnetPollReply(&artnetPollReply, ipAddress, e131ProxyUniverse);
    }
  #endif
}

static void prepareArtnetPollReply(ArtPollReply *reply) {
  // Art-Net
  reply->reply_id[0] = 0x41;
  reply->reply_id[1] = 0x72;
  reply->reply_id[2] = 0x74;
  reply->reply_id[3] = 0x2d;
  reply->reply_id[4] = 0x4e;
  reply->reply_id[5] = 0x65;
  reply->reply_id[6] = 0x74;
  reply->reply_id[7] = 0x00;

  reply->reply_opcode = ARTNET_OPCODE_OPPOLLREPLY;

  IPAddress localIP = Network.localIP();
  for (unsigned i = 0; i < 4; i++) {
    reply->reply_ip[i] = localIP[i];
  }

  reply->reply_port = ARTNET_DEFAULT_PORT;

  char * numberEnd = (char*) versionString; // strtol promises not to try to edit this.
  reply->reply_version_h = (uint8_t)strtol(numberEnd, &numberEnd, 10);
  numberEnd++;
  reply->reply_version_l = (uint8_t)strtol(numberEnd, &numberEnd, 10);

  // Switch values depend on universe, set before sending
  reply->reply_net_sw = 0x00;
  reply->reply_sub_sw = 0x00;

  reply->reply_oem_h = 0x00; // TODO add assigned oem code
  reply->reply_oem_l = 0x00;

  reply->reply_ubea_ver = 0x00;

  // Indicators in Normal Mode
  // All or part of Port-Address programmed by network or Web browser
  reply->reply_status_1 = 0xE0;

  reply->reply_esta_man = 0x0000;

  strlcpy((char *)(reply->reply_short_name), serverDescription, 18);
  strlcpy((char *)(reply->reply_long_name), serverDescription, 64);

  reply->reply_node_report[0] = '\0';

  reply->reply_num_ports_h = 0x00;
  reply->reply_num_ports_l = 0x01; // One output port

  reply->reply_port_types[0] = 0x80; // Output DMX data
  reply->reply_port_types[1] = 0x00;
  reply->reply_port_types[2] = 0x00;
  reply->reply_port_types[3] = 0x00;

  // No inputs
  reply->reply_good_input[0] = 0x00;
  reply->reply_good_input[1] = 0x00;
  reply->reply_good_input[2] = 0x00;
  reply->reply_good_input[3] = 0x00;

  // One output
  reply->reply_good_output_a[0] = 0x80; // Data is being transmitted
  reply->reply_good_output_a[1] = 0x00;
  reply->reply_good_output_a[2] = 0x00;
  reply->reply_good_output_a[3] = 0x00;

  // Values depend on universe, set before sending
  reply->reply_sw_in[0] = 0x00;
  reply->reply_sw_in[1] = 0x00;
  reply->reply_sw_in[2] = 0x00;
  reply->reply_sw_in[3] = 0x00;

  // Values depend on universe, set before sending
  reply->reply_sw_out[0] = 0x00;
  reply->reply_sw_out[1] = 0x00;
  reply->reply_sw_out[2] = 0x00;
  reply->reply_sw_out[3] = 0x00;

  reply->reply_sw_video = 0x00;
  reply->reply_sw_macro = 0x00;
  reply->reply_sw_remote = 0x00;

  reply->reply_spare[0] = 0x00;
  reply->reply_spare[1] = 0x00;
  reply->reply_spare[2] = 0x00;

  // A DMX to / from Art-Net device
  reply->reply_style = 0x00;

  Network.localMAC(reply->reply_mac);

  for (unsigned i = 0; i < 4; i++) {
    reply->reply_bind_ip[i] = localIP[i];
  }

  reply->reply_bind_index = 1;

  // Product supports web browser configuration
  // Node’s IP is DHCP or manually configured
  // Node is DHCP capable
  // Node supports 15 bit Port-Address (Art-Net 3 or 4)
  // Node is able to switch between ArtNet and sACN
  reply->reply_status_2 = (multiWiFi[0].staticIP[0] == 0) ? 0x1F : 0x1D;

  // RDM is disabled
  // Output style is continuous
  reply->reply_good_output_b[0] = 0xC0;
  reply->reply_good_output_b[1] = 0xC0;
  reply->reply_good_output_b[2] = 0xC0;
  reply->reply_good_output_b[3] = 0xC0;

  // Fail-over state: Hold last state
  // Node does not support fail-over
  reply->reply_status_3 = 0x00;

  for (unsigned i = 0; i < 21; i++) {
    reply->reply_filler[i] = 0x00;
  }
}

static void sendArtnetPollReply(ArtPollReply *reply, IPAddress ipAddress, uint16_t portAddress) {
  reply->reply_net_sw = (uint8_t)((portAddress >> 8) & 0x007F);
  reply->reply_sub_sw = (uint8_t)((portAddress >> 4) & 0x000F);
  reply->reply_sw_out[0] = (uint8_t)(portAddress & 0x000F);

  snprintf_P((char *)reply->reply_node_report, sizeof(reply->reply_node_report)-1, PSTR("#0001 [%04u] OK - WLED v%s"), pollReplyCount, versionString);

  if (pollReplyCount < 9999) {
    pollReplyCount++;
  } else {
    pollReplyCount = 0;
  }

  notifierUdp.beginPacket(ipAddress, ARTNET_DEFAULT_PORT);
  notifierUdp.write(reply->raw, sizeof(ArtPollReply));
  notifierUdp.endPacket();

  reply->reply_bind_index++;
}