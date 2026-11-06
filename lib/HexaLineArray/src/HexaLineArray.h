/**
 * HexaLineArray.h — 8-channel IR reflectance array for HEXA COMMAND HUB
 *
 * Reads an 8-sensor IR line array through a 74HC4051-class 8:1 analog mux:
 * three select lines pick a channel, one ADC pin reads the mux common. Gives
 * the same "declare an object, call its methods" idiom used by HexaDCMotor/
 * HexaServo/HexaOLED/HexaIMU/HexaPower.
 *
 * ── HOW AN IR REFLECTANCE SENSOR WORKS ──────────────────────────────────
 * Each channel is an IR LED pointed at the floor next to a phototransistor.
 * The phototransistor measures HOW MUCH IR CAME BACK — it has no concept of
 * colour. White scatters most of the IR back, so white reads LOW on the ADC.
 * Black absorbs it, so the line reads HIGH.
 *
 * Because it is a reflection measurement, the number also depends on the
 * surface, the sensor's ride height, the emitter's brightness (so: battery
 * voltage), and ambient IR — sunlight through a window is the classic killer.
 * A raw count of 1800 therefore means nothing on its own. It only means
 * something relative to THIS channel's own white and black readings, and no
 * two LED/phototransistor pairs have the same brightness or gain.
 *
 * That is what calibration is for.
 *
 * ── CALIBRATION: TWO REFERENCES, IN ORDER ───────────────────────────────
 * Calibration is two deliberate measurements, not a hunt:
 *
 *   STEP 1  put ALL 8 sensors over the plain background  -> captures calMin[]
 *   STEP 2  put ALL 8 sensors over the black line        -> captures calMax[]
 *
 * Each step AVERAGES the channel across the whole capture window, so
 * electrical noise averages out instead of being baked in. read() then
 * rescales each channel onto a common 0..1000 range, so a strong channel and
 * a weak channel agree about what "on the line" means:
 *
 *     normalized = (raw - calMin) / (calMax - calMin) * 1000
 *
 * Higher = darker = more line. A channel whose span comes out below
 * cfg.minSpan is treated as dead and forced to 0 — that is how a broken
 * emitter shows itself.
 *
 * GETTING ALL 8 ONTO BLACK. A track line is much narrower than the array, so
 * for step 2 either rotate the robot ~90 degrees and let the array lie ALONG
 * the line, or hold a black card under it. Prefer the rotation: it measures
 * the real track surface, which is what the robot will actually drive on.
 *
 * ORDER MATTERS, AND IS CHECKED. If step 2 comes out no darker than step 1 on
 * most channels (calMax <= calMin) the references were captured the wrong way
 * round. Both are rejected and isCalibrated() stays false rather than letting
 * the robot drive on inverted values.
 *
 * ── OPTIONAL STEP 3: GREY ───────────────────────────────────────────────
 * Some arenas have grey areas — a START/FINISH box, a parking bay. With only
 * white and black, grey simply lands somewhere on the 0..1000 scale and is
 * read as "line" or "background". To recognise it, calibrate it:
 *
 *   STEP 3  put ALL 8 sensors on the grey surface  -> captures the grey level
 *
 * Only possible once WHITE and BLACK are done. The grey must sit clearly
 * between them (cfg.greyMinGap from both, on most channels) — the sensor
 * sees INFRARED, and some inks that look mid-grey read almost white or almost
 * black to it. If it is too close, the capture is rejected (greyRejected())
 * and that grey simply cannot be told apart by this sensor.
 *
 * onGrey() is a WHOLE-ARRAY test: it is true only when at least
 * cfg.greyMinChannels of the six inner channels all read the grey level and
 * no channel reads hard black (cfg.greyHardLine). A sensor half over
 * the black line also reads mid-scale, so one or two grey-looking channels
 * mean nothing — that is why grey never changes mask(), onLine(), position()
 * or pattern(). On a grey area pattern() may well say TERMINAL (it all looks
 * darker than the threshold); check onGrey() first.
 *
 * Re-running STEP 1 (WHITE) discards the grey too: a new white/black pair
 * means a new venue or surface, so the grey has to be captured again.
 *
 * Calibration is stored in NVS (Preferences namespace "nrl") and SURVIVES
 * power-off, exactly like the paired controller MAC. It is not erased by
 * reflashing firmware. Recalibrate when the venue, lighting, track surface,
 * or sensor mounting height changes.
 *
 * It is only SAVED while no OpMode is running — in practice, from the
 * controller's LINE TEST screen. A capture finished during a run still
 * calibrates the array for that run, but is not written to flash.
 *
 * ── LINE POSITION ───────────────────────────────────────────────────────
 * position() is a weighted centroid over the normalized channels, reported
 * as 0..7000 where each 1000 is one sensor spacing and 3500 is dead centre:
 *
 *     position = Sum(norm[i] * i * 1000) / Sum(norm[i])
 *
 * error() is position() - 3500, so it runs +/-3500 and is 0 when centred.
 * This only means "distance" if your sensors are evenly spaced, and the sign
 * is only right if channel 0 is the physically LEFT-most sensor — set
 * cfg.reversed if yours is mounted the other way round.
 *
 * CHECK onLine() BEFORE READING position(). During a short dropout position()
 * deliberately returns an extreme (0 or 7000) on the side the line was last
 * seen, so a follower steers back toward it. A follower that reads position()
 * unconditionally AND keeps a nonzero forward speed will drive away from the
 * line it just lost. Branch on onLine() first and rotate in place instead.
 *
 * ── JUNCTIONS: WHAT THIS CLASS WILL AND WON'T TELL YOU ───────────────────
 * pattern() classifies what the array sees RIGHT NOW: LOST, LINE,
 * LEFT_BRANCH, RIGHT_BRANCH, TERMINAL. It deliberately does NOT tell you what
 * kind of junction you are at, because at the instant the outer sensors hit
 * a branch a Y, a T and a crossroads are genuinely indistinguishable — they
 * all look like LEFT_BRANCH/RIGHT_BRANCH/TERMINAL. To tell them apart you must
 * drive forward a little and look again: if the line continues ahead it was
 * a Y or a cross; if it vanished it was a T and you have to turn.
 *
 * That decision, and any debouncing of pattern() over a few ticks, is yours
 * to write. This class reports what the sensor sees; your OpMode decides
 * what to do about it.
 *
 * ── WIRING (robot board defaults, see BoardPins.h) ──────────────────────
 *     S0=GPIO18  S1=GPIO15  S2=GPIO16     mux channel select (binary S2:S1:S0)
 *     SIG=GPIO17                          mux common -> ADC
 *
 * NOTE: GPIO15/16 are the same pins as MOTOR_4_DIR/MOTOR_4_PWM, which are
 * bench-test-only 4th-motor channels. Do not wire both at once.
 *
 * NOTE: GPIO17/18 are the UART1 breakout (UART1_TX/UART1_RX). The factory QC
 * self-test drives them through a loopback jumper, so unplug the array during
 * QC, and do not use Serial1 on those pins while the array is fitted.
 *
 * Part of: HexaLineArray library
 * Author:  Rush / HEXA Robotics
 */

#pragma once

#include <Arduino.h>


/** How many sensors on the array. The mux is 8:1, so this is fixed at 8. */
static constexpr uint8_t HEXA_LINE_CHANNELS = 8;

/** Centre of the position() scale — position() runs 0..7000. */
static constexpr int HEXA_LINE_CENTRE = 3500;


/**
 * What the array sees right now.
 *
 * Classification uses three zones of the mask: the outer pairs (ch0-1 and
 * ch6-7) and the two middle sensors (ch3-4). Requiring the MIDDLE to be dark
 * before calling something a branch is what stops a line that has merely
 * drifted far left from being misread as a left branch — a real branch has
 * the robot still over the main line while extra sensors light up at an edge.
 */
enum class LinePattern : uint8_t {
    LOST,          ///< nothing dark — off the line, or the line ended
    LINE,          ///< an ordinary line, possibly off-centre
    LEFT_BRANCH,   ///< line ahead AND something at the left edge
    RIGHT_BRANCH,  ///< line ahead AND something at the right edge
    TERMINAL       ///< 7 or more of 8 channels dark
};

/** Human-readable name for a pattern — handy for telemetry. */
const char* linePatternName(LinePattern p);


/** Which reference surface a capture is measuring. */
enum class LineRef : uint8_t {
    WHITE,   ///< the plain background — becomes calMin[]
    BLACK,   ///< the line itself      — becomes calMax[]
    GREY     ///< optional grey area   — after WHITE and BLACK
};


/**
 * Configuration for the mux'd IR array.
 * Defaults match the robot board — most students never change these.
 */
struct HexaLineArrayConfig {
    /// Mux select pins, in order S0, S1, S2. Channel index is binary S2:S1:S0.
    uint8_t selPins[3] = { 18, 15, 16 };

    /// Mux common output -> an ADC-capable pin.
    ///
    /// On ESP32-S3, ADC1 is GPIO1..10 and ADC2 is GPIO11..20. ADC2 is shared
    /// with the WiFi radio, and the NRL engine keeps ESP-NOW up permanently,
    /// so an ADC2 pin here can read badly or not at all. If readings look
    /// dead or noisy with the robot connected, move this to GPIO1 (DIGITAL_1,
    /// ADC1_CH0) and rewire.
    uint8_t sigPin = 17;

    /// Settle time after switching the mux before sampling, microseconds.
    uint8_t settleUs = 15;

    /// Normalized value at or above which a channel counts as "on the line".
    /// Used by mask(), count(), onLine() and pattern().
    uint16_t darkThreshold = 250;

    /// Normalized value at or above which a channel contributes to the
    /// position() centroid. Kept EQUAL to darkThreshold: a lower weight
    /// threshold sounds like it should smooth the centroid at the line edges,
    /// but in practice it lets near-background channels drag the centroid
    /// toward the middle of the array.
    uint16_t weightThreshold = 250;

    /// A calibrated channel whose max-min span is below this is treated as
    /// dead (forced to 0). Catches broken emitters, and a capture step that
    /// was done over the wrong surface.
    uint16_t minSpan = 300;

    /// Span below this is usable but poor — worth flagging on screen so a
    /// marginal channel is visible before it causes a mystery on the track.
    uint16_t weakSpan = 800;

    /// How long the line may be missing before lost() latches true and
    /// position() returns -1. Below this the array reports an extreme
    /// position on the side it last saw the line, so a follower pivots back
    /// toward it instead of giving up on a momentary dropout.
    ///
    /// Counted in TICKED time: if the framework stops ticking the array (a
    /// link-loss pause), that time does not count toward this limit.
    uint16_t lostStopMs = 1200;

    /// Grey: a channel above this (normalized) is unmistakable tape. While any
    /// channel reads it, onGrey() is false — the line beats a scuffed grey
    /// patch. It is also the top of each channel's grey band, which runs from
    /// 2/3 of the captured grey (dirty white stays out) to halfway between the
    /// grey and this value.
    uint16_t greyHardLine = 700;

    /// Grey: how far (normalized) the captured grey must be from both white
    /// (0) and black (1000) on a channel for that channel to count as usable.
    /// Arena grey reads only ~130-210 on this sensor (it sees infrared), so
    /// this is kept low.
    uint16_t greyMinGap = 80;

    /// Grey: inner channels (1..6) that must read grey at once for onGrey().
    /// Also the number of usable inner channels a grey capture needs. The two
    /// edge channels are never used for grey: they clip on black and their
    /// grey/white gap is too small to trust.
    uint8_t greyMinChannels = 5;

    /// Set true if channel 0 is the physically RIGHT-most sensor. Flips the
    /// logical channel order so position() still increases left-to-right.
    bool reversed = false;
};


class HexaLineArray {
public:
    /** Constructor — stores config, does NOT touch hardware. */
    HexaLineArray() : HexaLineArray(HexaLineArrayConfig{}) {}
    explicit HexaLineArray(const HexaLineArrayConfig& config);

    /**
     * Initialize pins and the ADC, and load any saved calibration from NVS.
     * Fast and non-blocking — safe to call from an OpMode's init().
     * Returns true if a valid calibration was loaded.
     */
    bool begin();

    /** True once begin() has run. */
    bool isReady() const { return m_ready; }

    // ── Per-tick update ────────────────────────────────────────────────

    /**
     * Scan all 8 channels and update the cached values. Called once per tick
     * by the framework (NRLRunner::tick()) — students never call this.
     * Every accessor below reads the cache, so calling them repeatedly in one
     * tick costs nothing and always gives a consistent picture.
     */
    void tick();

    /** Tick every registered array. Called by NRLRunner; not for student use. */
    static void tickAll();

    /**
     * Clear the array registry. Called by NRLRunner after the OpMode is
     * deleted so stale pointers do not linger between runs. A file-static
     * HexaLineArray re-registers on its next begin().
     */
    static void clearRegistry();

    /**
     * Allow or forbid calibration writes to NVS. NRLRunner forbids them while
     * an OpMode is loaded and allows them again when it stops — no flash write
     * may happen during a run. Not for student use.
     */
    static void allowNvsWrites(bool allowed) { s_nvsWritesAllowed = allowed; }

    // ── Raw and normalized readings ────────────────────────────────────

    /** Raw ADC counts (0..4095) for one channel, from the last tick(). */
    uint16_t readRaw(uint8_t ch) const;

    /**
     * Calibrated reading, 0..1000, higher = darker = more line.
     * Returns 0 for an uncalibrated array or a dead channel.
     */
    uint16_t read(uint8_t ch) const;

    /**
     * One bit per channel, bit 0 = channel 0, set when that channel is at or
     * above darkThreshold. Print it as binary in telemetry and you can watch
     * the line move across the array: 00011000 centred, 11111111 on a cross.
     */
    uint8_t mask() const { return m_mask; }

    /** How many channels currently see the line (popcount of mask()). */
    uint8_t count() const;

    // ── Line position ──────────────────────────────────────────────────

    /**
     * Weighted line position, 0..7000, centre 3500.
     * Returns -1 once the line has been missing for longer than lostStopMs.
     * During a shorter dropout returns 0 or 7000 — an extreme on the side the
     * line was last seen. See the onLine() warning in the header block.
     */
    int position() const { return m_position; }

    /** position() - 3500, so +/-3500 and 0 when centred. Returns 0 when lost. */
    int error() const { return (m_position < 0) ? 0 : m_position - HEXA_LINE_CENTRE; }

    /** True if any channel is at or above darkThreshold RIGHT NOW. */
    bool onLine() const { return m_onLine; }

    /** True once the line has been missing for longer than lostStopMs. */
    bool lost() const { return m_hardLost; }

    /**
     * -1 if the line was last seen left of centre, +1 right.
     * Once the line has been seen off-centre even once this never returns to
     * 0 — a follower searching for a lost line needs a side to turn toward,
     * and 0 would leave it driving straight off the track.
     */
    int lastSeenSide() const { return m_lastSeenSide; }

    /**
     * |error| at the last sighting, 0..3500. Scale a search pivot with this:
     * a line that vanished at the extreme edge needs a harder turn than one
     * that faded near centre.
     */
    int lastSeenMagnitude() const { return m_lastSeenMag; }

    /** What the array sees right now — see the junction note at the top. */
    LinePattern pattern() const { return m_pattern; }

    // ── Grey surface (needs the optional GREY calibration step) ─────────

    /**
     * True when at least cfg.greyMinChannels channels read the calibrated grey
     * level right now — the array is over a grey AREA. Always false without a
     * grey calibration. Not debounced: confirm it over a few ticks before
     * acting on it, as with pattern().
     */
    bool onGrey() const { return m_onGrey; }

    /** How many inner channels (1..6) read the grey level right now (0..6). */
    uint8_t greyCount() const { return m_greyCount; }

    /** True once a grey level has been captured and accepted (or loaded). */
    bool haveGrey() const { return m_haveGrey; }

    /**
     * True if the last GREY capture was rejected: the grey was too close to
     * white or black on too many channels to be told apart. Cleared when the
     * next capture starts.
     */
    bool greyRejected() const { return m_greyRejected; }

    /** The captured grey level for a channel, 0..1000 (0 without one). */
    uint16_t greyLevel(uint8_t ch) const;

    // ── Calibration ────────────────────────────────────────────────────

    /**
     * Capture one reference surface. Non-blocking: it averages across the
     * following tick() calls and finishes on its own after durationMs.
     *
     * Call it twice — once with LineRef::WHITE over the background, once with
     * LineRef::BLACK over the line. When both are present the pair is
     * validated and saved to NVS automatically. Optionally a third time with
     * LineRef::GREY over a grey area, once the pair is done (ignored before).
     *
     * WHITE starts a new pair and discards any previous black reference, so
     * re-running the two steps is safe: the stored calibration is only
     * replaced once the new pair validates.
     *
     * Do NOT call this from an OpMode's init() — init() runs inside the
     * ESP-NOW receive callback and never gets the ticks needed to finish.
     * Use the controller's LINE TEST screen, or a TELEOP loop().
     */
    void startCapture(LineRef ref, uint16_t durationMs = 400);

    /** True while a capture is in progress. */
    bool isCapturing() const { return m_capturing; }

    /** Which surface the in-progress capture is measuring. */
    LineRef capturingRef() const { return m_capRef; }

    /** Capture progress, 0..100. */
    uint8_t captureProgress() const;

    /** True once the white reference has been captured. */
    bool haveWhite() const { return m_haveWhite; }

    /** True once the black reference has been captured. */
    bool haveBlack() const { return m_haveBlack; }

    /**
     * True if the last completed pair was rejected because the references
     * were captured in the wrong order (black first). Cleared when the next
     * capture starts.
     */
    bool refsSwapped() const { return m_swapped; }

    /**
     * Spread (max - min) of the raw samples taken during that channel's most
     * recent capture. A flat reference surface should give a small number; a
     * spread that rivals the channel's own span means the signal is
     * electrically noisy, not that the surface varied.
     */
    uint16_t captureSpread(uint8_t ch) const;

    /** True if a usable calibration is loaded (from NVS or a fresh capture). */
    bool isCalibrated() const { return m_calibrated; }

    /** Recorded max-min span for a channel. Below cfg.minSpan means dead. */
    uint16_t span(uint8_t ch) const;

    /** True if this channel's calibration span is usable. */
    bool channelOk(uint8_t ch) const;

    /** True if usable but below cfg.weakSpan. Display only. */
    bool channelWeak(uint8_t ch) const;

    /** Forget the calibration, in RAM and in NVS. */
    void clearCalibration();

    /** Read-only access to the active configuration. */
    const HexaLineArrayConfig& config() const { return m_cfg; }

private:
    HexaLineArrayConfig m_cfg;

    // Cached scan results, in LOGICAL left-to-right order (cfg.reversed applied).
    uint16_t m_raw[HEXA_LINE_CHANNELS]  = {};
    uint16_t m_norm[HEXA_LINE_CHANNELS] = {};

    uint16_t m_calMin[HEXA_LINE_CHANNELS] = {};
    uint16_t m_calMax[HEXA_LINE_CHANNELS] = {};
    uint16_t m_calGrey[HEXA_LINE_CHANNELS] = {};   // RAW grey average, per channel

    // Capture accumulators. 32-bit sum: 4095 * any realistic sample count
    // cannot overflow it.
    uint32_t m_capSum[HEXA_LINE_CHANNELS]    = {};
    uint16_t m_capLo[HEXA_LINE_CHANNELS]     = {};
    uint16_t m_capHi[HEXA_LINE_CHANNELS]     = {};
    uint16_t m_capSpread[HEXA_LINE_CHANNELS] = {};
    uint32_t m_capCount = 0;

    uint8_t     m_mask         = 0;
    int         m_position     = HEXA_LINE_CENTRE;
    bool        m_onLine       = false;
    bool        m_hardLost     = false;
    int         m_lastSeenSide = 0;
    int         m_lastSeenMag  = 0;
    uint32_t    m_lostMs       = 0;   // ticked time with no line (capped per tick)
    uint32_t    m_lastTickMs   = 0;
    LinePattern m_pattern      = LinePattern::LOST;
    bool        m_onGrey       = false;
    uint8_t     m_greyCount    = 0;

    bool     m_ready      = false;
    bool     m_registered = false;
    bool     m_calibrated = false;
    bool     m_capturing  = false;
    bool     m_haveWhite  = false;
    bool     m_haveBlack  = false;
    bool     m_swapped    = false;
    bool     m_haveGrey     = false;
    bool     m_greyRejected = false;
    LineRef  m_capRef     = LineRef::WHITE;
    uint32_t m_capStartMs = 0;
    uint16_t m_capDurMs   = 0;

    uint16_t    _readChannel(uint8_t muxCh) const;
    void        _normalize();
    void        _computePosition(uint32_t dtMs);
    LinePattern _classify() const;
    void        _finishCapture();
    void        _commitPair();
    void        _commitGrey();
    void        _classifyGrey();
    int         _normOf(uint8_t ch, uint16_t raw) const;
    bool        _loadCalibration();
    bool        _saveCalibration();
    void        _register();

    static constexpr uint8_t MAX_ARRAYS = 4;
    static HexaLineArray* s_registry[MAX_ARRAYS];
    static uint8_t        s_count;
    static bool           s_nvsWritesAllowed;
};
