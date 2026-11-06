#pragma once
#include <stdint.h>

// ============================================================
//  HexaPID — a PID controller you declare once and call every tick
// ============================================================
//  Turns an ERROR (how far off you are) into a CORRECTION (how hard to
//  steer back). Use it for line following, holding a heading, holding an
//  arm angle — anything where you measure, compare to a target and push
//  toward it.
//
//    P  pushes in proportion to the error right now.
//    I  adds up error over time, to remove a small steady offset.
//    D  reacts to how fast the error is changing, to damp overshoot.
//
//  Declare-then-call, like every Hexa object:
//
//    static HexaPID steer{{ .kP = 0.6f, .kD = 0.05f }};
//    ...
//    void start() override { steer.reset(); }          // fresh start each run
//    void loop()  override {
//        float err  = line.error() / 3500.0f;           // -1 .. +1
//        float turn = steer.update(err);                // -1 .. +1 by default
//        drive.drive(0.4f, turn);
//        telemetry.addData("P", steer.p());
//        telemetry.addData("D", steer.d());
//    }
//
//  SIGN: a positive error gives a positive output when the gains are
//  positive. If the robot steers AWAY from the line, flip the sign of the
//  error (or of the gains) — don't fight it with the motors.
//
//  TIME: update(error) measures the time since the previous call itself,
//  so I and D are in "per second" units and do not change if the loop
//  runs slower. A gap longer than maxDtMs (for example a link-loss pause)
//  counts as maxDtMs, so I does not jump and D does not collapse on resume.
//
//  RESET: call reset() whenever you start controlling something new — a new
//  run, or after the robot did something else (searching, turning). The
//  first update() after reset() has no D (there is no previous error to
//  compare with), so it never kicks.
//
//  Every method body lives in HexaPID.cpp (the prebuilt .a).

struct HexaPIDConfig {
    float kP = 0.0f;
    float kI = 0.0f;           // leave 0 for line following — I mostly adds overshoot there
    float kD = 0.0f;

    float outMin = -1.0f;      // output is clamped to [outMin, outMax]
    float outMax =  1.0f;      // (-1..+1 matches TankDrive::drive()'s turn input)

    /// Limit on the I term's contribution, in OUTPUT units (0..). The I sum
    /// also stops growing while the output is pinned at a limit in the same
    /// direction (anti-windup), so it can't build up and overshoot later.
    float iMax = 0.5f;

    /// D smoothing, 0..<1. 0 = raw D. Higher = smoother but slower: each new
    /// D is (dAlpha × old) + ((1 − dAlpha) × new). Try 0.5 if D is jittery.
    float dAlpha = 0.0f;

    /// Longest gap between two update() calls that counts as real time.
    uint16_t maxDtMs = 50;
};

class HexaPID {
public:
    HexaPID() : HexaPID(HexaPIDConfig{}) {}
    explicit HexaPID(const HexaPIDConfig& config);

    /** One control step. Returns the clamped output. Measures dt itself. */
    float update(float error);

    /** One control step with target and measurement: error = target − measured. */
    float update(float target, float measured);

    /** One control step with an explicit time step in seconds (> 0). */
    float updateDt(float error, float dtSec);

    /** Forget the past: clears I, the previous error and the D filter. */
    void reset();

    // ── Tuning ───────────────────────────────────────────────────
    void setGains(float kP, float kI, float kD);
    void setOutputLimits(float outMin, float outMax);
    void setConfig(const HexaPIDConfig& config);
    const HexaPIDConfig& config() const { return m_cfg; }

    // ── Last step, for telemetry ─────────────────────────────────
    float p()      const { return m_p; }       // P contribution
    float i()      const { return m_i; }       // I contribution
    float d()      const { return m_d; }       // D contribution
    float output() const { return m_out; }     // clamped sum
    float error()  const { return m_lastErr; }

private:
    HexaPIDConfig m_cfg;
    float    m_iSum    = 0.0f;   // integral of error over time (error·s)
    float    m_lastErr = 0.0f;
    float    m_dFilt   = 0.0f;
    bool     m_primed  = false;  // false until the first step after reset()
    uint32_t m_lastUs  = 0;
    float    m_p = 0.0f, m_i = 0.0f, m_d = 0.0f, m_out = 0.0f;
};
