#pragma once
#include <stdint.h>
#include "HexaPID.h"

// ============================================================
//  LineTuning — tunable constants for LineActions
// ============================================================
//  Speeds are fractions of full power (0..1), the same scale as
//  TankDrive::drive(). Times are milliseconds. Tick counts are robot
//  ticks (10 ms each at 100 Hz). Line readings are normalized 0..1000
//  (0 = white, 1000 = black), errors 0..3500.
//
//  Override per robot with LineActions::setTuning() — don't edit the
//  defaults here:
//    LineTuning t;
//    t.followSpeed = 0.45f;
//    t.pid.kP      = 0.7f;
//    lineActions.setTuning(t);
//
//  MOTOR FLOOR. A skid-steer chassis does not move at all below some PWM —
//  set that per motor with HexaDCMotorConfig::minDrive (about 30 of 255 on
//  the NRL robot), not here. turnSlowSpeed must stay above the speed at which
//  the chassis actually starts to rotate on the spot.
struct LineTuning {
    // ── Following ──────────────────────────────────────────────
    /// Steering PID. Its input is how far the line is off centre, −1..+1
    /// (line.error() / 3500); its output is how hard to turn, −1..+1.
    ///   kP — turn this hard per unit of error. Too low: drifts wide and
    ///        loses the line on bends. Too high: zig-zags on a straight.
    ///   kI — leave 0 for line following (it only adds overshoot).
    ///   kD — damping: resists the line moving across the bar. Raise it if
    ///        the robot zig-zags after a turn; too high and it shudders.
    ///   dAlpha — smooths kD, 0..0.9. Try 0.5 if the robot twitches.
    /// Tuning order: kD = 0, raise kP until it zig-zags, back off a third,
    /// then raise kD until the zig-zag dies out.
    HexaPIDConfig pid{ .kP = 0.50f, .kI = 0.0f, .kD = 0.04f };
    float  followSpeed   = 0.35f;  // forward speed with the line centred
    float  junctionSpeed = 0.25f;  // forward speed while crossing a junction (no steering)
    int8_t steerSign     = 1;      // -1 if the robot steers AWAY from the line
    float  rampRate      = 0.25f;  // TankDrive ramp per tick while a line action runs

    /// Slow down as the line moves off centre:
    ///   speed = followSpeed × (1 − speedDrop × |error|/3500), never below
    ///   minSpeedFrac × followSpeed.
    float    speedDrop     = 0.70f;
    float    minSpeedFrac  = 0.25f;
    /// Line this far off centre (or more) = about to lose it on a curve: cap
    /// the speed at edgeBrakeFrac × followSpeed.
    uint16_t edgeBrakeErr  = 1800;
    float    edgeBrakeFrac = 0.35f;

    // ── Junctions and corners ──────────────────────────────────
    /// A channel reads "dark" for junction detection above this — stricter
    /// than the array's own darkThreshold, so a line drifting toward one end
    /// of the bar is not mistaken for a branch.
    uint16_t eventThreshold = 500;
    /// A junction starts when an EDGE pair (ch0+ch1 or ch6+ch7) and the
    /// MIDDLE (ch3 or ch4) are dark together this many ticks in a row.
    uint8_t  confirmTicks   = 2;
    /// The robot then crosses the tape straight (no steering) until both edge
    /// pairs read clear this many ticks in a row — or crossMaxMs passes.
    uint8_t  releaseTicks   = 3;
    /// Still on the tape after this long = the robot is running along it (it
    /// met a corner at an angle): a one-sided one is turned as a corner.
    uint16_t crossMaxMs     = 600;
    /// While the line spreads toward an end of the bar (a junction or corner
    /// arriving), steering pauses — at most this many ticks, so a sharp curve
    /// that only looks like that for a moment is still steered round.
    uint8_t  suspectMaxTicks = 6;
    /// Once across: a branch on ONE side with no line ahead is a CORNER — the
    /// robot turns it by itself and carries on (it is not counted). Anything
    /// else is a JUNCTION. After a corner, a turn, or the start of a step,
    /// junctions are ignored until both edge pairs have read clear this many
    /// ticks in a row: the tape just handled has left the bar. Nothing longer
    /// — a junction can come right after a corner, and a robot still wobbling
    /// cannot fake one (the middle must be dark too, see confirmTicks).
    uint8_t  rearmTicks     = 8;
    /// Stopping at a junction: a short reverse pulse cancels the momentum,
    /// because stop() lets the motors coast. 0 = no pulse. Raise it if the
    /// robot still rolls on; lower it if it rocks backwards.
    uint16_t brakeMs    = 80;
    float    brakeSpeed = 0.30f;
    /// Corners: after the brake, stand still this long before the pivot roll.
    /// 0 = roll straight on from the brake: on the NRL robot corners then
    /// read ~110° (lastTurnDegrees) and a junction 20 cm after the corner
    /// was found 10 out of 10. 150 made the chassis stop short (~69°): it
    /// left the corner 20° off the line and drove past that junction.
    uint16_t cornerSettleMs = 0;
    /// How long straight() drives — enough for the bar to clear a junction.
    uint16_t straightMs = 200;

    // ── Turns (left / right / corners / U-turn / search) ───────
    /// left()/right() and corners first drive straight on this long, so the
    /// robot's PIVOT (the middle of the chassis on a 4-wheel skid-steer) —
    /// not the sensor bar — sits on the junction. Spinning then sweeps the
    /// bar across the branch at 90°. Counted from where a junction stop
    /// leaves the robot (bar just past the tape). Turns land short → raise;
    /// past the branch → lower.
    uint16_t pivotMs       = 60;     // bench-tuned on the NRL robot, 2026-10-28
    float    pivotSpeed    = 0.25f;
    float    turnSpeed     = 0.45f;  // spin speed while looking for the line
    float    turnSlowSpeed = 0.30f;  // spin speed from the gyro window on
    uint16_t turnMinMs     = 200;    // spin at least this long before accepting a line
    uint16_t uTurnMinMs    = 450;    // same for a U-turn (must sweep past ~90°)
    /// A turn reads ONLY the centre pair, ch3 and ch4, at eventThreshold:
    /// first they must leave the line it starts on (both clear this long),
    /// then the turn ends the moment ch3 or ch4 reads black.
    uint16_t leaveLineMs   = 25;
    uint16_t turnTimeoutMs = 3000;   // no line found by then → fault
    /// GYRO. Turns are driven by the robot's heading (the on-board IMU, the
    /// same one DriveActions turns use) once it is calibrated:
    ///   1. spin at turnSpeed until the gyro reads the turn's angle — 90° for
    ///      left/right/corners, 180° for a U-turn — less gyroEarlyDeg. Lines
    ///      seen before that are ignored. The allowance is wide on purpose:
    ///      a chassis that stopped a little short of the junction meets the
    ///      new line early (60–85° measured), and a line skipped there is
    ///      lost. Nothing else lies that close to 90° (or to 180°: a T's arms
    ///      pass at ~45°);
    ///   2. then keep turning at turnSlowSpeed, and stop the moment ch3 or
    ///      ch4 reads black;
    ///   3. no line by gyroLateDeg past the angle → TURN_TIMEOUT, not a spin.
    /// If turns overshoot to the timeout, the line was passed before step 2:
    /// the chassis is not pivoting on the junction — tune pivotMs.
    /// Without a calibrated gyro, turnMinMs / uTurnMinMs are used instead.
    bool     useGyro       = true;
    uint8_t  gyroEarlyDeg  = 30;
    uint8_t  gyroLateDeg   = 60;
    /// GRID (see LineActions.h): turns aim for the next grid direction.
    /// Off for curved/diagonal lines.
    bool     useGrid       = true;
    /// Drift trim: while following a straight steadily (line within
    /// gridTrustErr of centre, heading within gridTrustDeg of a grid line),
    /// pull grid 0° toward the robot's heading by this fraction per tick.
    float    gridTrimRate  = 0.005f;
    uint8_t  gridTrustDeg  = 15;
    uint16_t gridTrustErr  = 400;
    /// After the catch, on the grid, SQUARE UP: if the chassis points more
    /// than squareTolDeg off the grid line, turn on the spot (turnSlowSpeed)
    /// until within squareDoneDeg — stopping early if the line leaves the
    /// bar, or after squareMaxMs. The ch3/ch4 catch still decides the turn;
    /// this only straightens the robot on it before the next junction.
    uint8_t  squareTolDeg  = 6;
    uint8_t  squareDoneDeg = 2;
    uint16_t squareMaxMs   = 800;
    /// Right after the catch, a short counter-spin (at brakeSpeed) stops the
    /// chassis coasting past the line it just found. 0 = no pulse.
    uint16_t turnBrakeMs   = 80;
    /// Without a gyro, uTurn() reverses this long before spinning, so the spin
    /// doesn't stop on the bar it is parked at. With the gyro it spins on the
    /// spot: the U-turn ignores every line (that bar, the arms of a T) until
    /// it has turned ~150°.
    uint16_t uTurnBackupMs = 300;
    float    searchSpeed   = 0.35f;  // pivot speed when the line slips off one side

    // ── Line end ───────────────────────────────────────────────
    /// The line counts as ENDED (not slipped off a curve) when it vanished
    /// within this |error| of centre (0..3500)...
    uint16_t endCentreMag    = 1500;
    /// ...and has stayed gone this many ticks.
    uint8_t  endConfirmTicks = 4;

    // ── Start box (leaveStart) ─────────────────────────────────
    float    startSpeed     = 0.35f;  // straight out of the start box
    uint16_t startTimeoutMs = 4000;   // no line by then → NO_LINE fault
    /// A channel above this is unmistakable tape (grey never reads this dark).
    uint16_t hardLine       = 700;

    // ── Black boxes (leaveBox / followToBox) ──────────────────
    /// followToBox(): all 8 channels above eventThreshold for this long = the
    /// box. It must be longer than the bar takes to cross a tape at
    /// junctionSpeed (a stem meeting the loop darkens all 8 too, briefly).
    /// The robot stops this much further into the box — set park(ms) after.
    uint16_t boxConfirmMs   = 150;
    /// leaveBox(): the centre on the line, all four end channels white, this
    /// many ticks in a row. Drives at startSpeed, faults after startTimeoutMs.
    uint8_t  boxLeaveTicks  = 2;

    // ── Grey areas (need the GREY step in LINE TEST) ───────────
    uint8_t  greyConfirmTicks = 3;     // onGrey() this many ticks in a row
    /// followToGrey() ignores grey this long after it starts (e.g. while
    /// still leaving a grey start box).
    uint16_t greyArmMs        = 1000;
    /// Grey only counts if the line vanished within this |error| of centre —
    /// the finish box is driven into straight, a corner is left off an edge.
    uint16_t greyCentreMag    = 1200;
    uint16_t greySeekMs       = 800;   // after the line ends, drive on this long looking for grey
    uint16_t parkMs           = 300;   // park(): drive this long into the grey area, then stop
    float    parkSpeed        = 0.25f;
};
