#pragma once
#include "TankDrive.h"
#include "DriveActions.h"
#include "HexaLineArray.h"
#include "HexaPID.h"
#include "LineTuning.h"
#include "NRLAction.h"
#include <functional>
#include <vector>

// ============================================================
//  LineActions — line-following building blocks for any arena
// ============================================================
//  Wraps a TankDrive and a HexaLineArray into named, chainable ActionPtrs,
//  the same way DriveActions does for timed drives and gyro turns. Each one
//  follows the line (PID steering) until something happens, or turns onto a
//  line, and plugs straight into runAction()/runBlocking().
//
//    static TankDrive     drive(leftMotor, rightMotor);
//    static HexaLineArray line{ hexaLineConfig() };
//    static LineActions   lineActions(drive, line);
//    ...
//    runAction(lineActions.auto_()
//        .leaveStart()                           // out of the grey START box
//        .followToJunction(LineSide::RIGHT, 2)   // stop at the 2nd right branch
//        .right()                                // turn onto it
//        .followToEnd()                          // run to the end of that branch
//        .then(grab)                             // your own action
//        .uTurn()
//        .followToJunction()                     // back to the junction
//        .right()
//        .followToGrey()                         // into the grey FINISH box
//        .park()
//        .build());
//
//  TUNING. Every number LineActions uses — steering kP / kI / kD, speeds,
//  turn timings — lives in LineTuning (LineTuning.h says what each one does
//  and how to tune it). The defaults are tuned for the NRL robot; to change
//  some, keep them together at the top of your OpMode and pass them in:
//
//    static LineTuning myTuning() {
//        LineTuning t;
//        t.pid.kP      = 0.50f;   // steering strength
//        t.pid.kI      = 0.0f;    // leave 0 for line following
//        t.pid.kD      = 0.04f;   // damping after turns
//        t.followSpeed = 0.35f;   // forward speed on a straight
//        return t;
//    }
//    static LineActions lineActions(drive, line, myTuning());
//
//  JUNCTIONS AND CORNERS. A junction starts when an EDGE pair of the bar
//  (ch0+ch1 or ch6+ch7) and its MIDDLE (ch3 or ch4) read dark together for
//  tuning.confirmTicks: the tape runs from the middle of the bar out to its
//  end. A single line that has drifted toward one end, or wobbles after a
//  turn, darkens ch0..ch2 at most, never both. The robot then CROSSES the tape straight
//  (no steering, so a branch does not drag it off) until both edges clear,
//  and looks whether the line carries on ahead:
//    - a branch on ONE side and nothing ahead is a CORNER. The robot turns
//      it by itself and keeps going; corners are never counted, never stop
//      a step, and need no code.
//    - anything else is a JUNCTION: counted, recorded in lastJunction()
//      (left / right / straight), and — if it is the one a step waits
//      for — the robot brakes and stops with the bar just past the tape.
//  A T or a bar across the line counts as both left and right.
//
//  TURNING. left()/right() and corners (after braking to a stop) first roll
//  forward tuning.pivotMs so the middle of the chassis (where a skid-steer
//  spins) sits on the junction. Then spin: the centre pair (ch3/ch4) must
//  leave the old line, the gyro must reach about 90° (180° for a U-turn), and
//  the turn ends the moment ch3 or ch4 reads black — then a short counter-
//  brake, and a square-up to the grid line with the gyro.
//
//  GRID HEADING. On an arena of straight lines at right angles, every line
//  runs at 0°, 90°, 180° or 270° from the way the robot faced when the run
//  started. The first step of a run (after clearRoute()) records that with
//  the gyro — so start the robot square to the line. From then on:
//    - turns aim for the next grid direction, not "90° from wherever the
//      spin began", so a robot that came in crooked still ends square;
//    - steady following on a straight trims the gyro's slow drift.
//  Which way the gyro counts is assumed clockwise-positive (DriveActions'
//  convention) and checked by every real turn, so a board mounted the other
//  way round is corrected on its first turn.
//  tuning.useGrid = false turns it off (curved or diagonal lines).
//
//  LINE END vs SLIPPED OFF. If the line vanishes near the centre of the bar
//  it has ENDED. If it vanishes off one edge, the robot was on a curve: it
//  pivots back toward the side it last saw the line (tuning.searchSpeed).
//
//  FAULTS. When something goes wrong — the line is lost for good, a turn
//  finds no line, the line ends where a junction was expected — the robot
//  stops, fault() says why, and every LATER step built with auto_() is
//  skipped (your then() actions too), so a routine never carries on blind.
//  Show faultName() in telemetry; clearFault() before starting again.
//
//  ROUTE MEMORY. Every junction the robot crosses or stops at, and which way
//  it left it, is remembered automatically as the path back to where it
//  started (corners need nothing — they turn themselves both ways). Going
//  back the same way pops junctions off again, so the memory is always "how
//  to get home from here", on any layout. Use it to return to a junction
//  without counting by hand:
//
//    static int p2;
//    runAction(lineActions.auto_()
//        .followToJunction(LineSide::RIGHT, 2)
//        .markHere(&p2)            // remember this junction
//        .right().followToEnd()
//        .then(grab)
//        .uTurn()
//        .retraceTo(&p2)           // back to that junction, stopped on it
//        .right()                  // carry on from there, e.g. on to the drop
//        .build());
//
//  retraceHome() goes all the way back past the first junction. Clear the
//  memory with clearRoute() at the start of a run.
//
//  BLACK BOXES. A grey START/FINISH box uses leaveStart() / followToGrey().
//  A BLACK one reads as line on every channel, so it has its own steps:
//
//    .leaveBox()        // straight out until only the centre sees the line
//    ...
//    .followToBox()     // all 8 channels black long enough = the box: brake
//    .park(400)         // then 400 ms on into it (negative = reverse)
//
//  Needs a calibrated line array (CALIBRATE > LINE TEST). Every method body
//  lives in LineActions.cpp (the prebuilt .a).

/** Which side a junction branch must be on. A T or a bar counts as both. */
enum class LineSide : uint8_t { ANY, LEFT, RIGHT };

/** Why a line action stopped the routine. */
enum class LineFault : uint8_t {
    NONE,
    NOT_CALIBRATED,   ///< the array has no calibration — run LINE TEST
    LINE_LOST,        ///< slipped off the line and the search never found it
    UNEXPECTED_END,   ///< the line ended while a junction or grey was expected
    TURN_TIMEOUT,     ///< a turn / U-turn / search found no line in time
    NO_GREY,          ///< followToGrey(): no grey area after the line ended
    NO_LINE,          ///< leaveStart()/leaveBox(): no line found in front of the start box
    WRONG_JUNCTION,   ///< retrace: the junction reached has no exit the way back needs
    NO_ROUTE,         ///< retrace: not heading back, or that mark is not on the way back
    NO_BRANCH         ///< left()/right() on a junction that has no branch that way
};

/** Human-readable fault name — for telemetry. */
const char* lineFaultName(LineFault f);

/** Which ways the last junction went (read once the robot has crossed it). */
struct LineJunction {
    bool left     = false;   ///< a branch to the left
    bool right    = false;   ///< a branch to the right
    bool straight = false;   ///< the line carries on ahead (crossroads, not T)
};

class LineActions {
public:
    LineActions(TankDrive& drive, HexaLineArray& line, LineTuning tuning = LineTuning{})
        : _drive(drive), _line(line), _driveActs(drive), _tuning(tuning), _pid(tuning.pid) {}

    void setTuning(const LineTuning& t);
    const LineTuning& tuning() const { return _tuning; }

    // ── Following ──────────────────────────────────────────────
    /**
     * Drive straight out of a (grey) start box until the line is under the
     * bar. Faults NO_LINE after tuning.startTimeoutMs.
     */
    ActionPtr leaveStart();
    /**
     * Drive straight out of a BLACK start box until the bar is on the line
     * leaving it: ch3 or ch4 black, ch0/ch1/ch6/ch7 all white. (leaveStart()
     * would stop at once — the box itself reads as line.) Faults NO_LINE
     * after tuning.startTimeoutMs.
     */
    ActionPtr leaveBox();
    /** Follow to the count-th junction on `side`, cross it, and stop. Corners don't count. */
    ActionPtr followToJunction(LineSide side = LineSide::ANY, int count = 1);
    /** Same, with the count worked out when the step STARTS (e.g. from a code). */
    ActionPtr followToJunction(LineSide side, std::function<int()> count);
    /** Follow, passing any junctions, until the line ends (or a bar across it). Stops there. */
    ActionPtr followToEnd();
    /** Follow until the array is on a grey area (needs the LINE TEST grey step). */
    ActionPtr followToGrey();
    /**
     * Follow, passing junctions, into a BLACK box: all 8 channels black for
     * tuning.boxConfirmMs (longer than crossing a tape). Brakes and stops just
     * inside its edge — chain park(ms) to drive on into the box.
     */
    ActionPtr followToBox();
    /** Follow for a fixed time, passing junctions. */
    ActionPtr followFor(unsigned long ms);
    /** Follow, passing junctions, until your condition is true. */
    ActionPtr followUntil(std::function<bool()> cond);
    /**
     * Follow for `ms`, counting marks (junction-like ticks) on `side`, and
     * store the count in *out. Use it to read a code printed beside the line.
     */
    ActionPtr countMarks(LineSide side, unsigned long ms, int* out);

    // ── Turning and moving ─────────────────────────────────────
    /**
     * Turn left onto a branch: roll forward tuning.pivotMs so the chassis
     * pivot is on the junction, then spin until a line is centred under the
     * bar. Call it where followToJunction() stopped.
     */
    ActionPtr left();
    /** Same as left(), to the right. */
    ActionPtr right();
    /** Drive straight on for tuning.straightMs (e.g. across a junction). */
    ActionPtr straight();
    /** Spin round (right) until a line is centred again — at least ~90° first. */
    ActionPtr uTurn();
    /** Pivot toward the side the line was last seen until it is centred. */
    ActionPtr searchLine();
    /** Closed-loop straight drive off the line (DriveActions::driveInches). */
    ActionPtr driveInches(float inches);
    /** Drive tuning.parkMs further into the grey area, then stop. */
    ActionPtr park();
    /**
     * Drive straight for |ms| at tuning.parkSpeed, then stop — forward, or in
     * reverse if ms is negative (backing up after an overshoot). No line
     * needed: for the last stretch into a box after followToBox().
     */
    ActionPtr park(long ms);
    ActionPtr stop();

    // ── Route memory (see ROUTE MEMORY above) ──────────────────
    /**
     * A mark for the junction the robot is stopped at (or, before any
     * junction, 0 for the start). Pass it to retraceTo() later. It names
     * that junction itself: once the robot has gone back past it, the mark
     * is used up and retraceTo() faults NO_ROUTE rather than stop elsewhere.
     */
    int          mark() const {
        // Heading back between junctions the top of the route is the junction
        // AHEAD — not a place the robot has been to, so no mark (-1, which
        // retraceTo() refuses with NO_ROUTE). Mark while stopped on it.
        if (_routeBack && !_routePending) return -1;
        return _route.empty() ? 0 : _route.back().id;
    }
    /**
     * Step form of mark(): stores it in *out when the step runs — later, not
     * when the routine is built, so `out` must outlive the routine: use a
     * static or global int (or an OpMode member), never a local.
     */
    ActionPtr    markHere(int* out);
    /**
     * Follow back along the remembered route — taking each junction the way
     * that leads home — and stop at the junction *mark names. Call it once
     * the robot is heading back (after uTurn()).
     */
    ActionPtr    retraceTo(const int* mark);   // *mark is read when the step starts — same lifetime rule
    /**
     * Same, all the way back: leaves the first junction toward the start and
     * stops there. Chain followToEnd() / followToGrey() to finish the way in.
     */
    ActionPtr    retraceHome();
    /** Junctions between the robot and the start along its route. */
    int          routeDepth() const { return (int)_route.size(); }
    /** True while the robot is heading back toward the start. */
    bool         headingBack() const { return _routeBack; }
    /**
     * Forget the route — the robot's position becomes the new start. The
     * grid heading is set again by the next step, so call it with the robot
     * square to the line (in the start box).
     */
    void         clearRoute() { _route.clear(); _routeBack = false; _routePending = false; _gridSet = false; }

    // ── State, for telemetry and decisions ─────────────────────
    LineFault    fault()      const { return _fault; }
    const char*  faultName()  const { return lineFaultName(_fault); }
    /**
     * Clears the fault only. The route memory (and "stopped on a junction")
     * is kept, so after NO_BRANCH the routine can still choose another way.
     * If the robot was moved by hand, call clearRoute() too.
     */
    void         clearFault()       { _fault = LineFault::NONE; }
    /** Junctions confirmed since the last resetCounts(), passed or stopped at. */
    int          junctionCount() const { return _junctions; }
    /** Corners turned automatically since the last resetCounts(). */
    int          cornerCount()   const { return _corners; }
    const LineJunction& lastJunction() const { return _lastJunction; }
    /** How far the last turn or corner rotated, in degrees (gyro); -1 without a gyro. */
    float        lastTurnDegrees() const { return _lastTurnDeg; }
    /**
     * The robot's heading on the grid, degrees clockwise from the way it faced
     * at the start of the run (-180..180; lines run at 0, ±90, 180). -999
     * until the grid is set. See GRID HEADING above.
     */
    float        gridHeading() const;
    void         resetCounts()        { _junctions = 0; _corners = 0; }

    // ---- Fluent builder ----------------------------------------
    // Every step it adds is skipped once a fault is set (see FAULTS above).
    class Builder {
    public:
        explicit Builder(LineActions& owner) : _owner(owner) {}

        Builder& leaveStart();
        Builder& leaveBox();
        Builder& followToJunction(LineSide side = LineSide::ANY, int count = 1);
        Builder& followToJunction(LineSide side, std::function<int()> count);
        Builder& followToEnd();
        Builder& followToGrey();
        Builder& followToBox();
        Builder& followFor(unsigned long ms);
        Builder& followUntil(std::function<bool()> cond);
        Builder& countMarks(LineSide side, unsigned long ms, int* out);
        Builder& left();
        Builder& right();
        Builder& straight();
        Builder& uTurn();
        Builder& searchLine();
        Builder& driveInches(float inches);
        Builder& park();
        Builder& park(long ms);
        Builder& stop();
        Builder& markHere(int* out);
        Builder& retraceTo(const int* mark);
        Builder& retraceHome();
        Builder& sleepMs(unsigned long ms);
        /** Your own action (grab, release, a servo move …), in sequence. */
        Builder& then(ActionPtr action);

        ActionPtr build();

    private:
        LineActions&           _owner;
        std::vector<ActionPtr> _steps;
        void _add(ActionPtr a);
    };

    Builder auto_();

private:
    friend struct LineActionsImpl;   // the step classes, in LineActions.cpp

    TankDrive&     _drive;
    HexaLineArray& _line;
    DriveActions   _driveActs;
    LineTuning     _tuning;
    HexaPID        _pid;
    LineFault      _fault     = LineFault::NONE;
    int            _junctions = 0;
    int            _corners   = 0;
    LineJunction   _lastJunction;
    float          _lastTurnDeg = -1.0f;
    void _setFault(LineFault f);

    // Grid heading: every line runs at a multiple of 90° from the way the
    // robot faced when the run started.
    bool   _gridSet  = false;
    float  _gridZero = 0.0f;   // raw gyro heading at grid 0°
    int8_t _gyroSign = 1;      // +1 if the raw heading grows clockwise; each turn checks it
    void   _gridMark();              // set grid 0° here, if not set yet
    bool   _gridReady() const;       // set, gyro calibrated
    float  _gridAngle() const;       // clockwise from grid 0°, -180..180
    float  _gridOff() const;         // clockwise off the nearest grid line, -45..45

    // Route memory: one entry per junction on the path from the start.
    // Directions are quarter turns relative to the way the robot entered it
    // from the start side: 0 straight, 1 right, 2 back, 3 left.
    struct RouteJunction {
        uint8_t  exit;    // the way it left
        uint8_t  ways;    // bit per direction: the lines the junction has
        uint16_t id;      // what mark() hands out
    };
    std::vector<RouteJunction> _route;
    uint16_t _routeIds     = 0;
    bool     _routeBack    = false;   // heading back toward the start
    bool     _routePending = false;   // stopped on a junction, exit not chosen yet
    void _routeArrive(bool stopping);
    void _routeLeave(uint8_t turn);   // 0 straight, 1 right, 2 back, 3 left
    void _routeUTurn();
    bool _routeBlocked(uint8_t turn) const;   // stopped on a junction with no line that way
    bool _routeMatches() const;               // heading back: the junction reached is the top one
};
