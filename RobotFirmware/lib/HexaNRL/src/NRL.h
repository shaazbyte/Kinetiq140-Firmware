#pragma once

// ============================================================
//  NRL.h — student umbrella include
// ============================================================
//  One line gives you everything you need:
//
//    #include "NRL.h"
//
//  Declare your hardware at file-scope (above your class),
//  initialise in init(), then use in loop():
//
//    static HexaDCMotor leftMotor {{ .dirPin = MOTOR_L_DIR, .pwmPin = MOTOR_L_PWM }};
//    static HexaDCMotor rightMotor{{ .dirPin = MOTOR_R_DIR, .pwmPin = MOTOR_R_PWM, .flipped = true }};
//    static HexaServo   arm       {{ .signalPin = SERVO_1,  .startAngle = 90.0f }};
//    static HexaServo   gripper   {{ .signalPin = SERVO_2,  .startAngle = 15.0f }};
//    static HexaIMU     imu       { hexaImuConfig()  };   // on-board IMU (Wire)
//    static HexaOLED    myOled    { hexaOledConfig() };   // on-board OLED (Wire1)
//    static HexaLineArray line    { hexaLineConfig() };   // 8-ch IR line array
//    static TankDrive    drive(leftMotor, rightMotor);
//    static DriveActions driveActions(drive);
//
//  Gamepad (always available, no declaration needed):
//    gamepad1.leftY()              — axis -1.0 to +1.0
//    gamepad1.justPressed(BTN_DPAD_RIGHT) — true on the tick of press
//
//  User LED (always available, no declaration needed):
//    userLed.setSolid(255, 0, 0)           — solid red
//    userLed.setBlink(0, 255, 0, 500)      — blink green, 500ms period
//    userLed.setPulse(0, 0, 255, 1000)     — breathe blue, 1s cycle
//    userLed.setOff()                       — turn off
//
//  Battery voltage (always available, no declaration needed):
//    power.getBatteryVoltage()   — robot pack voltage (V)
//    power.isBatteryLow()        — true when the pack is low
//
//  Rail current — read directly from `power` (always available, no declaration
//  needed). There is ONE ACS712 per rail, not one per motor/servo, so this is
//  the whole rail's current regardless of how many motors/servos you declared:
//    power.getMainCurrent()      — total battery current (A)
//    power.getMotorCurrent()     — DC-motor branch current (A)
//    power.getServoCurrent()     — servo rail current (A, derived: main − motor)
//
//  IR line array (declare one — see the block comment in HexaLineArray.h):
//    line.read(i)      — channel i calibrated 0..1000, higher = darker
//    line.readRaw(i)   — channel i raw ADC 0..4095
//    line.mask()       — one bit per channel, set where the line is seen
//    line.position()   — 0..7000 across the array, 3500 = centred, -1 = lost
//    line.error()      — position() - 3500, so ±3500 and 0 when centred
//    line.onLine()     — line seen THIS tick; check before using position()
//    line.lost()       — true once the line has been gone for lostStopMs
//    line.pattern()    — LOST / LINE / LEFT_BRANCH / RIGHT_BRANCH / TERMINAL
//    line.onGrey()     — the whole array is on a grey area (START box, parking bay)
//  Calibrate it from the controller's CALIBRATE > LINE TEST screen: two steps,
//  background first, then the line. The result is saved and survives
//  power-off; line.isCalibrated() is false until you do it. If your arena has
//  grey areas, press DOWN on that screen with the array on the grey to add the
//  grey level; onGrey() stays false without it.
//
//  Line following (see LineActions.h — needs a calibrated line array):
//    static LineActions lineActions(drive, line);
//    runAction(lineActions.auto_()
//        .leaveStart()                           // out of the grey start box
//        .followToJunction(LineSide::RIGHT, 2)   // 2nd branch on the right (corners turn by themselves)
//        .right().followToEnd()                  // turn onto it, run to its end
//        .uTurn().followToJunction().left()      // and back
//        .followToGrey().park()                  // into the grey finish box
//        .build());
//    lineActions.faultName()  — why it stopped, for telemetry ("OK" if fine)
//
//  PID controller (declare one per thing you control — see HexaPID.h):
//    static HexaPID steer{{ .kP = 0.6f, .kD = 0.05f }};
//    steer.reset()           — in start(), and after doing anything else
//    steer.update(error)     — one step; returns the correction (-1..+1)
//    steer.p() / i() / d()   — each term, for telemetry while tuning
//
//  Telemetry:
//    telemetry.addData("key", value)

// Hardware types
#include "HexaDCMotor.h"      // DC motor driver
#include "HexaServo.h"        // servo driver
#include "HexaOLED.h"         // 128×64 OLED display
#include "HexaIMU.h"          // LSM6DSOX IMU (accel + gyro + heading)
#include "HexaPower.h"        // MCP3008 ADC — battery voltage + ACS712 current
#include "HexaLineArray.h"    // 8-channel muxed IR line array + line patterns
#include "TankDrive.h"        // differential drive helper
#include "DriveTuning.h"      // DriveTuning struct (speed/msPerInch/timeout defaults)
#include "DriveActions.h"     // DriveActions — turn(), driveInches(), auto_() chaining
#include "HexaPID.h"          // HexaPID — PID controller (line following, holding a heading)
#include "LineTuning.h"       // LineTuning struct (follow speed, PID gains, junction/turn timing)
#include "LineActions.h"      // LineActions — followToJunction(), right(), uTurn(), auto_() chaining
#include "Gamepad.h"          // gamepad1 object

// Framework internals
#include "BoardPins.h"        // MOTOR_L_DIR, SERVO_1, I2C_SDA … pin constants
#include "RobotConfig.h"      // botOled, gamepad1 (framework globals)
#include "NRLGamepad.h"       // BTN_X/A/B/Y, BTN_DPAD_*, BTN_RB/LB/LT/RT constants, NRLGamepad raw statics
#include "NRLComms.h"         // NRLComms::sendTelemetry("key", value)
#include "NRLAction.h"        // instant(), sequential(), parallel(), sleep_ms(), wait_until()
#include "NRLOpMode.h"        // NRLOpMode base class, init()/start()/loop()/stop(), runAction()
#include "OpModeRegistry.h"   // REGISTER_OPMODE macro
