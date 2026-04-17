// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.generated.TunerConstants;

/**
 * Robot-wide constants organized by subsystem.
 *
 * ── CAN IDs ──────────────────────── all bus IDs in one place
 * ── Turret ───────────────────────── flywheel, rotator, hood, zeroing
 * ── Intake ───────────────────────── deploy positions, PID, manual speeds
 * ── Trigger ──────────────────────── index / kickup duty cycles
 * ── Climber ──────────────────────── elevator speeds and tolerances
 * ── Vision (Limelight) ───────────── camera name, distance model, aim speeds
 * ── Drive ────────────────────────── swerve speed limits, deadband
 */
public class Constants {

    // =========================================================================
    // CAN IDs  (bus: "FRC 1599B")
    // =========================================================================
    public static class CAN_IDS {
        // Infrastructure
        public static final int pigeon             = 20;

        // Intake
        public static final int feedIntakeMotor    = 22;
        public static final int deployMotor        = 24;
        public static final int deployEncoder      = 33;

        // Trigger (index + kickup)
        public static final int indexMotor         = 23;
        public static final int kickUpMotor        = 28;

        // Turret
        public static final int flywheelMotorLeft   = 40;
        public static final int flywheelMotorRight  = 25;
        public static final int turretMotorRotator = 27;
        public static final int turretEncoder      = 32;

        // Climber
        public static final int climberMotor       = 31;
        public static final int climberEncoder     = 34;
    }

    // =========================================================================
    // Turret
    // =========================================================================
    public static class Turret {

        // --- Hardware ---
        /** PWM channel for the hood servo. */
        public static final int HOOD_SERVO_CHANNEL = 5;

        // --- Mechanical ---
        /** Motor-to-encoder gear ratio for the rotator. */
        public static final double GEAR_RATIO       = 10.0;
        /** Minimum allowed turret angle in degrees (soft limit). */
        public static final double MIN_ANGLE_DEG    = 1.0;
        /** Maximum allowed turret angle in degrees (soft limit). */
        public static final double MAX_ANGLE_DEG    = 175.0;
        /** Angle tolerance for "at setpoint" checks (degrees). */
        public static final double ANGLE_THRESHOLD_DEG = 0.5;

        // --- Flywheel speed ---
        /** Minimum flywheel velocity (RPS) before the trigger may fire. */
        public static final double MIN_FIRE_SPEED_RPS   = 30.0;
        /** isAtSpeed() tolerance band (RPS). */
        public static final double SHOOTER_THRESHOLD_RPS = 15.0;
        /** Delay (seconds) between reaching speed and starting the feed. */
        public static final double AUTO_SHOOT_FEED_DELAY_SECS = 0.3;

        // --- Distance → speed map  (Limelight distance units) ---
        public static final double DIST_CLOSE_FT  = 10.0;
        public static final double SPEED_CLOSE_RPS = 55.0;
        public static final double DIST_MID_FT    = 20.0;
        public static final double SPEED_MID_RPS  = 58.0;
        public static final double DIST_FAR_FT    = 35.0;
        public static final double SPEED_FAR_RPS  = 70.0;



        // --- Rotator PID  (CTRE Slot 0) ---
        public static final double ROTATOR_KP = 3.35;
        public static final double ROTATOR_KI = 0.0;
        public static final double ROTATOR_KD = 0.0;

        // --- Flywheel PID  (CTRE Slot 0) ---
        public static final double FLYWHEEL_KP = 0.64;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;

        // --- Manual rotation ---
        /** Duty-cycle scalar applied to joystick input for manual rotation. */
        public static final double ROTATE_OUTPUT_SCALE = 0.3;
        /** Joystick deadband — inputs below this magnitude are ignored. */
        public static final double MANUAL_DEADBAND     = 0.08;

        // --- Homing / zeroing ---
        /** Duty cycle applied while driving toward the hard stop. */
        public static final double ZERO_POWER              = -0.08;
        /** Supply current (A) that indicates the rotator has stalled. */
        public static final double ZERO_STALL_CURRENT_AMPS =  1.25;
        /** Per-sample current delta (A) used in stall detection. */
        public static final double ZERO_CURRENT_DELTA_AMPS =  0.1;

        // --- Hood servo ---
        /** Servo position (0–1) for hood-up (long shot). */
        public static final double HOOD_UP_POS   = 0.8;
        /** Servo position (0–1) for hood-down (close shot). */
        public static final double HOOD_DOWN_POS = 0.0;
    }

    // =========================================================================
    // Intake
    // =========================================================================
    public static class Intake {

        // --- Wheel speed ---
        /** Full-power intake duty cycle. */
        public static final double INTAKE_SPEED = 1.0;

        // --- Deploy motor positions (rotations, internal encoder) ---
        /** Position when intake is fully deployed (down). */
        public static final double DEPLOY_POS_ROT  = -0.7;
        /** Position when intake is fully retracted (up). */
        public static final double RETRACT_POS_ROT = -0.9;

        // --- Position tolerances ---
        /** Command completes when error is within this band (rotations). */
        public static final double POSITION_TOLERANCE_ROT = 0.035;
        /** Positions with |value| below this use Slot 0 (near-home hold). */
        public static final double SLOT_SELECT_THRESHOLD  = 0.015;

        // --- Deploy motor PID  (CTRE) ---
        /** Slot 0 kP — strong hold near the home position. */
        public static final double DEPLOY_SLOT0_KP = 5.0;
        /** Slot 1 kP — softer gain for travel to deploy/retract positions. */
        public static final double DEPLOY_SLOT1_KP = 1.5;

        // --- Manual / timed deploy ---
        /** Duty cycle for timed manual deploy moves. */
        public static final double DEPLOY_MANUAL_SPEED = 0.15;
        /** Time (seconds) to run deploy motor during DeployIntake. */
        public static final double DEPLOY_WAIT_SECS    = 1.2;

        // --- DeployJumpCommand oscillation ---
        /** Time (seconds) spent driving intake upward per cycle. */
        public static final double JUMP_UP_DURATION_SECS   = 0.62;
        /** Time (seconds) spent driving intake downward per cycle. */
        public static final double JUMP_DOWN_DURATION_SECS = 0.45;
        /** Duty cycle magnitude for jump oscillation. */
        public static final double JUMP_SPEED               = 0.22;
    }

    // =========================================================================
    // Trigger  (index motor + kickup motor)
    // =========================================================================
    public static class Trigger {
        /** Duty cycle for shooting direction. */
        public static final double SHOOT_SPEED   =  1.0;
        /** Duty cycle for reverse (unjam) direction. */
        public static final double REVERSE_SPEED = -1.0;
    }

    // =========================================================================
    // Climber  (elevator)
    // =========================================================================
    public static class Climber {
        /** Position tolerance (rotations) for ClimbUp/ClimbDown setpoint checks. */
        public static final double SETPOINT_THRESHOLD_ROT = 0.15;
        public static final double LINEUP_HEADING_THRESHOLD_DEG = 2;
        // --- LineUpClimb vision alignment ---
        /** Proportional gain for the lateral alignment P controller. */
        public static final double LINEUP_P           = 0.2;
        /** Lateral error (meters) considered "aligned" — ends LineUpClimb. */
        public static final double LINEUP_THRESHOLD_M = 0.2;

        // --- ManualClimb duty cycles ---
        /** Duty cycle when climbing up. */
        public static final double MANUAL_UP_SPEED   =  0.30;
        /** Duty cycle when climbing down (negative = downward). */
        public static final double MANUAL_DOWN_SPEED = -0.60;
    }

    // =========================================================================
    // Vision  (Limelight)
    // =========================================================================
    public static class LimelightConstants {
        /** NetworkTables name of the turret-facing Limelight. */
        public static final String LEFT_LIMELIGHT_NAME = "limelight-left";
        public static final String FRONT_LIMELIGHT_NAME = "limelight-front";

        // --- Distance model:  distance = DISTANCE_SCALE * ta ^ DISTANCE_EXPONENT ---
        public static final double DISTANCE_SCALE      = 46.39986;
        public static final double DISTANCE_EXPONENT   = -0.4918478;
        /** ta values above this threshold are too close to use for ranging. */
        public static final double MAX_TA_FOR_DISTANCE = 24.0;

        // --- Turret auto-aim thresholds (tx degrees off-center) ---
        /** Switch from fast to slow rotation below this tx error. */
        public static final double TX_THRESHOLD_LARGE  = 10.0;
        /** Stop rotating (consider locked) below this tx error. */
        public static final double TX_THRESHOLD_SMALL  = 4.5;

        // --- Turret auto-aim output speeds (duty cycle) ---
        /** Rotation speed when tx error > TX_THRESHOLD_LARGE. */
        public static final double TURRET_SPEED_FAST   = 0.15;
        /** Rotation speed when tx error ≤ TX_THRESHOLD_LARGE. */
        public static final double TURRET_SPEED_SLOW   = 0.07;
    }

    // =========================================================================
    // Field geometry
    // =========================================================================
    public static class Field {
        /** Center of the Blue alliance hub (meters, WPILib origin). */
        public static final Translation2d BLUE_HUB = new Translation2d(4.611, 4.035);
        /** Center of the Red alliance hub (meters, WPILib origin). */
        public static final Translation2d RED_HUB  = new Translation2d(11.927, 4.035);
    }

    // =========================================================================
    // Drive  (swerve drivetrain)
    // =========================================================================
    public static class Drive {
        /** Top speed (m/s) — sourced from TunerConstants at full 12 V. */
        public static final double MAX_SPEED_MPS      = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        /** Max angular rate (rad/s) — 0.75 rotations/sec, matches teleop drive default command. */
        public static final double MAX_ANGULAR_RATE_RPS = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        /** Fraction of max speed/rate applied as joystick deadband. */
        public static final double DEADBAND_PERCENT   = 0.14;

        // --- Speed scaling (used in drive default command) ---
        /** Current speed divisor (1.0 = full speed). Increase for a slower mode. */
        public static final double SPEED_DIVISOR      = 1.0;


    }
}
