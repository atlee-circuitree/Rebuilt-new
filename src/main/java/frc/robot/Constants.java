// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
import frc.robot.generated.TunerConstants;

public class Constants {

    public static class CAN_IDS {
        public static final int pigeon             = 20;
        public static final int feedIntakeMotor    = 22;
        public static final int indexMotor         = 23;
        public static final int deployMotor        = 24;
        public static final int turretMotorRight   = 25;
        public static final int turretMotorLeft    = 40;
        public static final int turretMotorRotator = 27;
        public static final int kickUpMotor        = 28;
        public static final int climberMotor       = 31;
        public static final int turretEncoder      = 32;
        public static final int deployEncoder      = 33;
        public static final int climberEncoder     = 34;
    }

    public static class Turret {
        // Hardware channels
        public static final int HOOD_SERVO_CHANNEL = 5;

        // Mechanical
        public static final double GEAR_RATIO = 10.0;
        public static final double minAngle = 8.0;    // degrees
        public static final double maxAngle = 120.0;  // degrees
        public static final double angleThreshold = 2.0; // degrees

        // Flywheel control
        /** Minimum flywheel speed (RPS) before the trigger is allowed to fire. */
        public static final double MIN_FIRE_SPEED_RPS = 30.0;
        /** Tolerance (RPS) for isAtSpeed() — flywheel is "at speed" when within this of target. */
        public static final double shooterThreshold = 5.0;
        /** Seconds to wait after flywheel reaches speed before running kickup/index. */
        public static final double autoShootFeedDelay = 0.3;

        // Distance-based flywheel speed map (distance units from Limelight.getDistance())
        public static final double distClose  = 10.0;
        public static final double speedClose = 55.0; // RPS
        public static final double distMid    = 20.0;
        public static final double speedMid   = 58.0; // RPS
        public static final double distFar    = 35.0;
        public static final double speedFar   = 75.0; // RPS

        // Rotator CTRE closed-loop (Slot 0)
        public static final double ROTATOR_KP = 0.1;
        public static final double ROTATOR_KI = 0.0;
        public static final double ROTATOR_KD = 0.0;

        // Flywheel CTRE closed-loop (Slot 0)
        public static final double FLYWHEEL_KP = 0.64;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;

        // Manual rotation
        /** Duty cycle scalar applied to stick input for manual turret rotation. */
        public static final double ROTATE_OUTPUT_SCALE = 0.15;
        /** Stick deadband — inputs below this magnitude are ignored. */
        public static final double MANUAL_DEADBAND = 0.08;

        // Zeroing
        public static final double ZERO_POWER              = -0.08; // duty cycle during homing
        public static final double ZERO_STALL_CURRENT_AMPS =  1.25; // current threshold for hard-stop
        public static final double ZERO_CURRENT_DELTA_AMPS =  0.1;  // per-sample delta threshold

        // Hood servo positions
        public static final double HOOD_UP_POS   = 0.8;
        public static final double HOOD_DOWN_POS = 0.0;
    }

    public static class Intake {
        public static final double intakeSpeed = 1.0;

        // Deploy motor positions (rotations)
        public static final double DEPLOY_POS_ROT  = -0.7; // deployed (down)
        public static final double RETRACT_POS_ROT = -0.9; // retracted (up)

        // Position tolerances
        public static final double POSITION_TOLERANCE_ROT = 0.035; // rotations
        /** Positions with absolute value below this use Slot 0 (near-home strong hold). */
        public static final double SLOT_SELECT_THRESHOLD  = 0.015; // rotations

        // Deploy motor CTRE PID slots
        public static final double DEPLOY_SLOT0_KP = 5.0;  // Slot 0: near-home strong hold
        public static final double DEPLOY_SLOT1_KP = 1.5;  // Slot 1: travel to deploy/retract

        // Manual / timed deploy
        public static final double DEPLOY_MANUAL_SPEED = 0.15; // duty cycle
        public static final double DEPLOY_WAIT_SECS    = 1.2;  // seconds for timed deploy

        // DeployJumpCommand oscillation timings
        public static final double JUMP_UP_DURATION_SECS   = 0.62;
        public static final double JUMP_DOWN_DURATION_SECS = 0.45;
        public static final double JUMP_SPEED = 0.22;
    }

    public static class Trigger {
        public static final double SHOOT_SPEED   =  1.0;
        public static final double REVERSE_SPEED = -1.0;
    }

    public static class Climber {
        public static final double threshold = 0.15; // position tolerance for setpoint commands

        // LineUpClimb P controller
        public static final double LINEUP_P           = 0.2;
        public static final double LINEUP_THRESHOLD_M = 0.2; // meters, error to consider aligned

        // ManualClimb speeds
        public static final double MANUAL_UP_SPEED   =  0.30;
        public static final double MANUAL_DOWN_SPEED = -0.60;
    }

    public static class LimelightConstants {
        public static final String TURRET_LIMELIGHT_NAME = "limelight-turret";

        // Distance formula: distance = DISTANCE_SCALE * ta^DISTANCE_EXPONENT
        public static final double DISTANCE_SCALE    = 46.39986;
        public static final double DISTANCE_EXPONENT = -0.4918478;
        /** ta values above this are too close to estimate distance reliably. */
        public static final double MAX_TA_FOR_DISTANCE = 24.0;

        // Turret auto-aim tx thresholds (degrees)
        public static final double TX_THRESHOLD_LARGE = 10.0;
        public static final double TX_THRESHOLD_SMALL = 3.0;

        // Turret auto-aim output speeds (duty cycle)
        public static final double TURRET_SPEED_FAST = 0.15;
        public static final double TURRET_SPEED_SLOW = 0.07;
    }

    public static class Drive {
        public static final boolean SpeedToggle = true;  // true = fast, false = slow
        public static final double Speed    = 1.0;
        public static final double minSpeed = 3.5;       // divisor for slow mode
        public static final double DEADBAND_PERCENT = 0.14;
        public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        public static final double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    }
}
