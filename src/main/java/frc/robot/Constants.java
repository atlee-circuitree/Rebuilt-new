// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.*;

import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class Constants {

    public class CAN_IDS {
        public static final int pigeon = 20;
        public static final int fuelTankMotor = 21;
        public static final int feedIntakeMotor = 22;
        public static final int indexMotor = 23;
        public static final int deployMotor = 24;
        public static final int turretMotorRight = 25;
        public static final int turretMotorLeft = 40;
        public static final int turretMotorRotator = 27;
        public static final int kickUpMotor = 28;
        public static final int tankMotorLeft = 29;
        public static final int tankMotorRight = 30;
        public static final int climberMotor = 31;
        public static final int turretEncoder = 32;
        public static final int deployEncoder = 33;
        public static final int climberEncoder = 34;

    }
    
    public class Channels {
        public static final int motorHoodLeft = 5;
        //public static final int motorHoodRight = 1;
        
    }

    public class Wrist {
        public static final double[] wristLimit =  new double[] {330, 30};
        public static final double wristEncoderOffset = 0.351;
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
    }

    public class Turret {
        public static final double minAngle = 8;
        public static final double maxAngle = 120;
        public static final double shooterThreshold = 0;
        public static final double angleThreshold = 2;
        public static final double autoShootFeedDelay = 0.3; // seconds to wait after turret reaches speed before running kickup and index

        // Distance-based flywheel speed map - TODO: tune all values
        // Distance units match Limelight.getDistance() (derived from ta via power-law formula)
        public static final double distClose  = 10.0;  // distance unit - close range
        public static final double speedClose = 55.0; // RPS
        public static final double distMid    = 20.0;  // distance unit - mid range
        public static final double speedMid   = 58.0; // RPS
        public static final double distFar    = 35.0;  // distance unit - far range
        public static final double speedFar   = 75.0;  // RPS
    }

    public class Intake {
        public static final double deployPosition = 25; // angle in degrees
        public static final double homePosition = 1;
        public static final double intakeSpeed = 1;
        public static final double intakeEncoderOffset = 0;
        public static final double deployLowThreshold = 0;
        public static final double deployHighThreshold = .3;
    }

    public class Climber {
        public static final double threshold = 0.15;
    }

    public class TriggerPositions {

    }

    public class HoodPositions {

    }

    public class Speed {

    }
    /*
     * lime light values =======================================
     * scale = 46.39986
     * distance = (scale / ta)
     */
    public class Properties {

        public static double intakeVelocity = .5;
        public static double outtakeVelocity = -.5;
        
    }
  
    
    public class Drive {
        public static boolean SpeedToggle = true; //true = fast, false = slow
        public static double Speed = 1;
        public static double maxSpeed = 1;
        public static double minSpeed = 3.5; // the value is what the speed is being divided by
        
        public static double MaxSpeed = (TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)); // kSpeedAt12Volts desired top speed 6 wads slow
        public static double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 1/2 of a rotation per second max
    }
}

