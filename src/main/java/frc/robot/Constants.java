// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class Constants {

    public class CAN_IDS {
        public static final int wristMotor = 20;
        public static final int wristEncoder = 21;
        public static final int intakeMotorLeft = 22;
        public static final int intakeMotorRight = 23;
        public static final int deployMotor = 24;
        public static final int turretMotorRight = 25;
        public static final int turretMotorLeft = 26;
        public static final int turretMotorRotator = 27;
        public static final int triggerMotor = 28;
        public static final int tankMotorLeft = 29;
        public static final int tankMotorRight = 30;
        public static final int climberMotorLeft = 31;
        public static final int climberMotorRight = 32;
    }
    
    public class Channels {
        public static final int motorHoodLeft = 0;
        public static final int motorHoodRight = 1;
        public static final int door = 2;
        public static final int deploy = 3;
    }

    public class Wrist {
        public static final double[] wristLimit =  new double[] {330, 30};
        public static final double wristEncoderOffset = 0.351;
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
    }

    public class Turret {
        public static final double minAngle = 0;
        public static final double maxAngle = 0;
        public static final double shooterThreshold = 0;
    }

    public class Intake {
        public static final double deployPosition = 90; // angle in degrees
        public static final double homePosition = 0;
        public static final double intakeSpeed = 0;
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
  
}

