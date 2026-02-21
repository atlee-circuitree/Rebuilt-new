// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class Constants {

    public class CAN_IDS {
        /*public static final int wristMotor = 31;
        public static final int wristEncoder = 3;
        public static final int intakeMotorLeft = 4;
        public static final int intakeMotorRight = 6;
        public static final int deployMotor = 5;
        public static final int turretMotorRight = 7;
        public static final int turretMotorLeft = 8;
        public static final int turretMotorRotator = 9;
        public static final int triggerMotor = 10;
        public static final int tankMotorLeft = 11;
        public static final int tankMotorRight = 12;
        public static final int climberMotorLeft = 13;
        public static final int climberMotorRight = 14;*/
    }
    
    public class Channels {
        public static final int motorHoodLeft = 1;
        public static final int motorHoodRight = 2;
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

