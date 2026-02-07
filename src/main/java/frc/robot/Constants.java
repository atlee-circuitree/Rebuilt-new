// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class Constants {

    public class CAN_IDS {
        public static final int wristMotor = 31;
        public static final int wristEncoder = 3;
        public static final int intakeMotor = -1;
        public static final int deployMotor = -1;
        public static final int turretMotorRight = -1;
        public static final int turretMotorLeft = -1;
        public static final int turretMotorRotator = -1;
        public static final int triggerMotor = -1;
        public static final int tankMotorLeft = -1;
        public static final int tankMotorRight = -1;
    }

    public class Wrist {
        public static final double[] wristLimit =  new double[] {330, 30};
        public static final double wristEncoderOffset = 0.351;
        public static final double P = 0.1;
        public static final double I = 0;
        public static final double D = 0;
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

