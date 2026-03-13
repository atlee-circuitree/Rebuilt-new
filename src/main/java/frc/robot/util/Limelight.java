package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

public class Limelight {
    private final static String limelightName = "limelight-turret";
    private final static NetworkTable limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    private final static PIDController turretPID = new PIDController(0.05, 0, 0);

    public static double getTurretSpeed() {
        return turretPID.calculate(getTX(), 0); // , 0 <--- this is desired value.
    }

    public static double getTA() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public static double getTX() {
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public static double getTY() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public static double getDistance() {
        if (getTA() > 24 || getTA() == 0) {
            return 0;
        }

        double value = (46.39986) * Math.pow(getTA(), -0.4918478);
        return value;

    }
    
}
