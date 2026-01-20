package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private final static String limelightName = "limelight-left";
    private final static NetworkTable limelight = NetworkTableInstance.getDefault().getTable(limelightName);
    private final static PIDController turretPID = new PIDController(0.05, 0, 0);

    public static double getTurretSpeed() {
        return turretPID.calculate(getTX(), 0);
    }

    public static double getTA() {
        return limelight.getValue("ta").getDouble();
    }

    public static double getTX() {
        return limelight.getValue("tx").getDouble();
    }

    public static double getTY() {
        return limelight.getValue("ty").getDouble();
    }

    public static double getDistance() {
        if (getTA() > 24 || getTA() == 0) {
            return 0;
        }

        double value = (46.39986) * Math.pow(getTA(), -0.4918478);
        return value;

    }

}
