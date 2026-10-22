package frc.robot.util;

import java.util.OptionalDouble;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

public class Limelight {
    private static final NetworkTable limelight = NetworkTableInstance.getDefault()
            .getTable(Constants.LimelightConstants.TURRET_LIMELIGHT_NAME);

        public static double getTurretSpeed() {
        // Only steer toward one of OUR alliance's centered HUB tags
        if (!hasHubTarget()) {
            return 0;
        }
        double tx = getTXNC();
        if (tx > Constants.LimelightConstants.TX_THRESHOLD_LARGE)
            return Constants.LimelightConstants.TURRET_SPEED_FAST;
        if (tx > Constants.LimelightConstants.TX_THRESHOLD_SMALL)
            return Constants.LimelightConstants.TURRET_SPEED_SLOW;
        if (tx < -Constants.LimelightConstants.TX_THRESHOLD_LARGE)
            return -Constants.LimelightConstants.TURRET_SPEED_FAST;
        if (tx < -Constants.LimelightConstants.TX_THRESHOLD_SMALL)
            return -Constants.LimelightConstants.TURRET_SPEED_SLOW;
        return 0;
    }

    public static double getTA() {
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public static double getTA(String limelightName) {
        return NetworkTableInstance.getDefault()
            .getTable(limelightName).getEntry("ta").getDouble(0.0);
    }

    public static double getTX() {
        return limelight.getEntry("tx").getDouble(0.0);
    }
        public static double getTXNC() {
        return limelight.getEntry("txnc").getDouble(0.0);
    }

    public static double getTY() {
        return limelight.getEntry("ty").getDouble(0.0);
    }

    // ---------------------------------------------------------------------------------------------
    // NEW: tag-based distance (replaces the area/"ta" guess for shooting)
    // ---------------------------------------------------------------------------------------------

    /** True if the Limelight currently sees a target (tv == 1). */
    public static boolean getTV() {
        return limelight.getEntry("tv").getDouble(0.0) >= 1.0;
    }

    /** AprilTag ID of the primary target, or -1 if none. */
    public static int getTID() {
        return (int) limelight.getEntry("tid").getDouble(-1.0);
    }

    /** True if the primary target is one of OUR alliance's centered HUB tags. */
    public static boolean hasHubTarget() {
        if (!getTV()) {
            return false;
        }
        int id = getTID();
        for (int candidate : hubTagIds()) {
            if (candidate == id) {
                return true;
            }
        }
        return false;
    }

    /**
     * Distance in METERS from the camera to the HUB tag, from the tag's vertical angle:
     * d = (tagHeight - lensHeight) / tan(cameraPitch + ty). Empty if no HUB tag is visible or the
     * angle is nonsense. Uses ty (not ta), so it does not depend on tag size or tilt.
     */
    public static OptionalDouble getHubDistanceMeters() {
        if (!hasHubTarget()) {
            return OptionalDouble.empty();
        }
        double ty = getTY();
        if (Constants.LimelightConstants.CAMERA_FLIP_TY) {
            ty = -ty;
        }
        double angleDeg = Constants.LimelightConstants.CAMERA_PITCH_DEGREES + ty;
        if (angleDeg <= 0.0 || angleDeg >= 90.0) {
            return OptionalDouble.empty();
        }
        double heightDiff = Constants.LimelightConstants.HUB_TAG_HEIGHT_METERS
                - Constants.LimelightConstants.CAMERA_LENS_HEIGHT_METERS;
        double d = heightDiff / Math.tan(Math.toRadians(angleDeg));
        return (d > 0.0 && d < Constants.LimelightConstants.MAX_VALID_DISTANCE_METERS)
                ? OptionalDouble.of(d)
                : OptionalDouble.empty();
    }

    private static int[] hubTagIds() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            return Constants.LimelightConstants.HUB_TAG_IDS_RED;
        }
        if (alliance.isPresent()) {
            return Constants.LimelightConstants.HUB_TAG_IDS_BLUE;
        }
        // Alliance unknown (e.g. on the bench): accept either side.
        int[] all = new int[Constants.LimelightConstants.HUB_TAG_IDS_RED.length
                + Constants.LimelightConstants.HUB_TAG_IDS_BLUE.length];
        System.arraycopy(Constants.LimelightConstants.HUB_TAG_IDS_RED, 0, all, 0,
                Constants.LimelightConstants.HUB_TAG_IDS_RED.length);
        System.arraycopy(Constants.LimelightConstants.HUB_TAG_IDS_BLUE, 0, all,
                Constants.LimelightConstants.HUB_TAG_IDS_RED.length,
                Constants.LimelightConstants.HUB_TAG_IDS_BLUE.length);
        return all;
    }

    // ---------------------------------------------------------------------------------------------
    // OLD area-based distance. Kept so nothing else breaks, but do NOT use it for shooting.
    // ---------------------------------------------------------------------------------------------

    public static double getDistance() {
        double ta = getTA();
        if (ta > Constants.LimelightConstants.MAX_TA_FOR_DISTANCE || ta == 0) {
            return 0;
        }
        return Constants.LimelightConstants.DISTANCE_SCALE
                * Math.pow(ta, Constants.LimelightConstants.DISTANCE_EXPONENT);
    }

    public static double getDistance(String limelightName) {
        double ta = getTA(limelightName);
        if (ta > Constants.LimelightConstants.MAX_TA_FOR_DISTANCE || ta == 0) {
            return 0;
        }
        return Constants.LimelightConstants.DISTANCE_SCALE
                * Math.pow(ta, Constants.LimelightConstants.DISTANCE_EXPONENT);
    }
}