package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.LimelightHelpers.PoseEstimate;

public class Limelight {

    // ── Turret limelight (aiming only) ──────────────────────────────────────
    private final static String turretName = "limelight-tur";
    private final static NetworkTable limelight = NetworkTableInstance.getDefault().getTable(turretName);
    private final static PIDController turretPID = new PIDController(0.01, 0, 0);

    private static final int[] RED_TAGS  = {8, 9, 10, 11};
    private static final int[] BLUE_TAGS = {24, 25, 26, 27};

    private static int[] getAllianceTags() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? RED_TAGS : BLUE_TAGS;
    }

    /** Returns true if at least one alliance-appropriate tag is visible on the turret limelight. */
    public static boolean getTurretTV() {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(turretName);
        int[] tags = getAllianceTags();
        for (LimelightHelpers.RawFiducial f : fiducials) {
            for (int id : tags) {
                if (f.id == id) return true;
            }
        }
        return false;
    }

    /**
     * Returns the PID-calculated motor speed to center on alliance tags.
     * If two tags are visible, splits the error between them (average txnc).
     * Returns 0 if no alliance tags are visible.
     */
    public static double getTurretSpeed() {
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(turretName);
        int[] tags = getAllianceTags();
        double txSum = 0;
        int count = 0;
        for (LimelightHelpers.RawFiducial f : fiducials) {
            for (int id : tags) {
                if (f.id == id) {
                    txSum += f.txnc;
                    count++;
                    break;
                }
            }
        }
        if (count == 0) return 0;
        return turretPID.calculate(txSum / count, 0);
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
        return (46.39986) * Math.pow(getTA(), -0.4918478);
    }

    // ── Climber limelight (MegaTag pose estimation) ──────────────────────────
    private final static String climberName = "limelight-climb";
    private static final double MAX_MEASUREMENT_LATENCY_MS = 100.0;

    public static boolean getTV() {
        return LimelightHelpers.getTV(climberName);
    }

    /**
     * Returns a fused MegaTag1 + MegaTag2 pose measurement from the climber
     * limelight, or empty if no valid target or the frame is too stale.
     *
     * @param currentPose      Current robot pose (for SetRobotOrientation)
     * @param pitchDeg         Robot pitch in degrees
     * @param rollDeg          Robot roll in degrees
     * @param yawRateDegPerSec Robot yaw rate in degrees per second
     */
    public static Optional<Measurement> getMeasurement(
            Pose2d currentPose, double pitchDeg, double rollDeg, double yawRateDegPerSec) {

        LimelightHelpers.SetRobotOrientation(
            climberName,
            currentPose.getRotation().getDegrees(), yawRateDegPerSec,
            pitchDeg, 0,
            rollDeg, 0
        );

        final PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(climberName);
        final PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(climberName);

        if (mt1 == null || mt2 == null || mt1.tagCount == 0 || mt2.tagCount == 0) {
            return Optional.empty();
        }

        // Drop stale frames — robot may have moved significantly since capture.
        if (mt2.latency > MAX_MEASUREMENT_LATENCY_MS) {
            return Optional.empty();
        }

        // Use MegaTag2 position (gyro-aided, more stable) with MegaTag1 rotation (less gyro drift).
        mt2.pose = new Pose2d(mt2.pose.getTranslation(), mt1.pose.getRotation());

        // Scale XY trust by how much of the image the tags occupy.
        // Larger area = closer = more accurate. Rotation stays high (MegaTag2 relies on gyro).
        double xyStdDev = Math.max(0.03, Math.min(3.0, 0.1 / Math.max(0.01, mt2.avgTagArea)));
        if (mt2.tagCount > 1) xyStdDev *= 0.5;
        final Matrix<N3, N1> stdDevs = VecBuilder.fill(xyStdDev, xyStdDev, 10.0);

        return Optional.of(new Measurement(mt2, stdDevs));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}
