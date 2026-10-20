package frc.robot.subsystems;
import limelight.Limelight;
import limelight.networktables.LimelightSettings.LEDMode;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    //Creating and setting up the objects for each limelight
    private final Limelight limelight;
    private final Limelight limelightLeft;
    private final Limelight limelightFront;

    public VisionSubsystem() {
        limelight = new Limelight("limelight");
        limelight.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(Pose3d.kZero)
            .save(); 

        limelightLeft = new Limelight("limelightLeft");
        limelightLeft.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(Pose3d.kZero)
            .save(); 

        limelightFront = new Limelight("limelightFront");
        limelightFront.getSettings()
            .withLimelightLEDMode(LEDMode.PipelineControl)
            .withCameraOffset(Pose3d.kZero)
            .save(); 
    }  
    
    @Override
    public void periodic(){
         if (limelight.getData().targetData.getTargetStatus()) {
            double tx = limelight.getData().targetData.getHorizontalOffset();
            double ty = limelight.getData().targetData.getVerticalOffset();
        }
        if (limelightLeft.getData().targetData.getTargetStatus()) {
            double tx = limelightLeft.getData().targetData.getHorizontalOffset();
            double ty = limelightLeft.getData().targetData.getVerticalOffset();
        }
        if (limelightFront.getData().targetData.getTargetStatus()) {
            double tx = limelightFront.getData().targetData.getHorizontalOffset();
            double ty = limelightFront.getData().targetData.getVerticalOffset();
        } 
    }   
}
    

