
package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class AutoTurret extends Command {

  // Zones in WPILib field meters (converted from 2022 field inches: divide by 39.3701)
  private final Rectangle2d Zone0     = new Rectangle2d(new Translation2d(12.47, 8.03), new Translation2d(16.54, 4.01));
  private final Rectangle2d Zone1     = new Rectangle2d(new Translation2d( 5.18, 5.49), new Translation2d(11.35, 2.74));
  private final Rectangle2d Zone2     = new Rectangle2d(new Translation2d( 5.18, 2.74), new Translation2d(11.35, 0.00));
  private final Rectangle2d Zone3     = new Rectangle2d(new Translation2d(12.47, 2.74), new Translation2d(16.54, 0.00));
  private final Rectangle2d DeadZone1 = new Rectangle2d(new Translation2d( 5.36, 4.75), new Translation2d( 6.55, 3.25));
  private final Rectangle2d DeadZone2 = new Rectangle2d(new Translation2d(12.65, 4.75), new Translation2d(13.84, 3.25));


    private final Turret turret;
    private final Trigger trigger;
    private final CommandSwerveDrivetrain drivetrain;
    // key = zone id, value = target point to shoot at
    private final HashMap<Integer, Translation2d> table;

  public AutoTurret(Turret turret, Trigger trigger, CommandSwerveDrivetrain drivetrain) {
    this.turret = turret;
    this.drivetrain = drivetrain;
    this.trigger = trigger;
    addRequirements(turret);
    addRequirements(trigger);

    table = new HashMap<>();
    table.put(0, new Translation2d(2.31, 6.83));
    table.put(1, new Translation2d(2.31, 1.19));
    table.put(2, new Translation2d(4.62, 4.01));
  }

  private int getZone(Pose2d pose)
  {

    double x = pose.getX();
    double y = pose.getY();

    if (Zone0.contains(new Translation2d(x, y))) {
      return 0;
    } else if (Zone1.contains(new Translation2d(x, y))) {
      return 1;
    } else if (Zone2.contains(new Translation2d(x, y))) {
      return 2;
    } else if (Zone3.contains(new Translation2d(x, y))) {
      return 3;
    } else {
      return 4;
    }
  }

  private boolean isInDeadZone (Pose2d pose)
  {
  
    double x = pose.getX();
    double y = pose.getY();
    
    if (DeadZone1.contains(new Translation2d(x, y))) {
      return true;
    } else if (DeadZone2.contains(new Translation2d(x, y))) {
      return true;
    }
    
    return false;
  }

  @Override
  public void execute()
  {
    Pose2d pose = drivetrain.getState().Pose;
    int zone = getZone(pose);
    
    Translation2d target = table.get(zone);
    if (isInDeadZone(pose))
    {
        turret.runAtPower(0.2);
        
        trigger.stop();
    }
    else
    {
        if (target == null) {
          // Zone has no registered target — hold position and don't fire
          trigger.stop();
          return;
        }
        if (zone == 4) {
          turret.setHoodPosition(false);
        } else {
          turret.setHoodPosition(true);
        }
        double dist = Math.sqrt(Math.pow(pose.getX() - target.getX(), 2) + Math.pow(pose.getY() - target.getY(), 2));
        double angle = Math.atan2(pose.getY() - target.getY(), pose.getX() - target.getX());
        angle -= pose.getRotation().getRadians();
        angle = Math.toDegrees(angle);
        //turret.rotateTo(angle);

        if (Math.abs(angle) <= 60)
        {
            turret.spinAtDistance();
            if (turret.isAtSpeed())
                trigger.shoot();
        }
        else
        {
            turret.runAtPower(0.2);
            trigger.stop();
        }
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
    trigger.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
