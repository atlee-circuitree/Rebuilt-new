
package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class AutoTurret extends Command {

    private class Point {
        int x;
        int y;

        public Point(int xi, int yi)
        {
            x = xi;
            y = yi;
        }
    }

    private Turret turret;
    private Trigger trigger;
    private CommandSwerveDrivetrain drivetrain;
    // key = zone id, value = target point to shoot at
    private HashMap<Integer, Point> table;

  public AutoTurret(Turret turret, Trigger trigger, CommandSwerveDrivetrain drivetrain) {
    this.turret = turret;
    this.drivetrain = drivetrain;
    this.trigger = trigger;
    addRequirements(turret);
    addRequirements(trigger);

    table = new HashMap<>();
    table.put(0, new Point(0, 0));
    table.put(1, new Point(0, 0));
    table.put(2, new Point(0, 0));
    table.put(3, new Point(0, 0));
    table.put(4, new Point(0, 0));
  }

  private int getZone(Pose2d pose)
  {
    double x = pose.getX();
    double y = pose.getY();
    if (x > 491 && y > 108)
        return 0;
    else if (x <= 491 && x > 204 && y > 108)
        return 1;
    else if (x <= 491 && x > 204 && y <= 108)
        return 2;
    else if (x > 491 && y <= 108)
        return 3;
    else    
        return 4;
  }

  private boolean isInDeadZone (Pose2d pose)
  {
    return false;  
  }

  @Override
  public void execute()
  {
    Pose2d pose = drivetrain.getState().Pose;
    int zone = getZone(pose);
    Point target = table.get(zone);
    if (isInDeadZone(pose))
    {
        turret.runAtPower(0.2);
        trigger.stop();
    }
    else
    {
        double dist = Math.sqrt(Math.pow(pose.getX() - target.x, 2) + Math.pow(pose.getY() - target.y, 2));
        double angle = Math.atan2(pose.getY() - target.y, pose.getX() - target.x);
        angle -= pose.getRotation().getRadians();
        angle = Math.toDegrees(angle);
        turret.rotateTo(angle);

        if (Math.abs(angle) <= 60)
        {
            turret.spin(100);
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
  public boolean isFinished() {
    return false;
  }
}
