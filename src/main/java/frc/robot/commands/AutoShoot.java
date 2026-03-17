package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class AutoShoot extends Command {
  private Turret turret;
  private Trigger trigger;
  private Timer feedDelayTimer;
  private boolean feedStarted;

  public AutoShoot(Turret turret, Trigger trigger) {
    this.turret = turret;
    this.trigger = trigger;
    feedDelayTimer = new Timer();
    // Turret is not required — used read-only so parallel spin commands can run concurrently.
    // Trigger IS required because shoot() writes to it.
    addRequirements(trigger);
  }

  @Override
  public void initialize() {
    feedDelayTimer.reset();
    feedStarted = false;
  }

  @Override
  public void execute() {
    if (turret.getVelocity() >= Constants.Turret.MIN_FIRE_SPEED_RPS) {
      if (!feedStarted) {
        feedDelayTimer.start();
        feedStarted = true;
      }
      if (feedDelayTimer.get() >= Constants.Turret.autoShootFeedDelay) {
        trigger.shoot();
      }
    } else {
      SmartDashboard.putNumber("Turret/current_velocity_rps", turret.getVelocity());
    }
  }

  @Override
  public void end(boolean interrupted) {
    trigger.stop();
    feedDelayTimer.stop();
    feedDelayTimer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
