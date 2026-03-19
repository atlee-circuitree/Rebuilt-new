package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
        feedDelayTimer.reset();
        feedDelayTimer.start();
        feedStarted = true;
      }
      if (feedDelayTimer.get() >= Constants.Turret.AUTO_SHOOT_FEED_DELAY_SECS) {
        trigger.shoot();
      }
    } else {
      // Speed dropped below threshold — reset so the delay restarts cleanly on recovery
      if (feedStarted) {
        feedDelayTimer.stop();
        feedDelayTimer.reset();
        feedStarted = false;
      }
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
