package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class AutoShoot extends Command {
  private static final double FORCE_FIRE_TIMEOUT_S = 3.0;

  private Turret turret;
  private Trigger trigger;
  private Timer feedDelayTimer;
  private Timer commandTimer;
  private boolean feedStarted;

  public AutoShoot(Turret turret, Trigger trigger) {
    this.turret = turret;
    this.trigger = trigger;
    feedDelayTimer = new Timer();
    commandTimer   = new Timer();
    SmartDashboard.putNumber("Current Turret Speed", 0);
    // not requiring turret because we are using it as read only
    // requiring it would prevent shooting if another command is working on turret
  }

  @Override
  public void initialize() {
    System.out.println("Shooter speed " + turret.getVelocity());
    feedDelayTimer.reset();
    commandTimer.restart();
    feedStarted = false;
  }

  @Override
  public void execute() {
    boolean atSpeed = turret.getVelocity() >= 30; //TODO: change value later for tuning
    boolean forceNow = commandTimer.hasElapsed(FORCE_FIRE_TIMEOUT_S);

    if (atSpeed || forceNow) {
      if (!feedStarted) {
        feedDelayTimer.start();
        feedStarted = true;
      }
      if (feedDelayTimer.get() >= Constants.Turret.autoShootFeedDelay) {
        trigger.shoot();
      }
    } else {
      SmartDashboard.putNumber("Current Turret Speed", turret.getVelocity());
    }
  }

  @Override
  public void end(boolean interrupted) {
    trigger.stop();
    feedDelayTimer.stop();
    feedDelayTimer.reset();
    commandTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
