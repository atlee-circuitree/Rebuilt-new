package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class Shoot extends Command {
  private final Turret turret;
  private final Trigger trigger;

  public Shoot(Turret turret, Trigger trigger) {
    this.turret = turret;
    this.trigger = trigger;
    // Turret is not required — used read-only so parallel spin commands can run concurrently.
    // Trigger IS required because shoot() writes to it.
    addRequirements(trigger);
  }

  @Override
  @Override
  public void execute() {
    if (turret.isAtSpeed()) {
      trigger.shoot();
    }
  }

  @Override
  public void end(boolean interrupted) {
    trigger.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
