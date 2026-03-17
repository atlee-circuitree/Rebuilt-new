package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;

public class ReverseShoot extends Command {
  private final Trigger trigger;

  public ReverseShoot(Trigger trigger) {
    this.trigger = trigger;
    addRequirements(trigger);
  }

  @Override
  public void execute() {
    trigger.reverseShoot();
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
