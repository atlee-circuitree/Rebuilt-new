package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class Shoot extends Command {
  private Turret turret;
  private Trigger trigger;
  
  public Shoot(Turret turret, Trigger trigger) {
    this.turret = turret;
    this.trigger = trigger;
    addRequirements(trigger);
    // not requiring turret because we are using it as read only
    // requiring it would prvent shooting if another command is working on turret
  }

  @Override
  public void execute() {
    if (turret.isAtSpeed())
      trigger.shoot();
    else
      trigger.stop();
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
