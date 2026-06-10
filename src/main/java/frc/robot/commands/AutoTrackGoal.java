package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class AutoTrackGoal extends Command {
  private Turret turret;

  public AutoTrackGoal(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    turret.rotateTo(turret.getAngleToGoal());
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
