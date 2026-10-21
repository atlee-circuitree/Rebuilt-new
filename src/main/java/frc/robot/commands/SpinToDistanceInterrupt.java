package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/** For whileTrue buttons: keeps the flywheel at the speed for the current Limelight distance. */
public class SpinToDistanceInterrupt extends Command {
  private final Turret turret;

  public SpinToDistanceInterrupt(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.spinAtDistance();
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}