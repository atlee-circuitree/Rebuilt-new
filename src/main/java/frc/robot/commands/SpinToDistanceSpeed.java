package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class SpinToDistanceSpeed extends Command {
  private Turret turret;

  public SpinToDistanceSpeed(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.spinAtDistance();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}