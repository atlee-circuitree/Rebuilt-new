package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/** Turns the turret to the HUB tag AND sets flywheel speed from distance, in one command. */
public class AimAndSpinAtHub extends Command {
  private final Turret turret;

  public AimAndSpinAtHub(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void execute() {
    turret.autoRotate();
    turret.spinAtDistance();
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
    turret.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}