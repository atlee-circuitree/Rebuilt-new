package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

/** Turns the turret to the HUB tag AND holds a fixed flywheel speed, as one command (one turret requirement). */
public class AimAndSpinToSpeed extends Command {
  private final Turret turret;
  private final double speed;

  public AimAndSpinToSpeed(Turret turret, double speed) {
    this.turret = turret;
    this.speed = speed;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Turret Required Speed", speed);
  }

  @Override
  public void execute() {
    turret.autoRotate();
    turret.spin(speed);
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