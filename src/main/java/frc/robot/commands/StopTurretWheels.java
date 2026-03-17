
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class StopTurretWheels extends Command {
  private final Turret turret;

  public StopTurretWheels(Turret turret) {
    this.turret = turret; 
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
