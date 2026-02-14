
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class SpinToSpeed extends Command {
  private Turret turret;
  private double speed;
  
  public SpinToSpeed(Turret turret, double speed) {
    this.turret = turret;  
    this.speed = speed;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.spin(speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
