
package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

// this is for use in while true buttons only
public class SpinToSpeedInterrupt extends Command {
  private Turret turret;
  private double speed;
  
  public SpinToSpeedInterrupt(Turret turret, double speed) {
    this.turret = turret;
    this.speed = speed;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    SmartDashboard.putNumber("Turret Required Speed", speed);
    turret.spin(speed);
  }

  @Override
  public void end(boolean interrupted)
  {
    turret.stopShooter();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
