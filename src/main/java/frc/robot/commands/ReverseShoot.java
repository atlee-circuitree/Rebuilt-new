package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trigger;
import frc.robot.subsystems.Turret;

public class ReverseShoot extends Command {
  private Turret turret;
  private Trigger trigger;
  
  public ReverseShoot(Turret turret, Trigger trigger) {
    this.turret = turret;
    this.trigger = trigger;
    SmartDashboard.putNumber("Current Turret Speed", 0);
    // not requiring turret because we are using it as read only
    // requiring it would prvent shooting if another command is working on turret
  }

  @Override
  public void initialize()
  {
    System.out.println("Shooter speed " + turret.getVelocity());
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
