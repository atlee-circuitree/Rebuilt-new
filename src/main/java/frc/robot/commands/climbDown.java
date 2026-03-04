
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ClimbDown extends Command {
  private Elevator climb;

  public ClimbDown(Elevator climber) {
    climb = climber;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    //climb.goToSetpoint(() -> {return Elevator.Setpoint.Ground;});
  }

  @Override
  public boolean isFinished() {
    return true; //Math.abs(climb.getPosition().magnitude() - Elevator.Setpoint.Ground.target.magnitude()) <= Constants.Climber.threshold; // meeseeks
  }
}
