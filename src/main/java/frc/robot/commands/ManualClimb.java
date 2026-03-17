
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

public class ManualClimb extends Command {
  private Elevator climb;
  private boolean goUp;

  public ManualClimb(Elevator climber, boolean goUp) {
    this.climb = climber;
    this.goUp = goUp;
    addRequirements(climb);
  }

  //maybe explain what both values mean.
  @Override
  public void initialize() {
    climb.driveAtSpeed(goUp ? 0.30 : -0.60); //-0.12
  }

  @Override
  public void end(boolean interrupted)
  {
    climb.holdPosition();
  }

  @Override
  public boolean isFinished() {
    return false; //Math.abs(climb.getPosition().magnitude() - Elevator.Setpoint.Ground.target.magnitude()) <= Constants.Climber.threshold; // meeseeks
  }
}
