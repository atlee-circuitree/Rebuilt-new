
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class climbDown extends Command {
  private Climber climb;

  public climbDown(Climber climber) {
    climb = climber;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.setPosition(Constants.Climber.homePosition);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(climb.getPosition() - Constants.Climber.homePosition) < Constants.Climber.climbThreshold;
  }
}
