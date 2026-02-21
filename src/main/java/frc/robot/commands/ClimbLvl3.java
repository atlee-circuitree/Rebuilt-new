
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ClimbLvl3 extends Command {
  private Climber climb;

  public ClimbLvl3(Climber climber) {
    climb = climber;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.setPosition(Constants.Climber.lvl3Position);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(climb.getPosition() - Constants.Climber.lvl3Position) < Constants.Climber.climbThreshold;
  }
}
