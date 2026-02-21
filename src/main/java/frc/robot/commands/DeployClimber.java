
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class DeployClimber extends Command {
  private Climber climb;
  private boolean out;

  public DeployClimber(Climber climber, boolean out) {
    climb = climber;
    this.out = out;
    addRequirements(climb);
  }

  @Override
  public void initialize() {
    climb.setDeploy(out);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
