package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;

public class SpinToDistanceSpeed extends Command {
  private Turret turret;

  public SpinToDistanceSpeed(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    double distance = Limelight.getDistance();
    if (distance <= 0) {
      turret.spin(Constants.Turret.SPEED_MID_RPS); // fallback: no limelight target
    } else {
      turret.spinAtDistance();
    }
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
