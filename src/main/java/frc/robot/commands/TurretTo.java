// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class TurretTo extends Command {
  private Turret turret;
  private double ang;
  // 4-cycle (80 ms) debounce — command only finishes after holding within tolerance for 4 consecutive cycles
  private final Debouncer m_atAngleDebouncer = new Debouncer(4 * 0.02, Debouncer.DebounceType.kRising);

  public TurretTo(Turret turret, double ang) {
    this.turret = turret;
    this.ang = ang;
    addRequirements(turret);
  }

  @Override
  public void initialize()
  {
    turret.setSetpoint(ang);
  }

  @Override
  public void execute()
  {
    turret.rotateTo();
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
  }

  @Override
  public boolean isFinished() {
    return m_atAngleDebouncer.calculate(Math.abs(turret.getAngle() - ang) < Constants.Turret.ANGLE_THRESHOLD_DEG);
  }
}
