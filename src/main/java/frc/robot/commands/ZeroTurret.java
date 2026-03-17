// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class ZeroTurret extends Command {
  private Turret turret;

  public ZeroTurret(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.clearZeroCurrents();
    turret.findZero();
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
    turret.setZero();
  }

  @Override
  public boolean isFinished() {
    return turret.zeroPeriodic();
  }
}
