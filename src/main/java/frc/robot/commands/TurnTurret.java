// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TurnTurret extends Command {
  private Turret turret;
  private final Timer timer = new Timer();
  private final Timer sweepTimer = new Timer();
  private double sweepSpeed;
  private static final double SWEEP_PERIOD_S = 1.5;

  /** Creates a new MoveTurret. */
  public TurnTurret(Turret tur) {
    // Use addRequirements() here to declare subsystem dependencies.
    turret = tur;
    addRequirements(tur);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    sweepTimer.restart();
    sweepSpeed = 0.1;
  }

  // Called every scheduler cycle while the command is running.
  @Override
  public void execute() {
    if (Limelight.getTurretTV()) {
      turret.autoRotate();
    } else {
      if (sweepTimer.hasElapsed(SWEEP_PERIOD_S)) {
        sweepSpeed = -sweepSpeed;
        sweepTimer.restart();
      }
      turret.rotate(sweepSpeed);
    }
  }

  @Override
  public void end(boolean interrupted) {
    turret.stopRotator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Limelight.getTurretTV() && Math.abs(Limelight.getTX()) <= 5.0) || timer.hasElapsed(7.0);
  }
}
