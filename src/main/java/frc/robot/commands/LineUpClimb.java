// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LineUpClimb extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric drive;
  private Pose2d pose;
  private double maxSpeed;
  private double maxRotation;
  private double P = 0.2;
  private double threshold = 0.2;

  public LineUpClimb(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    pose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d cur = drivetrain.getState().Pose;
    double x = (cur.getX() - pose.getX()) * P;
    double y = (cur.getY() - pose.getY()) * P;
    double r = (cur.getRotation().getDegrees() - pose.getRotation().getDegrees()) * P;

    drivetrain.setControl(drive.withVelocityX(x * maxSpeed)
                        .withVelocityY(y * maxSpeed)
                        .withRotationalRate(r * maxRotation)
    );
  }

  @Override
  public boolean isFinished() {
    Pose2d cur = drivetrain.getState().Pose;
    return Math.abs(cur.getX() - pose.getX()) < threshold && 
      Math.abs(cur.getY() - pose.getY()) < threshold &&
      Math.abs(cur.getRotation().getDegrees() - pose.getRotation().getDegrees()) < threshold;
  }
}
