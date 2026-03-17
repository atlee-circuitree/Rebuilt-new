// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class LineUpClimb extends Command {

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.FieldCentric drive;
  private final Pose2d pose;
  private final double maxSpeed    = Constants.Drive.MAX_SPEED_MPS;
  private final double maxRotation = Constants.Drive.MAX_ANGULAR_RATE_RPS;

  public LineUpClimb(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.drive = new SwerveRequest.FieldCentric();
    pose = new Pose2d(new Translation2d(0, 0), new Rotation2d(0));
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Pose2d cur = drivetrain.getState().Pose;
    double x = (cur.getX() - pose.getX()) * Constants.Climber.LINEUP_P;
    double y = (cur.getY() - pose.getY()) * Constants.Climber.LINEUP_P;
    double r = (cur.getRotation().getDegrees() - pose.getRotation().getDegrees()) * Constants.Climber.LINEUP_P;

    drivetrain.setControl(drive.withVelocityX(x * maxSpeed)
                        .withVelocityY(y * maxSpeed)
                        .withRotationalRate(r * maxRotation)
    );
  }

  @Override
  public boolean isFinished() {
    Pose2d cur = drivetrain.getState().Pose;
    return Math.abs(cur.getX() - pose.getX()) < Constants.Climber.LINEUP_THRESHOLD_M &&
      Math.abs(cur.getY() - pose.getY()) < Constants.Climber.LINEUP_THRESHOLD_M &&
      Math.abs(cur.getRotation().getDegrees() - pose.getRotation().getDegrees()) < Constants.Climber.LINEUP_HEADING_THRESHOLD_DEG;
  }
}
