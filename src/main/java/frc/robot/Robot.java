// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.util.Limelight;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;

  // Cached NetworkTable references to avoid repeated traversal in robotPeriodic()
  private final NetworkTable m_limelightTurretTable =
      NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.LEFT_LIMELIGHT_NAME);
  private final NetworkTable m_limelightLeftTable =
      NetworkTableInstance.getDefault().getTable(Constants.LimelightConstants.FRONT_LIMELIGHT_NAME);

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Limelight Turret Connected",
        m_limelightTurretTable.containsKey("tx"));
    SmartDashboard.putBoolean("Limelight Left Connected",
        m_limelightLeftTable.containsKey("tx"));
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    RobotContainer.getField().setRobotPose(m_robotContainer.getCurrentPose());
    SmartDashboard.putBoolean("Limelight Distance Far Check", (80 < Limelight.getDistance() && Limelight.getDistance() < 95));
    SmartDashboard.putNumber("Limelight Distance", Limelight.getDistance()); //close 49-57
    //limelight settins for field Hickory, 160 exposure
    SmartDashboard.putBoolean("Limelight Distance Close Check", (49 < Limelight.getDistance() && Limelight.getDistance() < 57));
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.startTeleop();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}