// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

    // subsystems
    private Elevator climber;
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private Intake intake;
    private Trigger trigger;
    private Turret turret;

    // drivetrain
    private static final Field2d field = new Field2d();

    public static Field2d getField() {
        return field;
    }
    
    public Pose2d getCurrentPose() {
        return drivetrain.getState().Pose;
    }

    private final SendableChooser<Command> autoChooser;


    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4; // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * Constants.Drive.DEADBAND_PERCENT).withRotationalDeadband(MaxAngularRate * Constants.Drive.DEADBAND_PERCENT)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);
    

    // IO devices
    private final CommandXboxController Player1 = new CommandXboxController(0);
    private final CommandXboxController Player2 = new CommandXboxController(1);


    public RobotContainer() {

      
        climber = new Elevator();
        intake = new Intake();
        trigger = new Trigger();
        turret = new Turret();
        
        //AUTO CHOOSER CREATION.
        autoChooser = AutoCommands.buildAutoChooser(drivetrain, drive, MaxSpeed, MaxAngularRate, turret, trigger, intake);

        SmartDashboard.putData("Field", field);

        // Do all other initialization
        configureBindings();
        SmartDashboard.putNumber("velocity", 1.0);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        mapEventsToCommands();
    }

    public void mapEventsToCommands()
    {
        // replace null with command instance
        // do this for all commands
        NamedCommands.registerCommand("intake", new RunIntake(intake).withTimeout(3.0));
        NamedCommands.registerCommand("shoot",  new SpinToDistanceSpeed(turret));
        NamedCommands.registerCommand("shoot distance", new SequentialCommandGroup(
            new SpinToDistanceSpeed(turret),
            new AutoShoot(turret, trigger)
        ));
        NamedCommands.registerCommand("kickup", new Shoot(turret, trigger).withTimeout(12.0));
        NamedCommands.registerCommand("auto shoot", new AutoShoot(turret, trigger));
        NamedCommands.registerCommand("deploy intake", new DeployIntake(intake));
        NamedCommands.registerCommand("retract intake", new RetractIntake(intake));
        NamedCommands.registerCommand("auto turret", new AutoTurret(turret, trigger, drivetrain).withTimeout(5.0));
        NamedCommands.registerCommand("line up climb", new LineUpClimb(drivetrain));
        NamedCommands.registerCommand("climb up", new ClimbUp(climber));
        NamedCommands.registerCommand("climb down", new ClimbDown(climber));
    }

    public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
        return autoChooser.getSelected();
    }

    private void configureDrivetrain() {
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
        Player1.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-Player1.getLeftY(), -Player1.getLeftX()))
        ));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void startTeleop()
    {
        turret.stopRotator();
        turret.stopShooter();
        intake.stopWheels();
        intake.stopDeploy();
    }

    //player 1 and 2 stick getters.

    public double getLeftY() 
    {
        double player1 = Player1.getLeftY();
        double player2 = Player2.getLeftY();

        if (Math.abs(player1) > Math.abs(player2)) {return player1;}
        return player2;
    }

    public double getLeftX() 
    {
        double player1 = Player1.getLeftX();
        double player2 = Player2.getLeftX();

        if (Math.abs(player1) > Math.abs(player2)) {return player1;}
        return player2;
    }

    public double getRightX() 
    {
        double player1 = Player1.getRightX();
        double player2 = Player2.getRightX();

        if (Math.abs(player1) > Math.abs(player2)) {return player1;}
        return player2;
    }

    public void Periodic() {
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.localize(drivetrain);
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        drivetrain.addVisionMeasurement(
            mt2.pose, 
            mt2.timestampSeconds);
    }
    
    private void configureBindings() {
        configureDrivetrain();
 drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> 
                drive.withVelocityX(-(getLeftY()) * ((Constants.Drive.MAX_SPEED_MPS) / Constants.Drive.SPEED_DIVISOR)) // Drive forward with negative Y (forward)
                    .withVelocityY(-(getLeftX()) * ((Constants.Drive.MAX_SPEED_MPS) / Constants.Drive.SPEED_DIVISOR)) // Drive left with negative X (left)
                    .withRotationalRate(-(getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

    

        
        // drivetrain.seedFieldCentric();
        Player1.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        Player2.start().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Button Map

        Player1.x().onTrue(new DeployIntake(intake)); // deploy
        Player1.y().onTrue(new RetractIntake(intake)); // retract
        Player1.a().whileTrue(new ManualDeploy(intake, Constants.Intake.DEPLOY_MANUAL_SPEED)); // down
        Player1.b().whileTrue(new ManualDeploy(intake, -Constants.Intake.DEPLOY_MANUAL_SPEED)); // up

        Player1.rightTrigger().whileTrue(new ParallelCommandGroup(
            new DeployJumpCommand(intake),
            new SpinToSpeedInterrupt(turret, Constants.Turret.SPEED_FAR_RPS),
            new Shoot(turret, trigger)
        )); // shoot and kick up, shooter first then kickup

        Player1.povUp().onTrue(new StopTurretWheels(turret));
        Player1.rightBumper().whileTrue(new ReverseShoot(trigger));

        Player1.leftTrigger().whileTrue(new RunIntake(intake)); // intake in
        Player1.leftBumper().whileTrue(new TurnTurret(turret));

        //Player 2 controls
        Player2.x().onTrue(new DeployIntake(intake)); // deploy
        Player2.y().onTrue(new RetractIntake(intake)); // retract
        Player2.a().whileTrue(new ManualDeploy(intake, Constants.Intake.DEPLOY_MANUAL_SPEED)); // down
        Player2.b().whileTrue(new ManualDeploy(intake, -Constants.Intake.DEPLOY_MANUAL_SPEED)); // up

        Player2.rightTrigger().whileTrue(new ParallelCommandGroup(
            new DeployJumpCommand(intake),
            new SpinToSpeedInterrupt(turret, Constants.Turret.SPEED_FAR_RPS),
            new Shoot(turret, trigger)
        )); // shoot and kick up, shooter first then kickup

        Player2.povUp().onTrue(new StopTurretWheels(turret));
        Player2.rightBumper().whileTrue(new ReverseShoot(trigger));

        Player2.leftTrigger().whileTrue(new RunIntake(intake)); // intake in

        //TODO: manual hood, and turret rotator
        Player2.leftBumper().toggleOnTrue(new TurnTurret(turret));
    }   

}
