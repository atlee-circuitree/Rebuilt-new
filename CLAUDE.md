# CLAUDE.md — FRC Robot Project (Team 1599B)

## Project Overview

This is an FRC (FIRST Robotics Competition) Java project using WPILib and CTRE Phoenix 6.
The robot is a swerve-drive shooter bot. All robot code follows WPILib command-based conventions
(Commands, Subsystems, etc.).

Key libraries:
- **WPILib** — command-based framework
- **CTRE Phoenix 6** — motor controllers (TalonFX), Pigeon 2 IMU, swerve drivetrain
- **PathPlanner** — autonomous path following

Build/deploy: `./gradlew build` to compile, deploy via WPILib VS Code extension.

---

## Vendor Documentation & Best Practices

### WPILib
- [WPILib Docs](https://docs.wpilib.org/en/stable/) — main reference for all WPILib APIs
- [Command-Based Programming](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html) — how Commands and Subsystems work; follow this pattern for all new commands
- [Subsystem Best Practices](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html) — default commands, periodic(), requiresSubsystem
- [PID Control](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html) — WPILib PID reference (for non-CTRE motors)
- [SmartDashboard / Shuffleboard](https://docs.wpilib.org/en/stable/docs/software/dashboards/index.html) — putting data on the dashboard
- [Field2d Widget](https://docs.wpilib.org/en/stable/docs/software/dashboards/glass/field2d-widget.html) — used by our `field` object in RobotContainer

### CTRE Phoenix 6
- [Phoenix 6 Docs](https://v6.docs.ctr-electronics.com/en/stable/) — main reference for TalonFX, CANcoder, Pigeon 2
- [TalonFX Configuration](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/configuration.html) — how to configure motors (slots, limits, etc.)
- [Motion Magic / PID Slots](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/closed-loop-requests.html) — closed-loop control; we use Slot 0 for rotator and flywheel PID
- [Swerve Best Practices](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/tuner-swerve/index.html) — CTRE swerve setup and tuning; our drivetrain is generated from Tuner X
- [Phoenix Tuner X](https://pro.docs.ctr-electronics.com/en/stable/docs/tuner/index.html) — used for CAN ID assignment, firmware updates, live tuning
- [SwerveRequest API](https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/swerve/SwerveRequest.html) — `FieldCentric`, `SwerveDriveBrake`, `PointWheelsAt` used in RobotContainer

### PathPlanner
- [PathPlanner Docs](https://pathplanner.dev/home.html) — main reference for path creation and auto building
- [Named Commands](https://pathplanner.dev/pplib-named-commands.html) — how we register commands in `mapEventsToCommands()`; all PathPlanner event markers must be registered here before `AutoBuilder` runs
- [AutoBuilder Setup](https://pathplanner.dev/pplib-auto-builder.html) — how `AutoBuilder.buildAutoChooser()` works (used when switching back to PathPlanner autos)
- [Holonomic Path Following](https://pathplanner.dev/pplib-holonomic.html) — swerve-specific path following config

### Limelight (Vision)
- [Limelight Docs](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary) — setup and configuration
- [NetworkTables API](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib) — how to read `tx`, `ty`, `ta`, `tv` from the Limelight; used in `TurnTurret` and distance calculations
- [Retroreflective Pipeline Setup](https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/targeting-and-crosshairs) — crosshair calibration and targeting for retroreflective tape; relevant when adjusting how `tx`/`ty` are zeroed for turret aiming
- [AprilTag Pipeline Setup](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-pipeline-settings) — if the game uses AprilTag targets instead of retroreflective tape, switch to this pipeline type
- [Latency & Pose Estimation](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization) — how to use Limelight MegaTag for odometry correction; useful if we want vision-assisted field localization during teleop or auto

---

## Community Resources

- [Chief Delphi](https://www.chiefdelphi.com/) — FRC community forum; search here for solutions to hardware/software problems before asking elsewhere
- [WPILib GitHub](https://github.com/wpilibsuite/allwpilib) — source and issue tracker for WPILib bugs
- [CTRE Phoenix 6 GitHub](https://github.com/CrossTheRoadElec/Phoenix6-Examples) — official CTRE code examples

---

## Project Structure

```
src/main/java/frc/robot/
├── Robot.java                  # Main robot lifecycle (autonomousInit, teleopInit, etc.)
├── RobotContainer.java         # Subsystem instantiation, button bindings, auto chooser
├── AutoCommands.java           # All autonomous routines (SendableChooser)
├── Constants.java              # All tunable constants organized by subsystem
├── Telemetry.java              # Drivetrain telemetry logger
├── commands/                   # One file per WPILib Command
└── subsystems/                 # One file per subsystem
    ├── CommandSwerveDrivetrain.java  — CTRE swerve drive, field-centric, PathPlanner integrated
    ├── Turret.java                   — Flywheel shooter (left/right TalonFX) + rotator + hood servo
    ├── Intake.java                   — Deploy motor (PID position) + feed wheel motor
    ├── Trigger.java                  — Index motor + kickup motor for feeding balls to shooter
    └── Elevator.java                 — Climber; currently disabled/commented out
```

---

## Key Constants (all in `Constants.java`)

- All CAN IDs → `Constants.CAN_IDS`
- Flywheel speeds (RPS) → `Constants.Turret.SPEED_CLOSE_RPS / SPEED_MID_RPS / SPEED_FAR_RPS`
- Intake deploy/retract positions → `Constants.Intake.DEPLOY_POS_ROT / RETRACT_POS_ROT`
- Drive speed/deadband → `Constants.Drive`
- Limelight settings → `Constants.LimelightConstants`
- Field hub positions → `Constants.Field.BLUE_HUB / RED_HUB`

When tuning motor speeds or timing values, **always change the constant in `Constants.java`**,
not hardcoded values in commands.

---

## Controllers

- **Player 1** — `CommandXboxController` (port 0) — primary driver
- **Player 2** — `CommandXboxController` (port 1) — operator (duplicate bindings)
- **Player 3** — `GenericHID` (port 3) — Stadia controller (drive + operator actions)

Stadia button mapping is documented in `RobotContainer.java` lines 76–90.

---

## Commands

When the user asks for a "command", they mean a **WPILib Command class** (extending `Command`,
or using `SequentialCommandGroup`, `ParallelCommandGroup`, `RunCommand`, etc.) — NOT a button
binding or inline lambda.

Existing commands in `commands/`:

| Command | Purpose |
|---------|---------|
| `SpinToSpeed` | Spin flywheel to a fixed RPS setpoint |
| `SpinToSpeedInterrupt` | Same but interruptible |
| `SpinToDistanceSpeed` | Set flywheel speed based on Limelight distance |
| `TurnTurret` | Auto-aim turret using Limelight tx |
| `AutoTurret` | Combined auto-aim + shoot |
| `Shoot` | Run trigger/kickup motors to fire |
| `AutoShoot` | Shoot with speed-ready gate |
| `StopTurretWheels` | Stop flywheel |
| `RunIntake` | Run intake feed wheels |
| `DeployIntake` | Deploy intake to down position (PID) |
| `RetractIntake` | Retract intake to up position (PID) |
| `ManualDeploy` | Run deploy motor at duty cycle |
| `DeployJumpCommand` | Oscillate intake to help feed balls |
| `ReverseShoot` | Reverse trigger motors (unjam) |
| `ZeroTurret` | Home turret rotator to hard stop |

---

## Autonomous

Autos are defined in `AutoCommands.java` and selected via `SendableChooser` on SmartDashboard.

Current auto options:
- Just Shoot
- Center Shoot (RED / BLUE)
- Human Player Shoot (RED / BLUE)
- Feeder Shoot

PathPlanner named commands are registered in `RobotContainer.mapEventsToCommands()`.

---

## File Edit Permissions

### Do NOT edit these files
These are auto-generated or otherwise locked — changes will be overwritten or break things:

| File | Reason |
|------|--------|
| `Telemetry.java` | Auto-generated drivetrain telemetry logger |
| `subsystems/CommandSwerveDrivetrain.java` | Auto-generated by CTRE Tuner X |
| `generated/TunerConstants.java` | Auto-generated by CTRE Tuner X |

### Safe to edit
Everything else in `src/main/java/frc/robot/` is fair game, including:
- `Robot.java`, `RobotContainer.java`, `AutoCommands.java`, `Constants.java`
- All files in `commands/` and subsystems not listed above

---

## Workflow Rules

1. **When asked to fix or modify a specific file, work on THAT file only.** Do not touch other
   files unless explicitly asked.
2. **When tuning a parameter**, change the constant in `Constants.java`, not hardcoded values.
3. **When adding a new autonomous routine**, add it to `AutoCommands.java` and register any new
   named commands in `RobotContainer.mapEventsToCommands()`.
4. **Check `Robot.java`** if the robot isn't behaving in a mode — commonly caused by commented-out
   lifecycle methods (`autonomousInit`, `teleopInit`, etc.).
