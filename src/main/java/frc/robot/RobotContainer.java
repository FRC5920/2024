////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2024 FIRST and other WPILib contributors.
// http://github.com/FRC5920
// Open Source Software; you can modify and/or share it under the terms of the
// license given in WPILib-License.md in the root directory of this project.
////////////////////////////////////////////////////////////////////////////////

/*-----------------------------------------------------------------------------\
|                                                                              |
|                       ================================                       |
|                       **    TEAM 5920 - Vikotics    **                       |
|                       ================================                       |
|                                                                              |
|                            °        #°                                       |
|                            *O       °@o                                      |
|                            O@ °o@@#° o@@                                     |
|                           #@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@@                                    |
|                           @@@@@@@@@@@@@@°                                    |
|                             #@@@@@@@@@@@@@O....   .                          |
|                             o@@@@@@@@@@@@@@@@@@@@@o                          |
|                             O@@@@@@@@@@@@@@@@@@@#°                    *      |
|                             O@@@@@@@@@@@@@@@@@@@@@#O                O@@    O |
|                            .@@@@@@@@°@@@@@@@@@@@@@@@@#            °@@@    °@@|
|                            #@@O°°°°  @@@@@@@@@@@@@@@@@@°          @@@#*   @@@|
|                         .#@@@@@  o#oo@@@@@@@@@@@@@@@@@@@@@.       O@@@@@@@@@@|
|                        o@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@°     @@@@@@@@@°|
|                        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@   .@@@@@o°   |
|          °***          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@  @@@@@o     |
|     o#@@@@@@@@@@@@.   *@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@o@@@@@@      |
|OOo°@@@@@@@@@@@@O°#@#   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@@    o°  .@@@@@@@@@@@@@@@@@@@@@@@@#*@@@@@@@@@@@@@@@@@@@@       |
|@@@@@@@@@@@@@@@*         O@@@@@@@@@@@@@@@@@@@@@@@   °@@@@@@@@@@@@@@@@@@o      |
|@@@@#@@@@@@@@@            @@@@@@@@@@@@@@@@@@@@@@       .*@@@@@@@@@@@@@@.      |
|@@@°      @@@@O           @@@@@@@@@@@@@@@@@@@@o           °@@@@@@@@@@@o       |
|          @@@@@          .@@@@@@@@@@@@@@@@@@@*               O@@@@@@@*        |
|           @@@@@        o@@@@@@@@@@@@@@@@@@@@.               #@@@@@O          |
|           *@@@@@@@*  o@@@@@@@@@@@@@@@@@@@@@@°              o@@@@@            |
|           @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.              @@@@@#            |
|          @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@O             #@@@@@             |
|          .@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#           .@@@@@°             |
|           @@@@@@@@@@O*    @@@@@@@@@@@@@@@@@@@@@°         °O@@@°              |
|            °O@@@@@@       @@@@@@@@@@@@@@@@@@@@@@@                            |
|              o@@@@@°      @@@@@@@@@@@@@@@@@@@@@@@@                           |
|               @@@@@@.     @@@@@@@@@@@@@@@@@@@@@@@@@o                         |
|                @@@@@@*    @@@@@@@@@@@@@@@@@@@@@@@@@@                         |
|                o@@@@@@.  o@@@@@@@@@@@@@@@@@@@@@@@@@@@                        |
|                 #@@@@@@  *@@@@@@@@@@@@@@@@@@@@@@@@@@@@                       |
|                  °***    @@@@@@@@@@@@@@@@@@@@@@@@@@@@@O                      |
|                         .OOOOOOOOOOOOOOOOOOOOOOOOOOOOOO                      |
\-----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.LED.ColorConstants;
import frc.robot.autos.AutoDashboardTab;
import frc.robot.commands.TeleopSwerveCTRE;
import frc.robot.commands.intakeCommands.TeleopIntakeTest;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystemIO;
import frc.robot.subsystems.climber.ClimberSubsystemIOReal;
import frc.robot.subsystems.climber.ClimberSubsystemIOSim;
import frc.robot.subsystems.dashboard.DashboardSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemIO;
import frc.robot.subsystems.intake.IntakeSubsystemIOReal;
import frc.robot.subsystems.intake.IntakeSubsystemIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystemIO;
import frc.robot.subsystems.pivot.PivotSubsystemIOReal;
import frc.robot.subsystems.pivot.PivotSubsystemIOSim;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerveCTRE.Telemetry;
import frc.robot.subsystems.swerveCTRE.TunerConstants;

public class RobotContainer {

  /** Subsystem providing Xbox controllers */
  public final JoystickSubsystem joystickSubsystem = new JoystickSubsystem();

  /** Swerve drive subsystem */
  public final CommandSwerveDrivetrain driveTrain = TunerConstants.DriveTrain;

  /** Pivot subsystem */
  public final PivotSubsystem pivotSubsystem;

  /** Climber subsystem */
  public final ClimberSubsystem climberSubsystem;

  /** Intake subsystem */
  public final IntakeSubsystem intakeSubsystem;

  // Subsystem facilitating display of dashboard tabs
  public final DashboardSubsystem dashboardSubsystem = new DashboardSubsystem();

  private final AutoDashboardTab autoDashboardTab = new AutoDashboardTab();

  // driving in open loop
  public final Telemetry swerveTelemetry = new Telemetry(TeleopSwerveCTRE.kMaxSpeed);

  // Subsystem used to drive addressable LEDs
  public final LEDSubsystem ledSubsystem = new LEDSubsystem(ColorConstants.kOff);

  /** Called to create the robot container */
  public RobotContainer() {
    ClimberSubsystemIO climberIO = null;
    IntakeSubsystemIO intakeIO = null;
    PivotSubsystemIO pivotIO = null;

    switch (Constants.getMode()) {
      case REAL:
        climberIO = new ClimberSubsystemIOReal(new ClimberSubsystemIO.Config());
        intakeIO = new IntakeSubsystemIOReal(new IntakeSubsystemIO.Config());
        pivotIO = new PivotSubsystemIOReal(new PivotSubsystemIO.Config());
        break;

      case SIM:
        climberIO = new ClimberSubsystemIOSim(new ClimberSubsystemIO.Config());
        intakeIO = new IntakeSubsystemIOSim(new IntakeSubsystemIO.Config());
        pivotIO = new PivotSubsystemIOSim(new PivotSubsystemIO.Config());
        break;

      case REPLAY:
        // Create empty implementations for log replay
        climberIO = new ClimberSubsystemIO() {};
        intakeIO = new IntakeSubsystemIO() {};
        break;
    }

    // Create the climber subsystem
    climberSubsystem = new ClimberSubsystem(climberIO);

    // Create the intake subsystem
    intakeSubsystem = new IntakeSubsystem(intakeIO);
    intakeSubsystem.setDefaultCommand(new TeleopIntakeTest(intakeSubsystem, joystickSubsystem));

    // Create the pivot subsystem
    pivotSubsystem = new PivotSubsystem(pivotIO);

    joystickSubsystem.configureButtonBindings(this);
    // Set up a command to drive the swerve in Teleoperated mode
    driveTrain.setDefaultCommand(
        new TeleopSwerveCTRE(driveTrain, joystickSubsystem.getDriverController()));

    // Register a function to be called to receive swerve telemetry
    driveTrain.registerTelemetry(swerveTelemetry::telemeterize);

    // Add the field dashboard tab
    dashboardSubsystem.add(autoDashboardTab);

    // if (Utils.isSimulation()) {
    //   driveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
  }

  /** Returns the present autonomous command */
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
