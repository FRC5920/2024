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

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LED.ColorConstants;
import frc.robot.Constants.CANDevice;
import frc.robot.autos.AutoDashboardTab;
import frc.robot.commands.TeleopSwerveCTRE;
import frc.robot.commands.autoCommands.ShootAmpClose;
import frc.robot.commands.autoCommands.ShootSpeakerClose;
import frc.robot.subsystems.JoystickSubsystem;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystemIO;
import frc.robot.subsystems.climber.ClimberSubsystemIOReal;
import frc.robot.subsystems.climber.ClimberSubsystemIOSim;
import frc.robot.subsystems.dashboard.DashboardSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystem;
import frc.robot.subsystems.flywheel.FlywheelSubsystemIO;
import frc.robot.subsystems.flywheel.FlywheelSubsystemIOReal;
import frc.robot.subsystems.flywheel.FlywheelSubsystemIOSim;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystemIO;
import frc.robot.subsystems.indexer.IndexerSubsystemIOReal;
import frc.robot.subsystems.indexer.IndexerSubsystemIOSim;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystemIO;
import frc.robot.subsystems.pivot.PivotSubsystemIOReal;
import frc.robot.subsystems.pivot.PivotSubsystemIOSim;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;
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

  /** Flywheel subsystem */
  public final FlywheelSubsystem flywheelSubsystem;

  /** Indexer subsystem */
  public final IndexerSubsystem indexerSubsystem;

  // Subsystem facilitating display of dashboard tabs
  public final DashboardSubsystem dashboardSubsystem = new DashboardSubsystem();

  private final AutoDashboardTab autoDashboardTab;

  // Register an object to receive telemetry from the swerve drive
  // public final CTRESwerveTelemetry swerveTelemetry = new CTRESwerveTelemetry();

  // Subsystem used to drive addressable LEDs
  public final LEDSubsystem ledSubsystem = new LEDSubsystem(ColorConstants.kOff);

  /** Swerve request used to process robot-centric motion commands from the AutoBuilder */
  private final SwerveRequest.ApplyChassisSpeeds m_autochassisSpeedsReq =
      new SwerveRequest.ApplyChassisSpeeds();

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Called to create the robot container */
  public RobotContainer() {

    // Configure the PathPlanner AutoBuilder, the set up the auto dashboard tab.
    // NOTE: these must occur in this order
    configureAutoBuilder();
    autoDashboardTab = new AutoDashboardTab();

    ClimberSubsystemIO climberIO = null;
    FlywheelSubsystemIO flywheelIO = null;
    IndexerSubsystemIO indexerIO = null;
    PivotSubsystemIO pivotIO = null;

    switch (Constants.getMode()) {
      case REAL:
        climberIO = new ClimberSubsystemIOReal(new ClimberSubsystemIO.Config());
        flywheelIO = new FlywheelSubsystemIOReal(CANDevice.IntakeFlywheelMotor);
        indexerIO =
            new IndexerSubsystemIOReal(
                CANDevice.IntakeIndexerMotor, CANDevice.IntakeGamepieceSensor);
        pivotIO = new PivotSubsystemIOReal(new PivotSubsystemIO.Config());
        break;

      case SIM:
        climberIO = new ClimberSubsystemIOSim(new ClimberSubsystemIO.Config());
        flywheelIO = new FlywheelSubsystemIOSim(CANDevice.IntakeFlywheelMotor);
        indexerIO =
            new IndexerSubsystemIOSim(
                CANDevice.IntakeIndexerMotor, CANDevice.IntakeGamepieceSensor);
        pivotIO = new PivotSubsystemIOSim(new PivotSubsystemIO.Config());
        break;

      case REPLAY:
        // Create empty implementations for log replay
        climberIO = new ClimberSubsystemIO() {};
        flywheelIO = new FlywheelSubsystemIO() {};
        indexerIO = new IndexerSubsystemIO() {};
        pivotIO = new PivotSubsystemIO() {};
        break;
    }

    // Create the climber subsystem
    climberSubsystem = new ClimberSubsystem(climberIO);
    climberSubsystem.setDefaultCommand(
        new ClimberSubsystem.ClimberJoystickTeleOp(
            climberSubsystem, joystickSubsystem.getOperatorController()));

    // Create the flywheel subsystem
    flywheelSubsystem = new FlywheelSubsystem(flywheelIO);

    // Create the indexer subsystem
    indexerSubsystem = new IndexerSubsystem(indexerIO);

    // Create the pivot subsystem
    pivotSubsystem = new PivotSubsystem(pivotIO);

    joystickSubsystem.configureButtonBindings(this);

    // Set up a command to drive the swerve in Teleoperated mode
    driveTrain.setDefaultCommand(
        new TeleopSwerveCTRE(driveTrain, joystickSubsystem.getDriverController()));

    // Tell the swerve drive subsystem to update our telemetry from its odometry thread
    // driveTrain.registerTelemetry(swerveTelemetry::update);

    // Add the field dashboard tab
    dashboardSubsystem.add(autoDashboardTab);

    // if (Utils.isSimulation()) {
    //   driveTrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /** Returns the present autonomous command */
  public Command getAutonomousCommand() {
    return autoDashboardTab.getCurrentAutoRoutine();
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////
  /**
   * Configures the PathPlanner AutoBuilder
   *
   * @remarks This should be called after the swerve drive subsystem is constructed.
   */
  private void configureAutoBuilder() {

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        driveTrain::getPose, // Robot pose supplier
        driveTrain
            ::seedFieldRelative, // Method to reset odometry (will be called if your auto has a
        // starting pose)
        driveTrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds) ->
            driveTrain.setControl(
                m_autochassisSpeedsReq.withSpeeds(
                    speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
            // your Constants class
            new PIDConstants(10.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(10.0, 0.0, 0.0), // Rotation PID constants
            TunerConstants.kSpeedAt12VoltsMps, // Max module speed, in m/s
            driveTrain
                .getDriveBaseRadius(), // Drive base radius in meters. Distance from robot center to
            // furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options
            // here
            ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        driveTrain // Reference to this subsystem to set requirements
        );

    // Register named commands
    NamedCommands.registerCommand("ShootAmpClose", new ShootAmpClose());
    NamedCommands.registerCommand("ShootSpeakerClose", new ShootSpeakerClose());
  }
}
