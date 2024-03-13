////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2023 FIRST and other WPILib contributors.
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
package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.lib.joystick.JoystickSubsystemBase;
import frc.lib.joystick.ProcessedXboxController;
import frc.robot.Constants.CameraTarget;
import frc.robot.RobotContainer;
import frc.robot.commands.ArmCommands.ClimberCommand;
import frc.robot.commands.ArmCommands.ClimberCommand.ClimberPreset;
import frc.robot.commands.ArmCommands.IntakeNote;
import frc.robot.commands.ArmCommands.PivotCommand;
import frc.robot.commands.ArmCommands.ZTargetIntake;
import frc.robot.commands.ArmCommands.PivotCommand.PivotPreset;
import frc.robot.commands.DriveWithZTargeting;
import frc.robot.commands.autoCommands.ShootAmpClose;
import frc.robot.commands.autoCommands.ShootSpeakerClose;
import frc.robot.commands.autoCommands.ShootSpeakerReverse;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;

/** A subsystem providing Xbox controllers for driving the robot manually */
public class JoystickSubsystem extends JoystickSubsystemBase {

  /** true to support a second "operator" controller */
  public static final boolean kOperatorControllerIsEnabled = true;

  /** A placeholder command that does nothing for unused button bindings */
  public static final InstantCommand kDoNothing = new InstantCommand();

  // private final SwerveRequest.RobotCentric m_driveForwardStraight =
  //     new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  /** Creates an instance of the subsystem */
  public JoystickSubsystem() {
    super(true, kOperatorControllerIsEnabled);
  }

  /**
   * Call this method to set up mappings between joystick buttons and commands.
   *
   * @param botContainer Object providing access to robot subsystems
   */
  @Override
  public void configureButtonBindings(RobotContainer botContainer) {
    configureDriverControllerBindings(botContainer);

    // Map buttons on operator controller
    if (kOperatorControllerIsEnabled) {
      configureOperatorControllerBindings(botContainer);
    }
  }

  /**
   * Use this method to define the mappings of buttons to commands on the driver controller. Buttons
   * can be created by instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * @param botContainer Object providing access to robot subsystems
   */
  private void configureDriverControllerBindings(RobotContainer botContainer) {
    CommandSwerveDrivetrain driveTrain = botContainer.driveTrain;

    // Map buttons on driver controller
    ProcessedXboxController driverController = getDriverController();

    driverController.A.whileTrue(new ShootAmpClose(botContainer));
    // Map B button to swerve brake command
    driverController.B.whileTrue(
        driveTrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake()));

    // Map POV
    driverController.povUp.onTrue(
        new ClimberCommand(botContainer.climberSubsystem, ClimberPreset.ClimbersUp));
    driverController.povDown.onTrue(
        new ClimberCommand(botContainer.climberSubsystem, ClimberPreset.ClimbersDown));

    driverController.X.whileTrue(kDoNothing);
    driverController.Y.whileTrue(new ShootSpeakerClose(botContainer));

    // // Map right bumper
    // driverController.rightBumper.whileTrue(kDoNothing);

    // // Map back button
    // reset the field-centric heading on left bumper press
    driverController.back.onTrue(driveTrain.runOnce(() -> driveTrain.seedFieldRelative()));
    driverController.leftBumper.whileTrue(new ShootAmpClose(botContainer));
    driverController.rightBumper.whileTrue(new ShootSpeakerReverse(botContainer));

    // driverController.rightBumper.whileTrue(
    //     new RunIndexerAtSpeed.RunIntakeAtSpeed(botContainer.intakeSubsystem,
    // IntakePreset.ShootRing));


    //driverController.leftBumper.whileTrue(
    //    new DriveWithZTargeting(driveTrain, driverController, CameraTarget.AprilTag2D));

    // // Map stick press buttons
    // driverController.leftStickPress.onTrue(kDoNothing);
    // driverController.rightStickPress.onTrue(kDoNothing);

    // // Map BACK button (small button on the left in the middle of the controller)
    // driverController.back.onTrue(kDoNothing);

    // // Map START button (small button on the left in the middle of the controller)
    // driverController.start.whileTrue(kDoNothing);

    // Driver indexer button
    driverController.rightTriggerAsButton.whileTrue(new ZTargetIntake(botContainer, driveTrain, driverController, CameraTarget.GameNote));
  }

  /**
   * Use this method to define the mappings of buttons to commands on the driver controller. Buttons
   * can be created by instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   *
   * @param botContainer Object providing access to robot subsystems
   */
  private void configureOperatorControllerBindings(RobotContainer botContainer) {
    ProcessedXboxController operatorController = getOperatorController();

    // Map buttons
    operatorController.A.onTrue(new PivotCommand(botContainer.pivotSubsystem, PivotPreset.Intake));
    operatorController.B.onTrue(
        new PivotCommand(botContainer.pivotSubsystem, PivotPreset.ShootBackward));
    operatorController.X.onTrue(
        new PivotCommand(botContainer.pivotSubsystem, PivotPreset.ShootAmp));
    operatorController.Y.onTrue(
        new PivotCommand(botContainer.pivotSubsystem, PivotPreset.ShootSpeaker));

    // Map POV
    operatorController.povUp.onTrue(
        new ClimberCommand(botContainer.climberSubsystem, ClimberPreset.ClimbersUp));
    operatorController.povDown.onTrue(
        new ClimberCommand(botContainer.climberSubsystem, ClimberPreset.ClimbersDown));

    // operatorController.leftTriggerAsButton.whileTrue(
    //     new RunIndexerAtSpeed.RunIntakeAtSpeed(
    //         botContainer.intakeSubsystem, IntakePreset.IntakeRing));
    // operatorController.rightTriggerAsButton.whileTrue(
    //     new RunIndexerAtSpeed.RunIntakeAtSpeed(botContainer.intakeSubsystem,
    // IntakePreset.ShootRing));

    // Map bumpers
    operatorController.leftBumper.whileTrue(kDoNothing);
    operatorController.rightBumper.whileTrue(kDoNothing);

    // Map triggers
    operatorController.leftTriggerAsButton.onTrue(kDoNothing);

    // Map stick press
    operatorController.leftStickPress.onTrue(kDoNothing);
    operatorController.rightStickPress.onTrue(kDoNothing);

    // Map small center buttons
    operatorController.back.onTrue(new InstantCommand(() -> botContainer.pivotSubsystem.park()));
    operatorController.start.onTrue(kDoNothing);
  }

  public static class JoystickInputs {
    public double xAxis;
    public double yAxis;

    public JoystickInputs() {
      xAxis = 0.0;
      yAxis = 0.0;
    }
  }
}
