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
package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LED.ColorConstants;
import frc.lib.LED.LEDLayer;
import frc.lib.joystick.ProcessedXboxController;
import frc.lib.utility.ZTargeter;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDs.LEDSubsystem.LayerID;
import frc.robot.subsystems.LEDs.LEDSubsystem.StripID;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.CameraConstants.TargetPipeline;
import frc.robot.subsystems.vision.TargetCameraSubsystem;

public class DriveWithZTargeting extends Command {

  /** Set to true to publish values to SmartDashboard */
  public static final boolean kPublishToDashboard = true;

  /** Default maximum linear speed the swerve drive should move at in meters per second */
  public static final double kMaxSpeed = 6.0;

  /** Default maximum rate the swerve drive should rotate in radians per second */
  public static final double kMaxAngularRate = 10; // Math.PI; Math.Pi was the default

  /** Deadband applied to linear motion as a normalized percentage (0.0 to 1.0) */
  private static final double kSpeedDeadband = 0.1;

  /** Deadband applied to angular rotation as a normalized percentage (0.0 to 1.0) */
  private static final double kAngularRateDeadband = 0.1;

  /** Camera subsystem used to provide target detection */
  public final TargetCameraSubsystem m_cameraSubsystem;

  /** The swerve drive controlled by this command */
  private final CommandSwerveDrivetrain m_swerve;

  /** XBox controller used to produce commands */
  private final ProcessedXboxController m_controller;

  /** Request object used to control the swerve drive */
  private final SwerveRequest.FieldCentric m_fieldCentricSwerveReq;

  /** Request object used to drive with bot-centric motion */
  private final SwerveRequest.RobotCentric m_botCentricSwerveReq;

  /** Request object for target */
  private final TargetPipeline m_targetPipeline;

  /** LED layers to display on */
  private final LEDLayer m_leftLEDs;

  private final LEDLayer m_rightLEDs;

  /** ZTargeting Library */
  private final ZTargeter m_zTargeter;

  /**
   * Creates an instance of the command
   *
   * @param swerve Swerve drive to be controlled
   * @param ledSubsystem Subsystem used to access LEDs
   * @param controller XBox controller used to control the swerve drive
   */
  public DriveWithZTargeting(
      RobotContainer botContainer,
      ProcessedXboxController controller,
      TargetPipeline targetPipeline) {
    m_cameraSubsystem = botContainer.targetCameraSubsystem;
    m_swerve = botContainer.driveTrain;
    m_controller = controller;
    m_targetPipeline = targetPipeline;

    addRequirements(
        botContainer.driveTrain, botContainer.ledSubsystem, botContainer.targetCameraSubsystem);

    // Set up a command used to drive open-loop using field-centric motion
    m_fieldCentricSwerveReq =
        new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * kSpeedDeadband)
            .withRotationalDeadband(kMaxAngularRate * kAngularRateDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Set up a command used to drive using robot-centric motion
    m_botCentricSwerveReq =
        new SwerveRequest.RobotCentric()
            .withDeadband(kMaxSpeed * kSpeedDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    m_leftLEDs = botContainer.ledSubsystem.getLayer(StripID.Left, LayerID.Top);
    m_rightLEDs = botContainer.ledSubsystem.getLayer(StripID.Right, LayerID.Top);
    m_zTargeter = new ZTargeter(() -> m_cameraSubsystem.getLatestResult());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_cameraSubsystem.setPipeline(m_targetPipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Default translation to regular swerve X/Y and rotation from the joystick
    double xVelocity = -m_controller.getLeftY() * kMaxSpeed;
    double yVelocity = -m_controller.getLeftX() * kMaxSpeed;
    double angularRate = -m_controller.getRightX() * kMaxAngularRate;
    Color ledColor = ColorConstants.kOff;

    // Get the rotation to a target.  Returns null if no target is found
    Rotation2d zRotation = m_zTargeter.getRotationToTarget();
    boolean targetExists = (zRotation != null);

    // Default to using the regular swerve controller mapping.  If a gamepiece is detected,
    // this will be overridden below
    m_fieldCentricSwerveReq
        .withVelocityX(xVelocity)
        .withVelocityY(yVelocity)
        .withRotationalRate(angularRate);

    String driveMode = "FieldCentric";

    if (!targetExists) {
      // Drive with normal swerve control if no target is present
      m_swerve.driveFieldCentric(m_fieldCentricSwerveReq);
    } else {
      // STATUS: a target is in view of the camera

      // If a note is being targeted, make all the LED's orange.  Otherwise, turn them off
      ledColor = Color.kOrangeRed;

      // Apply rate of rotation from Z-targeting
      double ztAngularRate = zRotation.getRadians() * kMaxAngularRate;
      double rightY = m_controller.getRightY();
      double rightX = m_controller.getRightX();

      if ((Math.abs(rightY) > 0.1) || (Math.abs(rightX) > 0.1)) {
        xVelocity = -rightY * kMaxSpeed;
        yVelocity = rightX * kMaxSpeed;
        angularRate = ztAngularRate;
        // Drive robot-centric if the right joystick is active
        m_swerve.driveRobotCentric(
            m_botCentricSwerveReq
                .withVelocityX(xVelocity)
                .withVelocityY(yVelocity)
                .withRotationalRate(angularRate));
        driveMode = "RobotCentric";

      } else {
        angularRate = ztAngularRate;
        // Drive field-centric using the left joystick if the right joystick is inactive
        m_swerve.driveFieldCentric(
            m_fieldCentricSwerveReq
                .withRotationalRate(angularRate)
                .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      }
    }

    if (kPublishToDashboard) {
      SmartDashboard.putBoolean("DriveWithZTargeting/targetExists", targetExists);
      SmartDashboard.putString("DriveWithZTargeting/driveMode", driveMode);
      SmartDashboard.putNumber("DriveWithZTargeting/xVelocity", xVelocity);
      SmartDashboard.putNumber("DriveWithZTargeting/yVelocity", yVelocity);
      SmartDashboard.putNumber("DriveWithZTargeting/angularRate", angularRate);
    }

    m_leftLEDs.fillColor(ledColor);
    m_rightLEDs.fillColor(ledColor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
