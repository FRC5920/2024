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
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.LED.ColorConstants;
import frc.lib.LED.LEDStrip;
import frc.lib.joystick.ProcessedXboxController;
import frc.lib.utility.ZTargeter;
import frc.robot.Constants.CameraInfo.GamePieceCamera;
import frc.robot.Constants.CameraTarget;
import frc.robot.subsystems.LEDs.LEDSubsystem;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;
import org.photonvision.PhotonCamera;

public class DriveWithZTargeting extends Command {

  public final PhotonCamera ArmCamera = new PhotonCamera(GamePieceCamera.cameraName);

  /** Default maximum linear speed the swerve drive should move at in meters per second */
  public static final double kMaxSpeed = 6.0;

  /** Default maximum rate the swerve drive should rotate in radians per second */
  public static final double kMaxAngularRate = 10; // Math.PI; Math.Pi was the default

  /** Deadband applied to linear motion as a normalized percentage (0.0 to 1.0) */
  private static final double kSpeedDeadband = 0.1;

  /** Deadband applied to angular rotation as a normalized percentage (0.0 to 1.0) */
  private static final double kAngularRateDeadband = 0.1;

  /** The swerve drive controlled by this command */
  private final CommandSwerveDrivetrain m_swerve;

  /** XBox controller used to produce commands */
  private final ProcessedXboxController m_controller;

  /** Request object used to control the swerve drive */
  private final SwerveRequest.FieldCentric m_swerveRequest;

  /** Request object for target */
  private final CameraTarget m_Target;

  /** LED strips on the */
  private final LEDStrip m_leftLEDStrip;

  private final LEDStrip m_rightLEDStrip;

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
      CommandSwerveDrivetrain swerve,
      LEDSubsystem ledSubsystem,
      ProcessedXboxController controller,
      CameraTarget target) {
    addRequirements(swerve, ledSubsystem);
    m_swerve = swerve;
    m_controller = controller;
    m_Target = target;
    m_leftLEDStrip = ledSubsystem.getLeftStrip();
    m_rightLEDStrip = ledSubsystem.getRightStrip();

    // Set up for driving open-loop using field-centric motion
    m_swerveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * kSpeedDeadband)
            .withRotationalDeadband(kMaxAngularRate * kAngularRateDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    m_zTargeter = new ZTargeter(m_Target, ArmCamera);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_zTargeter.initialize(); // Initialize Z-targeting
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVelocity = -m_controller.getLeftY() * kMaxSpeed;
    double yVelocity = -m_controller.getLeftX() * kMaxSpeed;
    double angularRate = -m_controller.getRightX() * kMaxAngularRate;

    // Get the rotation to a target.  Returns null if no target is found
    Rotation2d zRotation = m_zTargeter.getRotationToTarget();
    boolean targetExists = (zRotation != null);
    Color ledColor = ColorConstants.kOff;

    if (targetExists) {
      angularRate = zRotation.getRadians() * kMaxAngularRate;

      // If a note is being targeted, make all the LED's orange.  Otherwise, turn them off
      ledColor = (m_Target == CameraTarget.GameNote) ? Color.kOrange : ledColor;

      // Code for bot relative drive.
      if ((m_Target == CameraTarget.GameNote)
          && ((Math.abs(m_controller.getRightY()) > 0.1)
              || (Math.abs(m_controller.getRightX()) > 0.1))) {
        m_swerve.driveRobotCentric(-m_controller.getRightY(), m_controller.getRightX(), yVelocity);
      } else {
        //
        // Update our SwerveRequest with the requested velocities and apply them to the swerve drive
        m_swerveRequest
            .withVelocityX(xVelocity) // Drive forward with negative Y (forward)
            .withVelocityY(yVelocity) // Drive left with negative X (left)
            .withRotationalRate(angularRate); // Drive counterclockwise with negative X (left)
        m_swerve.driveFieldCentric(m_swerveRequest);
      }
    }

    m_leftLEDStrip.fillColor(ledColor);
    m_rightLEDStrip.fillColor(ledColor);
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
