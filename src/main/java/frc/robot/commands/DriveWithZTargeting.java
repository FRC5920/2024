// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonCamera;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.joystick.ProcessedXboxController;
import frc.lib.utility.ZTargeter;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;
import frc.robot.Constants.CameraTarget;
import frc.robot.RobotContainer;
import frc.robot.Constants.CameraInfo.GamePieceCamera;

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

  /** ZTargeting Library */
  private final ZTargeter m_zTargeter;
  
  /**
   * Creates an instance of the command
   *
   * @param swerve Swerve drive to be controlled
   * @param controller XBox controller used to control the swerve drive
   */

  public DriveWithZTargeting(CommandSwerveDrivetrain swerve, ProcessedXboxController controller, CameraTarget target) {
    addRequirements(swerve);
    m_swerve = swerve;
    m_controller = controller;
    m_Target = target;
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
    if (targetExists) {
      angularRate = zRotation.getRadians() * kMaxAngularRate;

      if (m_Target == CameraTarget.GameNote)
      {
        //TODO: Set LEDs Orange
      }
            /* Code for bot relative drive. 
            if ((m_Target == CameraTarget.GameNote)
              && ((Math.abs(m_controller.getRightY()) > 0.1)
                    || (Math.abs(m_controller.getRightX()) > 0.1))) {
              translation =
                  new Translation2d(-m_controller.getRightY(), m_controller.getRightX())
                      .times(RobotContainer.MaxSpeed);
              isFieldRelative = false;
            } else {
              translation = new Translation2d(yAxis, xAxis).times(RobotContainer.MaxSpeed);
              isFieldRelative = true;
            } */
    }

    // Update our SwerveRequest with the requested velocities and apply them to the swerve drive
    m_swerveRequest
        .withVelocityX(xVelocity) // Drive forward with negative Y (forward)
        .withVelocityY(yVelocity) // Drive left with negative X (left)
        .withRotationalRate(angularRate); // Drive counterclockwise with negative X (left)
    m_swerve.driveFieldCentric(m_swerveRequest);
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
