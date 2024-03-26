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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.joystick.ProcessedXboxController;
import frc.robot.subsystems.swerveCTRE.CommandSwerveDrivetrain;

////////////////////////////////////////////////////////////////////////////////
/**
 * Command class used to control a CTRE swerve drive in Teleoperated mode using an XBox Controller
 */
public class TeleopSwerveCTRE extends Command {
  /** Default maximum linear speed the swerve drive should move at in meters per second */
  // public static final double kMaxSpeed = 6.0;
  public static final double kMaxSpeed = 1.5;

  /** Default maximum rate the swerve drive should rotate in radians per second */
  // public static final double kMaxAngularRate = 10; // Math.PI; Math.Pi was the default
  public static final double kMaxAngularRate = 4; // Math.PI; Math.Pi was the default

  ////////////////////////////////////////
  // Slew rate limiting
  ////////////////////////////////////////
  /**
   * Maximum rate of change applied to X/Y motion (0.0 to 1.0): larger values allow faster changes
   */
  public static final double ktranslationSlewRate = 1.0;

  /**
   * Maximum rate of change applied to X/Y motion (0.0 to 1.0): larger values allow faster changes
   */
  public static final double kRotationSlewRate = 1.0;

  ////////////////////////////////////////
  // Motion deadbanding
  ////////////////////////////////////////

  /** Deadband applied to linear motion as a normalized percentage (0.0 to 1.0) */
  private static final double kSpeedDeadband = 0.1;

  /** Deadband applied to angular rotation as a normalized percentage (0.0 to 1.0) */
  private static final double kAngularRateDeadband = 0.1;

  ////////////////////////////////////////

  /** The swerve drive controlled by this command */
  private final CommandSwerveDrivetrain m_swerve;

  /** XBox controller used to produce commands */
  private final ProcessedXboxController m_controller;

  /** Request object used to control the swerve drive */
  private final SwerveRequest.FieldCentric m_swerveRequest;

  /** Slew rate limiter used to smooth acceleration in the X direction */
  private final SlewRateLimiter m_xRateLimiter = new SlewRateLimiter(1.0 / ktranslationSlewRate);
  /** Slew rate limiter used to smooth acceleration in the Y direction */
  private final SlewRateLimiter m_yRateLimiter = new SlewRateLimiter(1.0 / ktranslationSlewRate);
  /** Slew rate limiter used to smooth rotational acceleration */
  private final SlewRateLimiter m_rotationRateLimiter =
      new SlewRateLimiter(1.0 / kRotationSlewRate);

  /**
   * Creates an instance of the command
   *
   * @param swerve Swerve drive to be controlled
   * @param controller XBox controller used to control the swerve drive
   */
  public TeleopSwerveCTRE(CommandSwerveDrivetrain swerve, ProcessedXboxController controller) {
    addRequirements(swerve);
    m_swerve = swerve;
    m_controller = controller;

    // Set up for driving open-loop using field-centric motion
    m_swerveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(kMaxSpeed * kSpeedDeadband)
            .withRotationalDeadband(kMaxAngularRate * kAngularRateDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = m_controller.getLeftY();
    double ySpeed = m_controller.getLeftX();
    double angularRate = m_controller.getRightX();

    /*double xSpeed = m_xRateLimiter.calculate(-1.0 * m_controller.getLeftY());
    double ySpeed = m_yRateLimiter.calculate(-1.0 * m_controller.getLeftX());
    double angularRate = m_rotationRateLimiter.calculate(-1.0 * m_controller.getRightX());*/

    // Update our SwerveRequest with the requested velocities and apply them to the swerve drive
    m_swerveRequest
        .withVelocityX(xSpeed * kMaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(ySpeed * kMaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(
            angularRate * kMaxAngularRate); // Drive counterclockwise with negative X (left)
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
