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
package frc.robot.subsystems.swerveCTRE;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  /** Default maximum linear speed the swerve drive should move at in meters per second */
  public static final double kMaxSpeed = 6.0;

  /** Default maximum rate the swerve drive should rotate in radians per second */
  public static final double kMaxAngularRate = Math.PI;

  /** Deadband applied to linear motion */
  public static final double kSpeedDeadband = 0.1;

  /** Deadband applied to angular rotation */
  public static final double kAngularRateDeadband = 0.1;

  private static final double kSimLoopPeriod = 0.005; // 5 ms

  private static final int kNumSwerveModules = 4;

  public static final String kLogPrefix = "CommandSwerveDrivetrain";
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  private SwerveRequest.RobotCentric m_botCentricSwerveReq = new SwerveRequest.RobotCentric();

  /** Instance of an object used to log inputs fed to the swerve drive base */
  private CTRESwerveInputsAutoLogged m_loggedInputs = new CTRESwerveInputsAutoLogged();

  /** m_motorOutputLogger is used to log swerve motor measurements as outputs */
  private final CTRESwerveOutputLogger m_loggedOutputs;

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    m_loggedOutputs = new CTRESwerveOutputLogger(kLogPrefix, this, kNumSwerveModules);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    m_loggedOutputs = new CTRESwerveOutputLogger(kLogPrefix, this, kNumSwerveModules);

    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  @Override
  public void periodic() {
    // Process and log inputs
    Logger.processInputs(kLogPrefix + "/swerveInputs", m_loggedInputs);

    // Log swerve outputs
    m_loggedOutputs.toLog();
  }

  // Returns the current estimated pose of the robot
  public Pose2d getPose() {
    return this.getState().Pose;
  }

  // Returns the current estimated pose of the robot
  public ChassisSpeeds getChassisSpeeds() {
    return this.getState().speeds;
  }

  /**
   * Applies a field-centric control request to the swerve drivetrain.
   *
   * @details This method logs field-centric control values issued to the swerve drive
   * @param request The request to apply
   */
  public void driveFieldCentric(SwerveRequest.FieldCentric request) {
    m_loggedInputs.requestType = SwerveRequest.FieldCentric.class.getName();
    m_loggedInputs.xVelocity = request.VelocityX;
    m_loggedInputs.yVelocity = request.VelocityY;
    m_loggedInputs.angularRate = request.RotationalRate;

    this.setControl(request);
  }

  /**
   * Applies a robot-centric control request to the swerve drivetrain.
   *
   * @param botCentricReq Robot-centric swerve request with velocities
   */
  public void driveRobotCentric(SwerveRequest.RobotCentric botCentricReq) {
    m_loggedInputs.requestType = SwerveRequest.RobotCentric.class.getName();
    m_loggedInputs.xVelocity = botCentricReq.VelocityX;
    m_loggedInputs.yVelocity = botCentricReq.VelocityY;
    m_loggedInputs.angularRate = botCentricReq.RotationalRate;

    this.setControl(botCentricReq);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /** Returns the radius of the drive base */
  public double getDriveBaseRadius() {
    double driveBaseRadius = 0;
    for (var moduleLocation : m_moduleLocations) {
      driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
    }

    return driveBaseRadius;
  }
}
