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
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

/** An object that receives telemetry from the CTRE Phoenix 6 SwerveDrivetrain processing thread */
public class CTRESwerveTelemetry {

  /** Set this to true to publish swerve drive values to the dashboard */
  private static final boolean kPublishToDashboard = true;

  /** Most recent robot pose estimated by the swerve subsystem odometry */
  private Pose2d m_lastPose = new Pose2d();

  /** Most recent chassis speeds reported by swerve subsystem odometry */
  private ChassisSpeeds m_lastChassisSpeeds = new ChassisSpeeds();

  /* Keep a reference of the last pose to calculate the speeds */
  private double lastTime = Utils.getCurrentTimeSeconds();

  /** Object used to publish telemetry to the dashboard */
  private final DashboardPublisher m_dashboardPublisher;

  /**
   * Construct a telemetry object, with the specified max speed of the robot
   *
   * @param maxSpeed Maximum speed in meters per second
   */
  public CTRESwerveTelemetry() {
    m_dashboardPublisher = (kPublishToDashboard) ? new DashboardPublisher() : null;
  }

  /** Returns the last pose reported by swerve telemetry */
  public Pose2d getPose() {
    return m_lastPose;
  }

  /** Returns the last ChassisSpeeds reported by swerve telemetry */
  public ChassisSpeeds getChassisSpeeds() {
    return m_lastChassisSpeeds;
  }

  /**
   * Accumulate telemetry data and (optionally) publish swerve data to the dashboard
   *
   * @details This function is registered to be called from the context of the high-rate thread used
   *     to process CTRE SwerveDrive odometry.
   */
  public void update(SwerveDriveState state) {
    // Copy the updated pose out of the swerve state
    Pose2d newPose = new Pose2d(state.Pose.getX(), state.Pose.getY(), state.Pose.getRotation());
    // Copy the chassis speeds out of the swerve state
    ChassisSpeeds newSpeeds =
        new ChassisSpeeds(
            state.speeds.vxMetersPerSecond,
            state.speeds.vxMetersPerSecond,
            state.speeds.omegaRadiansPerSecond);

    synchronized (this) {
      m_lastPose = newPose;
      m_lastChassisSpeeds = newSpeeds;
    }

    if (kPublishToDashboard) {
      /* Telemeterize the robot's general speeds */
      double currentTime = Utils.getCurrentTimeSeconds();
      double diffTime = currentTime - lastTime;
      lastTime = currentTime;
      Translation2d distanceDiff = state.Pose.minus(m_lastPose).getTranslation();

      Translation2d velocities = distanceDiff.div(diffTime);

      m_dashboardPublisher.speed.set(velocities.getNorm());
      m_dashboardPublisher.velocityX.set(velocities.getX());
      m_dashboardPublisher.velocityY.set(velocities.getY());
      m_dashboardPublisher.odomPeriod.set(state.OdometryPeriod);

      m_dashboardPublisher.fieldTypePub.set("Field2d");
      m_dashboardPublisher.fieldPub.set(
          new double[] {newPose.getX(), newPose.getY(), newPose.getRotation().getDegrees()});
    }
  }

  private static class DashboardPublisher {
    /* Robot speeds for general checking */
    public DoublePublisher velocityX;
    public DoublePublisher velocityY;
    public DoublePublisher speed;
    public DoublePublisher odomPeriod;

    public DoubleArrayPublisher fieldPub;
    public StringPublisher fieldTypePub;

    public DashboardPublisher() {
      /* What to publish over networktables for telemetry */
      NetworkTableInstance inst = NetworkTableInstance.getDefault();
      NetworkTable driveStats = inst.getTable("SwerveOdometry");

      velocityX = driveStats.getDoubleTopic("Velocity X").publish();
      velocityY = driveStats.getDoubleTopic("Velocity Y").publish();
      speed = driveStats.getDoubleTopic("Speed").publish();
      odomPeriod = driveStats.getDoubleTopic("Odometry Period").publish();

      NetworkTable table = inst.getTable("SwerveOdometry");
      fieldPub = table.getDoubleArrayTopic("robotPose").publish();
      fieldTypePub = table.getStringTopic(".type").publish();
    }
  }
}
